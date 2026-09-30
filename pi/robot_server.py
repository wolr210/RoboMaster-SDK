"""Run on the Pi: stream webcam frames out, take velocity setpoints in.

  python pi/robot_server.py --conn sta --robot-ip 192.168.0.205 --camera 0

Two sockets, because the two directions want opposite things:

  udp/9812   velocity setpoints in, acks out. Latest-wins; a lost packet is
             superseded by the next one, so retransmission would only add lag.
  tcp/9813   JPEG frames out. Length-prefixed, and the sender keeps only the
             newest frame, so a stalled link drops frames instead of building
             a backlog of stale ones.

Velocity packets are JSON:
  {"seq": 12, "x": 0.4, "y": 0.0, "z": 30.0}   x/y m/s, z deg/s
  {"seq": 13, "stop": true}

Frames are: 16-byte header (!IdI = length, capture time, frame id) + JPEG.
"""
import argparse
import json
import socket
import struct
import threading
import time

import cv2

from robomaster import config
from robomaster import robot

FRAME_HEADER = struct.Struct("!IdI")

# drive_speed limits: x/y m/s, z deg/s. Out-of-range values raise inside the
# SDK's checkers, so clamp before handing anything over.
X_LIMIT = 3.5
Y_LIMIT = 3.5
Z_LIMIT = 600.0


def clamp(value, limit):
    return max(-limit, min(limit, float(value)))


class Camera:
    """Capture thread that only ever holds the newest frame."""

    def __init__(self, index, width, height, fps, quality):
        self._quality = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        self._cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            self._cap = cv2.VideoCapture(index)
        if not self._cap.isOpened():
            raise SystemExit("could not open camera {0}".format(index))
        # MJPG keeps USB bandwidth down at higher resolutions, and a 1-frame
        # buffer stops the driver handing us stale frames.
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)
        try:
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        self._lock = threading.Lock()
        self._latest = None          # (jpeg bytes, capture time, frame id)
        self._new_frame = threading.Condition(self._lock)
        self._count = 0
        self._running = True
        self._thread = threading.Thread(target=self._task, daemon=True)
        self._thread.start()
        print("camera {0}: {1}x{2} @ {3}".format(
            index, int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            self._cap.get(cv2.CAP_PROP_FPS)))

    def _task(self):
        while self._running:
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            ok, buf = cv2.imencode(".jpg", frame, self._quality)
            if not ok:
                continue
            with self._new_frame:
                self._count += 1
                self._latest = (buf.tobytes(), time.time(), self._count)
                self._new_frame.notify_all()

    def wait_for_frame(self, last_id, timeout=1.0):
        """Block until a frame newer than last_id exists, then return it."""
        with self._new_frame:
            if self._latest is None or self._latest[2] <= last_id:
                self._new_frame.wait(timeout)
            if self._latest is None or self._latest[2] <= last_id:
                return None
            return self._latest

    def close(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self._cap.release()


class Telemetry:
    """Latest chassis velocity and IMU sample, fed by the SDK's DDS pushes.

    Ages are computed here, on the Pi, so the client never has to compare
    clocks across machines.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._velocity = None        # (vgx, vgy, vgz, vbx, vby, vbz), t
        self._imu = None             # (ax, ay, az, gx, gy, gz), t

    def on_velocity(self, info):
        with self._lock:
            self._velocity = (tuple(round(float(v), 5) for v in info), time.monotonic())

    def on_imu(self, info):
        with self._lock:
            self._imu = (tuple(round(float(v), 5) for v in info), time.monotonic())

    def snapshot(self):
        now = time.monotonic()
        out = {}
        with self._lock:
            if self._velocity:
                values, stamped = self._velocity
                out["vel"] = {"world": values[0:3], "body": values[3:6],
                              "age": round(now - stamped, 4)}
            if self._imu:
                values, stamped = self._imu
                out["imu"] = {"accel": values[0:3], "gyro": values[3:6],
                              "age": round(now - stamped, 4)}
        return out


class VelocityControl:
    """Holds the setpoint and talks to the robot on its own clock.

    chassis.drive_speed() waits for an ack from the robot and can block for
    seconds, which must never stall the socket loops.
    """

    def __init__(self, ep_robot, rate_hz, watchdog_s):
        self._chassis = ep_robot.chassis
        self._period = 1.0 / rate_hz
        self._watchdog = watchdog_s

        self._lock = threading.Lock()
        self._setpoint = (0.0, 0.0, 0.0)
        self._setpoint_at = 0.0
        self._last_seq = -1
        self._battery = None
        self._stopped = True
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._task, daemon=True)
        self._thread.start()

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        try:
            self._chassis.drive_speed(0, 0, 0)
        except Exception as exc:
            print("final stop failed:", exc)

    def set_battery(self, value):
        self._battery = value

    def apply(self, packet):
        seq = int(packet.get("seq", -1))
        with self._lock:
            if 0 <= seq < self._last_seq:      # UDP reordering
                return {"seq": seq, "t": time.time(), "stale": True,
                        "battery": self._battery}
            self._last_seq = max(seq, self._last_seq)
            if packet.get("stop"):
                self._setpoint = (0.0, 0.0, 0.0)
            else:
                self._setpoint = (clamp(packet.get("x", 0.0), X_LIMIT),
                                  clamp(packet.get("y", 0.0), Y_LIMIT),
                                  clamp(packet.get("z", 0.0), Z_LIMIT))
            self._setpoint_at = time.monotonic()
        return {"seq": seq, "t": time.time(), "battery": self._battery}

    def _task(self):
        while self._running:
            loop_start = time.monotonic()
            with self._lock:
                x, y, z = self._setpoint
                age = loop_start - self._setpoint_at

            if age > self._watchdog:
                if not self._stopped:
                    print("watchdog: no setpoint for {0:.1f}s, stopping".format(age))
                    self._drive(0.0, 0.0, 0.0)
                    self._stopped = True
            else:
                self._drive(x, y, z)
                self._stopped = (x == 0.0 and y == 0.0 and z == 0.0)

            slack = self._period - (time.monotonic() - loop_start)
            if slack > 0:
                time.sleep(slack)

    def _drive(self, x, y, z):
        try:
            self._chassis.drive_speed(x=x, y=y, z=z)
        except Exception as exc:
            print("drive_speed failed:", exc)


def video_server(camera, port, stop_event):
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("0.0.0.0", port))
    listener.listen(1)
    listener.settimeout(1.0)
    print("video: listening on tcp/{0}".format(port))

    while not stop_event.is_set():
        try:
            client, addr = listener.accept()
        except socket.timeout:
            continue
        except OSError:
            break
        print("video: client {0} connected".format(addr))
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        last_id = 0
        try:
            while not stop_event.is_set():
                frame = camera.wait_for_frame(last_id, timeout=1.0)
                if frame is None:
                    continue
                payload, captured_at, frame_id = frame
                last_id = frame_id
                client.sendall(FRAME_HEADER.pack(len(payload), captured_at, frame_id))
                client.sendall(payload)
        except (OSError, socket.timeout) as exc:
            print("video: client gone ({0})".format(exc))
        finally:
            client.close()
    listener.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--conn", default="sta", choices=["rndis", "ap", "sta"])
    parser.add_argument("--proto", default="udp", choices=["udp", "tcp"])
    parser.add_argument("--robot-ip", default=None)
    parser.add_argument("--cmd-port", type=int, default=9812)
    parser.add_argument("--video-port", type=int, default=9813)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--quality", type=int, default=80, help="JPEG quality, 1-100")
    parser.add_argument("--rate", type=float, default=20.0, help="commands/s to the robot")
    parser.add_argument("--watchdog", type=float, default=0.5,
                        help="stop the robot if no setpoint arrives for this long")
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--telemetry-freq", type=int, default=20, choices=[1, 5, 10, 20, 50],
                        help="chassis velocity/IMU push rate, Hz (DDS accepts only these)")
    parser.add_argument("--no-telemetry", action="store_true",
                        help="skip the velocity/IMU subscriptions")
    args = parser.parse_args()

    if args.robot_ip:
        config.ROBOT_IP_STR = args.robot_ip

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type=args.conn, proto_type=args.proto)
    print("connected to robot:", ep_robot.get_sn())

    control = VelocityControl(ep_robot, args.rate, args.watchdog)
    try:
        ep_robot.battery.sub_battery_info(freq=1, callback=control.set_battery)
    except Exception as exc:
        print("battery subscription failed (not fatal):", exc)

    telemetry = Telemetry()
    if not args.no_telemetry:
        for name, subscribe, handler in (
                ("velocity", ep_robot.chassis.sub_velocity, telemetry.on_velocity),
                ("imu", ep_robot.chassis.sub_imu, telemetry.on_imu)):
            try:
                subscribe(freq=args.telemetry_freq, callback=handler)
                print("subscribed to chassis {0} at {1} Hz".format(name, args.telemetry_freq))
            except Exception as exc:
                print("{0} subscription failed (not fatal): {1}".format(name, exc))
    control.start()

    stop_event = threading.Event()
    camera = None
    video_thread = None
    if not args.no_video:
        camera = Camera(args.camera, args.width, args.height, args.fps, args.quality)
        video_thread = threading.Thread(
            target=video_server, args=(camera, args.video_port, stop_event), daemon=True)
        video_thread.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.cmd_port))
    sock.settimeout(1.0)
    print("commands: listening on udp/{0}, {1:.0f} Hz to the robot, {2:.1f}s watchdog"
          .format(args.cmd_port, args.rate, args.watchdog))

    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                continue
            try:
                packet = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, ValueError):
                continue
            ack = control.apply(packet)
            ack.update(telemetry.snapshot())
            try:
                sock.sendto(json.dumps(ack).encode("utf-8"), addr)
            except OSError:
                pass
    except KeyboardInterrupt:
        print("\nshutting down")
    finally:
        stop_event.set()
        control.close()
        sock.close()
        if camera:
            camera.close()
        if video_thread:
            video_thread.join(timeout=2.0)
        for unsubscribe in (ep_robot.battery.unsub_battery_info,
                            ep_robot.chassis.unsub_velocity,
                            ep_robot.chassis.unsub_imu):
            try:
                unsubscribe()
            except Exception:
                pass
        ep_robot.close()


if __name__ == '__main__':
    main()
