"""Import this on the laptop (GPU side): frames in, velocity commands out.

    from robot_client import RoboMasterLink

    with RoboMasterLink("192.168.0.123") as link:      # the Pi's IP
        while running:
            frame = link.read_frame(timeout=1.0)       # newest BGR ndarray
            if frame is None:
                continue
            x, z = my_gpu_model(frame)
            link.drive(x=x, z=z)                       # returns immediately

read_frame() always hands back the newest frame and decodes on demand, so a
slow GPU loop falls behind in frames, never in latency. drive() only updates a
setpoint; a background thread repeats it at a fixed rate, so the robot keeps
moving smoothly between iterations and the server's watchdog stops it if this
process dies.

Standalone check (drives the robot, keep it clear of obstacles):

    python robot_client.py --host 192.168.0.123 --view
    python robot_client.py --host 192.168.0.123 --demo
"""
import argparse
import json
import socket
import struct
import threading
import time

import cv2
import numpy

FRAME_HEADER = struct.Struct("!IdI")


class RoboMasterLink:
    def __init__(self, host, cmd_port=9812, video_port=9813, rate_hz=20.0, video=True):
        self._host = host
        self._cmd_addr = (host, cmd_port)
        self._video_addr = (host, video_port)
        self._period = 1.0 / rate_hz
        self._running = True

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(1.0)

        self._lock = threading.Lock()
        self._setpoint = (0.0, 0.0, 0.0)
        self._seq = 0
        self._sent_at = {}
        self._last_ack = None
        self._last_ack_at = 0.0
        self._rtt = None

        self._frame_lock = threading.Condition()
        self._frame = None            # (jpeg bytes, capture time, frame id)
        self._frame_count = 0
        self._dropped = 0
        self._last_read_id = 0
        self._last_meta = None

        self._threads = [
            threading.Thread(target=self._send_task, daemon=True),
            threading.Thread(target=self._recv_task, daemon=True),
        ]
        if video:
            self._threads.append(threading.Thread(target=self._video_task, daemon=True))
        for thread in self._threads:
            thread.start()

    # -- control ---------------------------------------------------------
    def drive(self, x=0.0, y=0.0, z=0.0):
        """Set the velocity: x/y in m/s (forward/strafe), z in deg/s (yaw)."""
        with self._lock:
            self._setpoint = (float(x), float(y), float(z))

    def stop(self):
        self.drive(0.0, 0.0, 0.0)
        self._send({"stop": True})

    def close(self):
        try:
            self.stop()
            time.sleep(0.05)
        finally:
            self._running = False
            self._sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -- video -----------------------------------------------------------
    def read_frame(self, timeout=1.0, newest_only=True, with_meta=False):
        """Newest frame as a BGR ndarray, or None if none arrived in time.

        With with_meta=True, returns (frame, meta) where meta holds
        captured_at (the Pi's clock, seconds) and frame_id. Consecutive
        captured_at values come from the same clock, so differences are a
        sound dt even though the absolute value is offset from this machine's.

        Decoding happens here rather than in the receive thread, so frames the
        caller never consumes cost nothing but bandwidth.
        """
        deadline = time.monotonic() + timeout
        with self._frame_lock:
            while self._frame is None or (newest_only and self._frame[2] <= self._last_read_id):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return (None, None) if with_meta else None
                self._frame_lock.wait(remaining)
            payload, captured_at, frame_id = self._frame
            self._last_read_id = frame_id
        meta = {"captured_at": captured_at, "frame_id": frame_id}
        self._last_meta = meta
        frame = cv2.imdecode(numpy.frombuffer(payload, dtype=numpy.uint8), cv2.IMREAD_COLOR)
        return (frame, meta) if with_meta else frame

    @property
    def last_frame_meta(self):
        """Metadata for the most recent frame read, or None."""
        return self._last_meta

    @property
    def frames_received(self):
        return self._frame_count

    @property
    def frames_skipped(self):
        """Frames that arrived but were superseded before being read."""
        return self._dropped

    # -- status ----------------------------------------------------------
    @property
    def rtt(self):
        """Command round trip to the Pi in seconds, or None."""
        return self._rtt

    @property
    def connected(self):
        return self._last_ack_at > 0 and (time.monotonic() - self._last_ack_at) < 2.0

    @property
    def battery(self):
        return (self._last_ack or {}).get("battery")

    # -- telemetry -------------------------------------------------------
    # Served by the chassis DDS subscriptions on the Pi and piggybacked on
    # every command ack, so these cost no extra traffic. All are None until
    # the first sample arrives, and stay at their last value if the
    # subscription dies -- check the *_age properties before trusting them.

    @property
    def velocity_body(self):
        """Chassis velocity in the body frame, m/s, as (3,) float array."""
        return self._vector("vel", "body")

    @property
    def velocity_world(self):
        """Chassis velocity in the power-on world frame, m/s, as (3,) array."""
        return self._vector("vel", "world")

    @property
    def accel(self):
        """IMU acceleration, (3,) array. Units are the firmware's own: the SDK
        rounds but does not rescale. At rest, |accel| near 1.0 means g, near
        9.81 means m/s^2 -- check once and convert in your own code."""
        return self._vector("imu", "accel")

    @property
    def gyro(self):
        """IMU angular rate, (3,) array, firmware units (rad/s as documented)."""
        return self._vector("imu", "gyro")

    @property
    def velocity_age(self):
        """Seconds since the velocity sample was taken, or None."""
        return self._age("vel")

    @property
    def imu_age(self):
        """Seconds since the IMU sample was taken, or None."""
        return self._age("imu")

    @property
    def telemetry(self):
        """Everything from the last ack: velocity, imu, ages, battery, rtt."""
        ack = self._last_ack or {}
        return {"battery": ack.get("battery"), "rtt": self._rtt,
                "velocity_body": self.velocity_body,
                "velocity_world": self.velocity_world,
                "accel": self.accel, "gyro": self.gyro,
                "velocity_age": self.velocity_age, "imu_age": self.imu_age}

    def _vector(self, group, key):
        values = (self._last_ack or {}).get(group, {}).get(key)
        if values is None:
            return None
        return numpy.asarray(values, dtype=float)

    def _age(self, group):
        block = (self._last_ack or {}).get(group)
        if block is None or self._last_ack_at == 0.0:
            return None
        # Age measured on the Pi when the ack was built, plus the time since
        # that ack landed here. Both terms are local to one clock.
        return block.get("age", 0.0) + (time.monotonic() - self._last_ack_at)

    # -- internals -------------------------------------------------------
    def _send(self, payload):
        with self._lock:
            self._seq += 1
            payload["seq"] = self._seq
        seq = payload["seq"]
        self._sent_at[seq] = time.monotonic()
        for old in [k for k in self._sent_at if k < seq - 64]:
            del self._sent_at[old]
        try:
            self._sock.sendto(json.dumps(payload).encode("utf-8"), self._cmd_addr)
        except OSError:
            pass

    def _send_task(self):
        while self._running:
            start = time.monotonic()
            with self._lock:
                x, y, z = self._setpoint
            self._send({"x": x, "y": y, "z": z})
            slack = self._period - (time.monotonic() - start)
            if slack > 0:
                time.sleep(slack)

    def _recv_task(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(1024)
            except (socket.timeout, OSError):
                continue
            try:
                ack = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, ValueError):
                continue
            now = time.monotonic()
            sent_at = self._sent_at.pop(ack.get("seq"), None)
            if sent_at is not None:
                self._rtt = now - sent_at
            self._last_ack = ack
            self._last_ack_at = now

    def _video_task(self):
        while self._running:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            try:
                sock.connect(self._video_addr)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                sock.close()
                time.sleep(1.0)      # server not up yet; keep trying
                continue
            try:
                while self._running:
                    header = self._recv_exact(sock, FRAME_HEADER.size)
                    if header is None:
                        break
                    length, captured_at, frame_id = FRAME_HEADER.unpack(header)
                    payload = self._recv_exact(sock, length)
                    if payload is None:
                        break
                    with self._frame_lock:
                        if self._frame is not None and self._frame[2] > self._last_read_id:
                            self._dropped += 1
                        self._frame = (payload, captured_at, frame_id)
                        self._frame_count += 1
                        self._frame_lock.notify_all()
            except OSError:
                pass
            finally:
                sock.close()

    @staticmethod
    def _recv_exact(sock, count):
        chunks = []
        remaining = count
        while remaining:
            try:
                chunk = sock.recv(min(remaining, 65536))
            except socket.timeout:
                return None
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)


def view(host, cmd_port, video_port):
    with RoboMasterLink(host, cmd_port, video_port) as link:
        print("press q to quit")
        while True:
            frame = link.read_frame(timeout=2.0)
            if frame is None:
                print("no frame; is robot_server.py running with a camera?")
                continue
            cv2.putText(frame, "rtt {0}  batt {1}  frames {2} (skipped {3})".format(
                "-" if link.rtt is None else "{0:.0f}ms".format(link.rtt * 1000),
                link.battery, link.frames_received, link.frames_skipped),
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("robomaster", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()


def demo(host, cmd_port, video_port):
    moves = [("forward", 0.3, 0.0, 0.0), ("stop", 0, 0, 0),
             ("back", -0.3, 0.0, 0.0), ("stop", 0, 0, 0),
             ("rotate", 0.0, 0.0, 45.0), ("stop", 0, 0, 0)]
    with RoboMasterLink(host, cmd_port, video_port, video=False) as link:
        time.sleep(0.5)
        if not link.connected:
            print("no ack from the Pi; is robot_server.py running?")
            return
        print("connected, rtt {0:.0f}ms, battery {1}".format(
            (link.rtt or 0) * 1000, link.battery))
        for name, x, y, z in moves:
            print("{0:<8} x={1} y={2} z={3}".format(name, x, y, z))
            link.drive(x=x, y=y, z=z)
            time.sleep(1.0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True, help="the Pi's IP address")
    parser.add_argument("--cmd-port", type=int, default=9812)
    parser.add_argument("--video-port", type=int, default=9813)
    parser.add_argument("--view", action="store_true", help="show the video stream")
    parser.add_argument("--demo", action="store_true", help="drive a short test pattern")
    args = parser.parse_args()
    if args.demo:
        demo(args.host, args.cmd_port, args.video_port)
    elif args.view:
        view(args.host, args.cmd_port, args.video_port)
    else:
        with RoboMasterLink(args.host, args.cmd_port, args.video_port) as link:
            time.sleep(1.0)
            print("connected:", link.connected, "rtt:", link.rtt,
                  "battery:", link.battery, "frames:", link.frames_received)
