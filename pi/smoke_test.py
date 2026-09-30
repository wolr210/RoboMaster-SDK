"""Verify Pi -> RoboMaster command control (and optionally the USB webcam).

Read-only by default. Pass --move to actually drive the chassis.

  python pi_smoke_test.py --conn rndis
  python pi_smoke_test.py --conn ap --move --camera 0
  python pi_smoke_test.py --conn sta --robot-ip 192.168.1.42
"""
import argparse
import time

from robomaster import config
from robomaster import robot


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--conn", default="rndis", choices=["rndis", "ap", "sta"])
    parser.add_argument("--proto", default="udp", choices=["udp", "tcp"])
    parser.add_argument("--robot-ip", default=None, help="robot IP, for --conn sta")
    parser.add_argument("--local-ip", default=None,
                        help="this machine's IP; set it if the Pi resolves its "
                             "hostname to 127.0.1.1 and the handshake misbehaves")
    parser.add_argument("--move", action="store_true", help="drive a 0.3 m square-ish nudge")
    parser.add_argument("--camera", type=int, default=None, help="webcam index to grab a frame from")
    args = parser.parse_args()

    if args.robot_ip:
        config.ROBOT_IP_STR = args.robot_ip
    if args.local_ip:
        config.LOCAL_IP_STR = args.local_ip

    ep = robot.Robot()
    ep.initialize(conn_type=args.conn, proto_type=args.proto)
    try:
        print("robot version :", ep.get_version())
        print("robot sn      :", ep.get_sn())
        print("chassis fw    :", ep.chassis.get_version())
        print("battery fw    :", ep.battery.get_version())

        # DDS push path: the robot has to reach us, not just answer us.
        seen = []
        ep.battery.sub_battery_info(freq=1, callback=seen.append)
        time.sleep(3)
        ep.battery.unsub_battery_info()
        print("battery push  :", seen if seen else "NO DATA (robot could not reach this host)")

        if args.move:
            print("moving forward 0.3 m ...")
            ep.chassis.move(x=0.3, y=0, z=0, xy_speed=0.3).wait_for_completed()
            print("moving back 0.3 m ...")
            ep.chassis.move(x=-0.3, y=0, z=0, xy_speed=0.3).wait_for_completed()
            print("move done")
        else:
            print("(skipping motion; pass --move to drive)")
    finally:
        ep.close()

    if args.camera is not None:
        import cv2
        cap = cv2.VideoCapture(args.camera)
        try:
            if not cap.isOpened():
                print("webcam {0}: could not open".format(args.camera))
            else:
                ok, frame = cap.read()
                print("webcam {0}: read={1} shape={2}".format(
                    args.camera, ok, None if frame is None else frame.shape))
        finally:
            cap.release()


if __name__ == '__main__':
    main()
