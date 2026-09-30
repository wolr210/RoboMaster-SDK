"""Listen for the robot's IP broadcast while it is in router (networking) mode.

Run it from any machine on the same LAN as the robot. The robot broadcasts its
serial number, so a match against --sn is definitive.

  python pi/find_robot.py --sn 3JKCK6U0030AJ5
"""
import argparse
import binascii
import select
import socket
import time

# 40927 is config.ROBOT_BROADCAST_PORT; 45678 is the port older firmware used
# (and what ConnectionHelper.wait_for_connection listens on).
PORTS = [40927, 45678]


def open_socket(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", port))
    except OSError as exc:
        print("port {0}: cannot bind ({1})".format(port, exc))
        sock.close()
        return None
    return sock


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sn", default=None, help="stop as soon as this serial is seen")
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()

    socks = [s for s in (open_socket(p) for p in PORTS) if s]
    if not socks:
        raise SystemExit("no broadcast port could be bound")
    print("listening on {0} for {1:.0f}s ...".format(
        [s.getsockname()[1] for s in socks], args.timeout))

    seen = {}
    deadline = time.time() + args.timeout
    try:
        while time.time() < deadline:
            ready, _, _ = select.select(socks, [], [], 1.0)
            for sock in ready:
                data, addr = sock.recvfrom(1024)
                serial = data.split(b"\x00")[0].decode("utf-8", "replace")
                key = (addr[0], serial)
                if key in seen:
                    continue
                seen[key] = True
                print("{0:<16} port={1:<6} sn={2!r} raw={3}".format(
                    addr[0], sock.getsockname()[1], serial,
                    binascii.hexlify(data[:24]).decode()))
                if args.sn and serial == args.sn:
                    print("\nmatch: robot is at {0}".format(addr[0]))
                    print("run: python pi/smoke_test.py --conn sta --robot-ip {0}".format(addr[0]))
                    return
    finally:
        for sock in socks:
            sock.close()

    if not seen:
        print("nothing heard; the robot is probably not on this network")


if __name__ == '__main__':
    main()
