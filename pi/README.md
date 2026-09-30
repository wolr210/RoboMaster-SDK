# RoboMaster over a Raspberry Pi link

Run the robot from a machine that isn't attached to it: a Pi holds the robot
connection and a USB webcam, a workstation runs the heavy vision/ML code, and
the two talk over the network.

```
  workstation (GPU)                          Raspberry Pi                 robot
  ┌────────────────────┐                ┌──────────────────────┐      ┌─────────┐
  │ your control loop  │                │ robot_server.py      │      │ EP      │
  │  robot_client.py   │  udp/9812 ───► │  velocity setpoints  │ ───► │ chassis │
  │  RoboMasterLink    │  ◄─── tcp/9813 │  JPEG frames         │      │         │
  └────────────────────┘                │  USB webcam          │      └─────────┘
                                        └──────────────────────┘
```

Two files matter:

| File | Runs on | Purpose |
|---|---|---|
| `pi/robot_server.py` | Pi | Holds the robot session + camera; serves frames, accepts velocity |
| `pi/robot_client.py` | workstation | `RoboMasterLink`: `read_frame()` / `drive()` |

Supporting scripts: `pi/setup.sh` (install), `pi/smoke_test.py` (verify the
robot link), `pi/find_robot.py` (locate the robot on a LAN).

---

## 1. Install on the Pi

Raspberry Pi OS 64-bit (Debian trixie, Python 3.13). From a clone of this repo:

```bash
git clone <your-fork-url> ~/RoboMaster-SDK
cd ~/RoboMaster-SDK
bash pi/setup.sh                 # optional arg: venv path, default ~/rm-venv
source ~/rm-venv/bin/activate
```

`setup.sh` installs `build-essential cmake pkg-config python3-dev python3-venv`
and the FFmpeg dev libraries, creates a virtualenv (Raspberry Pi OS enforces
PEP 668, so a venv is mandatory), builds `libmedia_codec`, installs the
`robomaster` package, and verifies both import.

**Why `libmedia_codec` is built even with no robot camera:** `robomaster/media.py`
imports it at module scope and `robot.py` → `camera.py` → `media.py`, so
`import robomaster.robot` fails without it. It does not need to work at runtime
for chassis control.

Every command below assumes the venv is active (`source ~/rm-venv/bin/activate`).

### Reinstalling after a code change

`setup.sh` installs non-editable, so a `git pull` alone changes nothing:

```bash
cd ~/RoboMaster-SDK && git pull
pip install --force-reinstall --no-deps .          # Python changes
pip install --force-reinstall --no-deps ./lib/libmedia_codec   # C++ changes
```

For active development, `pip install -e .` makes pulls take effect immediately.

---

## 2. Connect the Pi to the robot

Three transports, set with `--conn`:

| `--conn` | Setup | Notes |
|---|---|---|
| `sta` | Robot and Pi on the same router | **Preferred.** Pi keeps internet/Pi Connect. Needs `--robot-ip` |
| `ap` | Pi joins the robot's own Wi-Fi | Robot is always `192.168.2.1`; consumes the Pi's Wi-Fi radio |
| `rndis` | USB cable, Pi is USB host | Pi must be at `192.168.42.3`, robot at `192.168.42.2` |

### Router mode (`sta`)

The robot must already be paired to the network (the pairing itself needs the
robot's camera to scan a QR code — see *Limitations*). Find it:

```bash
python pi/find_robot.py --sn <14-char serial>
```

It listens on UDP 40927 and 45678 for the robot's address broadcast and prints
the IP. The router's DHCP table works too.

### AP mode

Join the robot's `RM<serial>` Wi-Fi, then use `--conn ap`. If the Pi also has
Ethernet, stop the robot's network from becoming the default route:

```bash
sudo nmcli connection modify "RM<serial>" ipv4.never-default yes ipv6.never-default yes
```

### USB (`rndis`)

The Pi must be the USB **host**: its USB-A port to the robot's USB port. The
Pi's own USB-C is power/device only — it cannot host, and powering the Pi from
the robot browns it out. Then:

```bash
sudo ip addr add 192.168.42.3/24 dev usb0 && sudo ip link set usb0 up
ping -c2 192.168.42.2
```

The address must be exactly `192.168.42.3`: the SDK hardcodes it as the address
it registers with the robot, so DDS pushes go nowhere if the Pi is elsewhere.

### Verify before going further

```bash
python pi/smoke_test.py --conn sta --robot-ip 192.168.0.205
python pi/smoke_test.py --conn sta --robot-ip 192.168.0.205 --move   # drives 0.3 m
```

Prints robot/chassis/battery firmware versions and subscribes to battery push
for 3 s. **The battery push line is the important one** — it requires the robot
to reach *back* to the Pi, which client isolation on a router will block even
when commands work fine.

---

## 3. Run the server on the Pi

```bash
python pi/robot_server.py --conn sta --robot-ip 192.168.0.205 --camera 0
```

| Flag | Default | Meaning |
|---|---|---|
| `--conn` | `sta` | `sta` / `ap` / `rndis` |
| `--robot-ip` | – | Robot IP, for `sta` |
| `--proto` | `udp` | SDK command transport to the robot |
| `--cmd-port` | `9812` | UDP port for velocity setpoints |
| `--video-port` | `9813` | TCP port for frames |
| `--camera` | `0` | `cv2.VideoCapture` index of the USB webcam |
| `--width` / `--height` | `640` / `480` | Capture resolution |
| `--fps` | `30` | Requested camera rate |
| `--quality` | `80` | JPEG quality, 1–100 |
| `--rate` | `20` | Velocity commands/s sent to the robot |
| `--watchdog` | `0.5` | Stop the robot if no setpoint arrives for this long (s) |
| `--no-video` | off | Control only, no camera |
| `--telemetry-freq` | `20` | Chassis velocity/IMU push rate, Hz. DDS accepts only 1/5/10/20/50 |
| `--no-telemetry` | off | Skip the velocity/IMU subscriptions |

It prints the negotiated camera format on start. Ctrl-C stops the robot,
unsubscribes, and closes the robot session.

Run it under systemd or `tmux` if you want it to survive an SSH disconnect.

---

## 4. Client API

Copy `pi/robot_client.py` next to your code, or add `pi/` to `sys.path`. It
needs only `opencv-python` and `numpy`; it does **not** need the `robomaster`
package, so the workstation needs no SDK install.

```python
from robot_client import RoboMasterLink

with RoboMasterLink("192.168.0.123") as link:   # the Pi's IP
    while running:
        frame = link.read_frame(timeout=1.0)
        if frame is None:
            continue
        vx, wz = my_controller(frame)
        link.drive(x=vx, z=wz)
```

### Constructor

```python
RoboMasterLink(host, cmd_port=9812, video_port=9813, rate_hz=20.0, video=True)
```

- `host` — the **Pi's** IP, not the robot's.
- `rate_hz` — how often the background thread repeats the current setpoint.
  Keep it above `1 / watchdog` on the server.
- `video=False` — skip the video thread entirely for control-only use.

Threads start in the constructor and are daemons. Use it as a context manager,
or call `close()` yourself.

### Control

| Method | Behaviour |
|---|---|
| `drive(x=0.0, y=0.0, z=0.0)` | Set the velocity setpoint. Returns immediately; never blocks or raises |
| `stop()` | Zero the setpoint and send an explicit stop |
| `close()` | Stop, then shut the link down (called by `__exit__`) |

**Units are identical to `chassis.drive_speed()`:** `x` forward m/s, `y` right
(strafe) m/s, `z` yaw **degrees/second**. Clamped server-side to ±3.5 m/s and
±600 °/s. If your controller produces rad/s, convert: `z_deg = math.degrees(wz)`.

`drive()` sets a *setpoint*, not a pulse. The robot holds that velocity until
you change it, you stop, or the watchdog fires. It is safe to call at any rate,
including every iteration of a slow loop.

### Video

```python
frame = link.read_frame(timeout=1.0)                      # BGR ndarray or None
frame, meta = link.read_frame(timeout=1.0, with_meta=True)
```

| Parameter | Default | Meaning |
|---|---|---|
| `timeout` | `1.0` | Seconds to wait for a frame newer than the last one read |
| `newest_only` | `True` | Only return frames not yet returned; `False` re-returns the current one |
| `with_meta` | `False` | Return `(frame, meta)` instead of `frame` |

`meta` is `{"captured_at": float, "frame_id": int}`. `captured_at` is the **Pi's**
wall clock at capture. Differences between consecutive frames are a sound `dt`
(same clock); the absolute value is offset from the workstation's clock unless
both are NTP-synced, so don't use it for one-way latency.

Returns `None` (or `(None, None)`) on timeout — always check before using.

### Status

| Property | Meaning |
|---|---|
| `connected` | An ack arrived from the Pi within 2 s |
| `rtt` | Last command round trip, seconds, or `None` |
| `battery` | Robot battery percent from the last ack, or `None` |
| `frames_received` | Frames received since start |
| `frames_skipped` | Frames superseded before being read — how far your loop lags |
| `last_frame_meta` | `meta` for the most recent `read_frame()` |

Gate your loop on `connected`; it goes `False` within 2 s of the Pi or network
dying, while the robot stops on its own via the watchdog.

### Telemetry

The Pi subscribes to the chassis velocity and IMU DDS feeds and piggybacks the
latest sample on every command ack, so these cost no extra traffic and refresh
at whichever is slower, `--rate` or `--telemetry-freq`.

| Property | Type | Meaning |
|---|---|---|
| `velocity_body` | `(3,)` float array or `None` | Chassis velocity in the **body** frame, m/s |
| `velocity_world` | `(3,)` float array or `None` | Chassis velocity in the **power-on world** frame, m/s |
| `accel` | `(3,)` float array or `None` | IMU acceleration, firmware units (see below) |
| `gyro` | `(3,)` float array or `None` | IMU angular rate, firmware units (rad/s as documented) |
| `velocity_age` | float or `None` | Seconds since that velocity sample was taken |
| `imu_age` | float or `None` | Seconds since that IMU sample was taken |
| `telemetry` | dict | All of the above plus `battery` and `rtt`, in one call |

```python
v = link.velocity_body               # np.array([vx, vy, vz]) in m/s, or None
if v is not None and link.velocity_age < 0.2:
    ...                              # fresh enough to use
```

**Every one of these is `None` until the first sample arrives**, and they hold
their last value if the subscription dies — which is what `velocity_age` and
`imu_age` are for. Ages are computed on the Pi at ack time and topped up
locally with monotonic time, so no cross-machine clock comparison is involved
and they are safe to threshold.

**Frames.** `velocity_body` is the chassis body frame (x forward, y right,
matching `drive()`); `velocity_world` is relative to the pose at robot
power-on, not to any map you maintain. Converting either to a camera frame is
the caller's job — the link does no rotation.

**IMU units are the firmware's.** The SDK rounds these values but does not
rescale them, and DJI documents acceleration in g with angular rate in rad/s.
Verify once rather than trusting it: sit the robot still and read `accel`. A
magnitude near `1.0` means g (multiply by 9.81 for m/s²); near `9.81` means it
is already m/s².

**Rate.** `--telemetry-freq` accepts only the DDS-permitted values 1, 5, 10,
20, 50 Hz. Samples only reach you on an ack, so the effective rate is bounded
by the client's `rate_hz` too; leave both at 20 for ~20 Hz telemetry.

### Command line

```bash
python robot_client.py --host <pi-ip> --view    # live window, rtt/battery/counters
python robot_client.py --host <pi-ip> --demo    # forward/back/rotate; needs floor space
python robot_client.py --host <pi-ip>           # print status and exit
```

---

## 5. Integration patterns

### Replacing direct SDK calls in an existing handler

Code that drives the robot in-process typically looks like:

```python
self.ep.chassis.drive_speed(x=vx, y=vy, z=0.0, timeout=self.command_timeout)
frame = self.camera.read_cv2_image(...)
```

The network equivalent, same units, same sign conventions:

```python
self.link.drive(x=vx, y=vy, z=0.0)      # no timeout= : see below
frame = self.link.read_frame(timeout=1.0)
```

Mapping notes:

- **`timeout=` has no equivalent and needs none.** In the SDK it starts a local
  `threading.Timer` that stops the robot later — which dies with the process it
  was meant to protect. The server's `--watchdog` replaces it and runs on the
  Pi, so it still fires if the workstation crashes.
- **`connect()` / `close()`** become constructing and closing the link. The
  robot session itself lives in the server process, so the client can restart
  freely without re-initializing the robot.
- **Chassis velocity and IMU are proxied** on every ack: `link.velocity_body`,
  `link.velocity_world`, `link.accel`, `link.gyro`, with `velocity_age` /
  `imu_age` for staleness. A handler getter maps straight across:

  ```python
  def get_velocity_reading(self):
      v = self.link.velocity_body          # (3,) m/s in the chassis body frame
      if v is None or self.link.velocity_age > MAX_AGE:
          return None
      return VelocityReading(vel=body_to_camera_frame(v))

  def get_accel_reading(self):
      a = self.link.accel                  # (3,), firmware units
      if a is None or self.link.imu_age > MAX_AGE:
          return None
      return body_to_camera_frame(a)
  ```

  The rotation into the camera frame stays with the caller, as it is in the
  simulator path.
- **Other DDS subjects** (`sub_position`, `sub_attitude`, `sub_esc`, …) are not
  proxied — see *Extending*.
- **Blocking action calls** (`chassis.move(...).wait_for_completed()`) are not
  proxied either; the link is velocity-only by design.

### Shape of a control loop

```python
import math
from robot_client import RoboMasterLink

with RoboMasterLink(PI_IP) as link:
    prev_t = None
    while True:
        frame, meta = link.read_frame(timeout=1.0, with_meta=True)
        if frame is None:
            link.stop()                     # no vision: don't keep driving blind
            continue
        dt = None if prev_t is None else meta["captured_at"] - prev_t
        prev_t = meta["captured_at"]

        vx, wz_rad = controller(frame, dt)
        if not link.connected:
            break                            # link died; watchdog stops the robot
        link.drive(x=vx, z=math.degrees(wz_rad))
```

Exiting the `with` block stops the robot. If the process is killed outright,
the server's watchdog stops it within `--watchdog` seconds.

### Image quality for gradient-based vision

Frames are JPEG-compressed. For optical flow, normal flow, feature tracking, or
anything differentiating the image, the default `--quality 80` puts 8×8 block
artifacts directly into spatial gradients.

- Use `--quality 92` or higher for gradient-based work; compare results against
  locally captured frames before trusting the numbers.
- **Keep the resolution fixed and equal to your calibration resolution.**
  Camera intrinsics scale with image size; `--width/--height` must match what
  the calibration was computed at.
- The webcam is on the Pi, so calibrate *that* camera at the streamed
  resolution — laptop-webcam intrinsics do not transfer.
- Bandwidth at 640×480/q80 is ~30–50 KB per frame, ~6–8 Mbit/s at 20 fps.
  Quality 95 roughly doubles it.

### Latency budget

| Stage | Typical |
|---|---|
| Capture + JPEG encode on the Pi | ~35 ms |
| Network (LAN Wi-Fi) | 5–20 ms |
| Decode in `read_frame()` | ~5 ms |
| Command out + robot ack | ~20 ms |

Roughly 50–80 ms glass-to-controller, plus ~20 ms back to the wheels.

---

## 6. Wire protocol

Useful for debugging with `nc`/Wireshark, or for reimplementing either end.

**Commands — UDP, one JSON object per datagram, client → server:**

```json
{"seq": 12, "x": 0.4, "y": 0.0, "z": 30.0}
{"seq": 13, "stop": true}
```

`seq` strictly increases; the server discards packets with `seq` lower than the
highest seen (UDP reorders). Every packet is acked to the sender's address:

```json
{"seq": 12, "t": 1727712345.67, "battery": 87,
 "vel": {"world": [0.12, -0.03, 0.0], "body": [0.11, 0.0, 0.0], "age": 0.031},
 "imu": {"accel": [0.01, 0.0, 1.002], "gyro": [0.0, 0.0, 0.52], "age": 0.028}}
```

`vel` and `imu` are absent until the first DDS sample arrives, and absent
entirely under `--no-telemetry`. `age` is seconds since the sample was taken,
measured on the Pi.

`"stale": true` marks an ack for an out-of-order packet.

**Video — TCP, server → client, repeating:**

```
16-byte header:  !IdI  = (payload_length, captured_at, frame_id)
payload:         JPEG bytes, exactly payload_length
```

Big-endian, no padding. The server accepts one client at a time and sends only
frames newer than the last one sent; the client reconnects automatically if the
stream drops.

---

## 7. Troubleshooting

| Symptom | Cause |
|---|---|
| `connected` stays `False`, robot moves anyway | Acks blocked inbound — Windows Firewall, or router client isolation |
| `connected` `False` and robot doesn't move | Server not running, wrong `--host`, or wrong `--cmd-port` |
| `read_frame()` always `None` | Server started with `--no-video`, camera index wrong, or tcp/9813 blocked |
| Robot stutters between moving and stopping | Client `rate_hz` below `1 / watchdog`, or heavy packet loss |
| Frames lag further and further behind | Consuming slower than the link delivers — check `frames_skipped`, lower `--fps`/`--quality` |
| Server exits at `initialize()` | Wrong `--conn` or `--robot-ip`; run `pi/smoke_test.py` first |
| Battery push shows `NO DATA` in the smoke test | Robot can't reach the Pi — router client isolation |
| `UnboundLocalError: proxy_addr` | Pre-fix SDK copy installed; reinstall the package |

---

## 8. Limitations

- **Chassis only.** No gimbal, blaster, arm, or LED over the link. Telemetry is
  limited to battery, chassis velocity and IMU; position and attitude are not
  proxied. The server holds a full `robot.Robot` object, so adding more is
  mechanical — see *Extending*.
- **One video client at a time.** A second connection waits until the first
  disconnects.
- **No authentication or encryption.** Anything on the network that can reach
  udp/9812 can drive the robot. Keep it on a trusted LAN.
- **The robot's own camera is unused.** This link streams a USB webcam on the
  Pi. On this particular robot the gimbal-mounted camera module (`0x01`) does
  not respond, so `camera`/`gimbal`/`led` SDK calls time out; chassis, battery,
  and vision modules work.
- **Re-pairing the robot to a new Wi-Fi network needs the robot's camera** to
  scan a QR code — credentials are camera-only in both the binary and plaintext
  SDKs. Workaround: give the new network the SSID and password the robot
  already knows.

### Extending

To add another DDS subject, follow what `Telemetry` already does in
`robot_server.py`: give it an `on_<subject>` callback, subscribe next to the
velocity and IMU subscriptions in `main()`, add a block to `snapshot()`, and
unsubscribe in the `finally`. `snapshot()` is merged into every ack, so the
client sees it in `_last_ack` — add a property using the `_vector()` / `_age()`
helpers. Keep per-packet additions small; anything bulky deserves its own TCP
stream rather than inflating a 20 Hz ack.
