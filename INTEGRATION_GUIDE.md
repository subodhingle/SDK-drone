# SDK-drone Integration Guide

How to integrate this SDK with any drone — from a basic RC quad to a custom
autonomous platform.

---

## Overview

The SDK abstracts all drone hardware behind 3 concepts:

```
Your Code
    │
    ▼
DroneSDK  ←── single interface
    │
    ├── MAVLink backend   (ArduCopter, PX4, Pixhawk, Cube)
    ├── AirSim backend    (Microsoft simulator)
    ├── DJI backend       (DJI via UDP bridge)
    └── Sim backend       (pure software, no hardware)
```

You write your logic once against `DroneSDK`. Switching drone hardware
means changing one line — the backend string.

---

## Part 1 — Integrating a MAVLink Drone (ArduCopter / PX4)

### Compatible hardware
- Any drone running ArduCopter or PX4 firmware
- Flight controllers: Pixhawk 1/2/4/6, Cube Orange, Matek, Holybro, etc.
- Companion computers: Raspberry Pi, Jetson Nano, any Linux SBC

### Wiring

```
Flight Controller (FC)
    │
    ├── USB → PC/laptop          (direct serial)
    ├── TELEM1 → USB-UART → PC   (serial via adapter)
    └── TELEM2 → companion PC    (onboard integration)
```

### Connection options

```python
# Option A: USB direct
sdk = DroneSDK("mavlink", connection="COM3")          # Windows
sdk = DroneSDK("mavlink", connection="/dev/ttyUSB0")  # Linux

# Option B: Via Mission Planner (MP forwards telemetry)
# In MP: Config → Planner → Output to IP → 127.0.0.1:14551
sdk = DroneSDK("mavlink", connection="udp:127.0.0.1:14551")

# Option C: Direct UDP (no Mission Planner)
sdk = DroneSDK("mavlink", connection="udp:127.0.0.1:14550")

# Option D: Companion computer on drone (onboard)
sdk = DroneSDK("mavlink", connection="/dev/ttyAMA0", baud=921600)
```

### Minimal flight script

```python
from sdk.drone_sdk import DroneSDK
import time

sdk = DroneSDK("mavlink", connection="udp:127.0.0.1:14551")

if not sdk.connect():
    print("Connection failed — check cable/port")
    exit()

# Check state before arming
state = sdk.get_state()
print(f"Battery: {state.battery_pct}%  GPS: {state.gps_fix}  Mode: {state.mode}")

if state.battery_pct < 20:
    print("Low battery — charge before flying")
    exit()

# Arm and takeoff
sdk.arm()
sdk.takeoff(altitude=5.0)
time.sleep(5)   # wait to reach altitude

# Fly forward 3 seconds
sdk.send_velocity(vx=1.0, vy=0.0, vz=0.0)
time.sleep(3)

# Stop and land
sdk.hover()
time.sleep(1)
sdk.land()
sdk.disconnect()
```

### ArduCopter GUIDED mode requirement

The SDK sends velocity commands using `MAV_FRAME_BODY_OFFSET_NED`.
The drone **must be in GUIDED mode** to accept these.

```python
sdk.set_mode("GUIDED")   # switch to GUIDED before sending velocity
sdk.send_velocity(vx=1.0, vy=0.0, vz=0.0)
```

The SDK does not auto-switch modes — you control when to switch.

---

## Part 2 — Integrating with AirSim (Simulation)

### Setup

1. Download AirSim Blocks from:
   https://github.com/microsoft/AirSim/releases

2. Create `~/Documents/AirSim/settings.json`:
```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "CameraDefaults": {
    "CaptureSettings": [{"ImageType": 0, "Width": 640, "Height": 480}]
  }
}
```

3. Run Blocks, then connect:
```python
sdk = DroneSDK("airsim")
sdk.connect()
sdk.takeoff(altitude=3.0)
sdk.send_velocity(vx=2.0, vy=0.0, vz=0.0)
```

### Getting camera feed from AirSim

```python
import airsim, cv2, numpy as np

client = airsim.MultirotorClient()
client.confirmConnection()

while True:
    response = client.simGetImage("0", airsim.ImageType.Scene)
    img = cv2.imdecode(np.frombuffer(response, np.uint8), cv2.IMREAD_COLOR)
    cv2.imshow("AirSim", img)
    if cv2.waitKey(1) == ord('q'):
        break
```

---

## Part 3 — Integrating a DJI Drone

### Architecture

DJI drones require a mobile device running the DJI Mobile SDK.
The SDK-drone communicates with a bridge app on the phone over UDP.

```
PC (SDK-drone)
    │  UDP JSON commands
    ▼
Phone (bridge app)
    │  DJI Mobile SDK
    ▼
DJI Drone
```

### Bridge app protocol

The bridge app must implement this UDP JSON protocol:

**Receive from SDK (commands):**
```json
{"cmd": "takeoff", "alt": 3.0}
{"cmd": "land"}
{"cmd": "velocity", "vx": 1.0, "vy": 0.0, "vz": 0.0, "yaw": 0.1}
{"cmd": "goto", "lat": 18.5204, "lon": 73.8567, "alt": 10.0}
{"cmd": "rtl"}
{"cmd": "arm"}
{"cmd": "disarm"}
```

**Send to SDK (telemetry, 10Hz):**
```json
{"alt": 3.2, "roll": 1.1, "pitch": -0.5, "yaw": 45.0,
 "bat": 85, "armed": true, "vx": 0.5, "vy": 0.0}
```

### Connect

```python
sdk = DroneSDK("dji", connection="192.168.1.100:8889")
sdk.connect()
sdk.takeoff(altitude=3.0)
```

---

## Part 4 — Building a Custom Backend

If your drone uses a proprietary protocol (ROS, custom serial, HTTP API),
you can add a new backend in 3 steps.

### Step 1 — Subclass `_BackendBase`

```python
# In sdk/drone_sdk.py, add your class:

class _MyDroneBackend(_BackendBase):
    def __init__(self, host: str, port: int):
        self._host  = host
        self._port  = port
        self._state = DroneState()

    def connect(self) -> bool:
        # Open your connection here
        # e.g. serial, socket, HTTP session
        self._state.connected = True
        return True

    def disconnect(self):
        pass   # close connection

    def arm(self) -> bool:
        # Send arm command to your drone
        return True

    def disarm(self) -> bool:
        return True

    def takeoff(self, altitude: float) -> bool:
        # Send takeoff command
        return True

    def land(self) -> bool:
        return True

    def return_to_launch(self) -> bool:
        return True

    def set_mode(self, mode: str) -> bool:
        return True

    def send_velocity(self, vx, vy, vz, yaw_rate=0.0) -> bool:
        # Translate to your drone's velocity command format
        # e.g. serial packet, HTTP POST, ROS topic publish
        return True

    def send_position(self, lat, lon, alt) -> bool:
        return True

    def get_state(self) -> DroneState:
        return DroneState(**self._state.__dict__)
```

### Step 2 — Register in DroneSDK.__init__

```python
# In DroneSDK.__init__, add your backend:
elif self.backend_name == "mydrone":
    host, _, port = connection.partition(":")
    self._backend = _MyDroneBackend(host, int(port))
```

### Step 3 — Use it

```python
sdk = DroneSDK("mydrone", connection="192.168.1.50:9000")
sdk.connect()
sdk.takeoff(3.0)
sdk.send_velocity(vx=1.0, vy=0.0, vz=0.0)
```

---

## Part 5 — Integrating with the AI Tracking System

The SDK is already wired into the drone AI system (`main.py`).
At startup you choose your backend:

```
+------------------------------------------+
|  DRONE BACKEND                           |
|  1. MAVLink  (ArduCopter/Mission Planner)|
|  2. AirSim   (simulator)                 |
|  3. DJI      (UDP bridge)                |
|  4. Sim      (software only)             |
|  5. Skip     (vision only)               |
+------------------------------------------+
```

The AI system then:
1. Detects objects with YOLOv8 on GPU
2. Tracks them with ByteTrack + ReID memory
3. Computes PID velocity commands
4. Sends them via `sdk.send_velocity(vx, vy, vz, yaw_rate)`
5. Reads telemetry via `sdk.get_telemetry()` for the HUD

### Using the SDK standalone (without the full AI system)

```python
from sdk.drone_sdk import DroneSDK

class MyTracker:
    """Your own tracking logic."""
    def get_error(self, frame):
        # Returns (error_x, error_y) normalized -1 to 1
        return 0.1, -0.05

sdk     = DroneSDK("mavlink", connection="udp:127.0.0.1:14551")
tracker = MyTracker()

sdk.connect()
sdk.arm()
sdk.takeoff(3.0)

import cv2
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    err_x, err_y = tracker.get_error(frame)

    # Simple P controller
    yaw_rate = err_x * 0.5
    vert     = -err_y * 0.4

    sdk.send_velocity(vx=0.3, vy=0.0, vz=vert, yaw_rate=yaw_rate)

    if cv2.waitKey(1) == ord('q'):
        break

sdk.land()
sdk.disconnect()
```

---

## Part 6 — Telemetry Reference

`sdk.get_telemetry()` returns a dict compatible with the HUD display:

```python
{
    "lat":         float,   # GPS latitude
    "lon":         float,   # GPS longitude
    "alt_msl":     float,   # altitude above sea level (m)
    "alt_rel":     float,   # altitude above launch point (m)
    "heading":     int,     # compass heading 0-360
    "groundspeed": float,   # horizontal speed (m/s)
    "battery_v":   float,   # battery voltage
    "battery_pct": int,     # battery percentage (0-100, -1=unknown)
    "mode":        str,     # flight mode string e.g. "GUIDED"
    "armed":       bool,    # True if motors armed
    "gps_fix":     int,     # 0=no fix, 2=2D, 3=3D
    "satellites":  int,     # number of GPS satellites
    "roll":        float,   # degrees
    "pitch":       float,   # degrees
    "yaw":         float,   # degrees
    "vx":          float,   # forward velocity (m/s)
    "vy":          float,   # lateral velocity (m/s)
    "vz":          float,   # vertical velocity (m/s)
    "lidar_dist":  float,   # rangefinder distance (m), -1=no sensor
}
```

---

## Part 7 — Safety Checklist

Before flying with any backend:

- [ ] Battery above 50% for first test
- [ ] GPS fix type 3 (3D fix) before arming outdoors
- [ ] Propellers clear of people and obstacles
- [ ] Drone in GUIDED mode before sending velocity commands
- [ ] Test `send_velocity` with small values first (vx=0.2)
- [ ] Have a kill switch ready (R key = unlock/stop, L key = land)
- [ ] Test full pipeline in simulation (`DroneSDK("sim")`) before real flight

---

## Summary

| Task | Code |
|------|------|
| Connect MAVLink | `DroneSDK("mavlink", connection="udp:127.0.0.1:14551")` |
| Connect AirSim | `DroneSDK("airsim")` |
| Connect DJI | `DroneSDK("dji", connection="192.168.1.100:8889")` |
| Test without hardware | `DroneSDK("sim")` |
| Arm + takeoff | `sdk.arm(); sdk.takeoff(3.0)` |
| Send velocity | `sdk.send_velocity(vx=1.0, vy=0.0, vz=0.0, yaw_rate=0.1)` |
| Stop movement | `sdk.hover()` |
| Land | `sdk.land()` |
| Get telemetry | `sdk.get_telemetry()` |
| Add custom drone | Subclass `_BackendBase`, register in `DroneSDK.__init__` |
