# SDK-drone Installation Guide

## Requirements

- Python 3.10 or higher
- pip (comes with Python)
- Git

---

## Step 1 — Clone the repository

```bash
git clone https://github.com/subodhingle/SDK-drone.git
cd SDK-drone
```

---

## Step 2 — Create a virtual environment

**Windows:**
```bash
python -m venv venv
.\venv\Scripts\activate
```

**Linux / macOS:**
```bash
python3 -m venv venv
source venv/bin/activate
```

---

## Step 3 — Install base dependencies

```bash
pip install -r requirements.txt
```

---

## Step 4 — Install backend-specific dependencies

Install only what you need for your drone:

### MAVLink (ArduCopter / PX4 / Mission Planner)
```bash
pip install pymavlink
```

### AirSim (Microsoft simulator)
```bash
pip install airsim
```

### DJI (UDP bridge)
No extra packages needed.
Install the companion bridge app on your Android/iOS device.
The app translates UDP JSON commands to DJI Mobile SDK calls.

### Simulation (no hardware)
No extra packages needed. Works out of the box.

---

## Step 5 — Verify installation

```bash
python example_usage.py
```

Expected output:
```
=== Simulation Backend ===
[DroneSDK] Backend: sim
[SDK/Sim] Simulation backend started
[SDK/Sim] Armed
[SDK/Sim] Takeoff → 3.0m
Moving forward...
Rotating...
State: alt=3.0m  roll=...  pitch=...
[SDK/Sim] Landed
Done.
```

---

## Backend Connection Guide

### MAVLink — Mission Planner (recommended)

1. Connect your flight controller to Mission Planner
2. In Mission Planner: **Config → Planner → Output this data to IP**
   - Add entry: `127.0.0.1` port `14551`
3. Use connection string: `udp:127.0.0.1:14551`

```python
sdk = DroneSDK("mavlink", connection="udp:127.0.0.1:14551")
```

### MAVLink — Direct serial (no Mission Planner)

Check Device Manager for your COM port, then:
```python
sdk = DroneSDK("mavlink", connection="COM3", baud=57600)
```

### MAVLink — SITL (software simulation)

```bash
# Start ArduCopter SITL first
sim_vehicle.py -v ArduCopter --console --map
```
```python
sdk = DroneSDK("mavlink", connection="udp:127.0.0.1:14550")
```

### AirSim

1. Download AirSim Blocks from https://github.com/microsoft/AirSim/releases
2. Run the Blocks environment
3. Connect:
```python
sdk = DroneSDK("airsim")
```

### DJI UDP Bridge

1. Install the bridge app on your Android/iOS device
2. Connect your phone to the same WiFi network as your PC
3. Find your phone's IP address
4. Connect:
```python
sdk = DroneSDK("dji", connection="192.168.1.100:8889")
```

### Simulation (no hardware)

```python
sdk = DroneSDK("sim")
```

---

## Quick Integration Example

```python
from sdk.drone_sdk import DroneSDK
import time

# Choose your backend
sdk = DroneSDK("sim")   # change to "mavlink", "airsim", or "dji"
sdk.connect()

# Basic flight sequence
sdk.arm()
sdk.takeoff(altitude=3.0)
time.sleep(2)

# Send velocity commands (body frame)
# vx=forward, vy=right, vz=down, yaw_rate=rad/s
sdk.send_velocity(vx=1.0, vy=0.0, vz=0.0, yaw_rate=0.0)
time.sleep(3)

# Get telemetry
state = sdk.get_state()
print(f"Altitude: {state.alt_rel:.1f}m")
print(f"Battery:  {state.battery_pct}%")
print(f"Mode:     {state.mode}")

# Land safely
sdk.hover()
sdk.land()
sdk.disconnect()
```

---

## Troubleshooting

**MAVLink: No heartbeat received**
- Check Mission Planner is running and UDP output is configured
- Check firewall is not blocking port 14551
- Try direct serial connection instead

**AirSim: Connection refused**
- Make sure AirSim Blocks environment is running before connecting
- Check `settings.json` has `"SimMode": "Multirotor"`

**DJI: No response**
- Verify phone and PC are on the same WiFi network
- Check bridge app is running and showing "Connected"
- Try pinging the phone IP from PC: `ping 192.168.1.100`

**ImportError: No module named 'pymavlink'**
```bash
pip install pymavlink
```

**ImportError: No module named 'airsim'**
```bash
pip install airsim
```

---

## File Structure

```
SDK-drone/
├── sdk/
│   ├── __init__.py       # Package init
│   └── drone_sdk.py      # Universal SDK (all backends)
├── example_usage.py      # Runnable examples
├── requirements.txt      # Base dependencies
├── README.md             # API reference
└── INSTALLATION.md       # This file
```
