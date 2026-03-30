# SDK-drone

Universal Drone SDK for any drone integration.

> Full installation guide: [INSTALLATION.md](INSTALLATION.md)

## Supported Backends

| Backend | Hardware | Connection |
|---------|----------|------------|
| `mavlink` | ArduCopter / PX4 / Mission Planner | UDP / TCP / Serial |
| `airsim` | Microsoft AirSim simulator | localhost |
| `dji` | DJI drones via UDP bridge app | WiFi UDP |
| `sim` | Software simulation (no hardware) | none |

## Quick Start

```python
from sdk.drone_sdk import DroneSDK

# Simulation (no hardware needed)
sdk = DroneSDK("sim")
sdk.connect()
sdk.arm()
sdk.takeoff(altitude=3.0)
sdk.send_velocity(vx=0.5, vy=0.0, vz=0.0, yaw_rate=0.0)
sdk.land()
sdk.disconnect()

# MAVLink (ArduCopter / Mission Planner)
sdk = DroneSDK("mavlink", connection="udp:127.0.0.1:14551")

# AirSim
sdk = DroneSDK("airsim")

# DJI via UDP bridge
sdk = DroneSDK("dji", connection="192.168.1.100:8889")
```

## API Reference

```python
sdk.connect()                          # Connect to drone
sdk.disconnect()                       # Disconnect
sdk.arm()                              # Arm motors
sdk.disarm()                           # Disarm motors
sdk.takeoff(altitude=3.0)              # Takeoff to altitude (metres)
sdk.land()                             # Land
sdk.return_to_launch()                 # RTL
sdk.set_mode("GUIDED")                 # Set flight mode
sdk.send_velocity(vx, vy, vz, yaw_rate)  # Body-frame velocity (m/s, rad/s)
sdk.send_position(lat, lon, alt)       # Fly to GPS coordinate
sdk.hover()                            # Stop all movement
sdk.get_state()                        # Returns DroneState dataclass
sdk.get_telemetry()                    # Returns HUD-compatible dict
```

## DroneState Fields

```
lat, lon, alt_msl, alt_rel    — position
roll, pitch, yaw, heading     — attitude (degrees)
vx, vy, vz, groundspeed       — velocity (m/s)
battery_v, battery_pct        — power
gps_fix, satellites           — GPS
armed, mode, connected        — status
lidar_dist                    — rangefinder (metres, -1 = no sensor)
```

## Integration with Drone AI System

The SDK is fully integrated with the AI tracking system.
At startup, choose your backend:

```
1. MAVLink  (ArduCopter / Mission Planner)
2. AirSim   (simulator)
3. DJI      (UDP bridge)
4. Sim      (software only)
5. Skip     (vision only)
```

## Install

```bash
pip install pymavlink          # for mavlink backend
pip install airsim             # for airsim backend
# DJI and Sim require no extra packages
```
