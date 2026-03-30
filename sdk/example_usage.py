"""
DroneSDK — Usage Examples
Run from project root: .\venv\Scripts\python.exe drone_ai_system\sdk\example_usage.py
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sdk.drone_sdk import DroneSDK
import time

def example_simulation():
    print("\n=== Simulation Backend ===")
    sdk = DroneSDK("sim")
    sdk.connect()
    sdk.arm()
    sdk.takeoff(altitude=3.0)
    time.sleep(1)

    print("Moving forward...")
    sdk.send_velocity(vx=0.5, vy=0.0, vz=0.0, yaw_rate=0.0)
    time.sleep(2)

    print("Rotating...")
    sdk.send_velocity(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.3)
    time.sleep(1)

    sdk.hover()
    state = sdk.get_state()
    print(f"State: alt={state.alt_rel:.1f}m  roll={state.roll}  pitch={state.pitch}")
    print(f"Telemetry: {sdk.get_telemetry()}")

    sdk.land()
    sdk.disconnect()
    print("Done.")

def example_mavlink():
    print("\n=== MAVLink Backend (Mission Planner) ===")
    sdk = DroneSDK("mavlink", connection="udp:127.0.0.1:14551")
    if not sdk.connect():
        print("MAVLink not available — run Mission Planner first")
        return
    sdk.arm()
    sdk.takeoff(altitude=5.0)
    time.sleep(3)
    sdk.send_velocity(vx=1.0, vy=0.0, vz=0.0)
    time.sleep(2)
    sdk.land()
    sdk.disconnect()

def example_airsim():
    print("\n=== AirSim Backend ===")
    sdk = DroneSDK("airsim")
    if not sdk.connect():
        print("AirSim not running — start Blocks environment first")
        return
    sdk.takeoff(altitude=3.0)
    time.sleep(2)
    sdk.send_velocity(vx=2.0, vy=0.0, vz=0.0)
    time.sleep(3)
    sdk.land()
    sdk.disconnect()

def example_dji():
    print("\n=== DJI Backend (UDP bridge) ===")
    # Requires DJI bridge app running on mobile device
    sdk = DroneSDK("dji", connection="192.168.1.100:8889")
    sdk.connect()
    sdk.takeoff(altitude=3.0)
    time.sleep(2)
    sdk.send_velocity(vx=0.5, vy=0.0, vz=0.0)
    time.sleep(2)
    sdk.land()
    sdk.disconnect()

if __name__ == "__main__":
    example_simulation()
    # Uncomment to test other backends:
    # example_mavlink()
    # example_airsim()
    # example_dji()
