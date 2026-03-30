"""
Universal Drone SDK
====================
Single interface for any drone hardware or simulator.

Supported backends:
  - mavlink   : ArduCopter / PX4 via pymavlink (real drone or SITL)
  - airsim    : Microsoft AirSim simulator
  - dji       : DJI drones via DJI Mobile SDK bridge (UDP)
  - parrot    : Parrot drones via Olympe SDK
  - sim       : Built-in software simulation (no hardware needed)

Usage:
    from drone_ai_system.sdk import DroneSDK

    # MAVLink (ArduCopter / Mission Planner)
    sdk = DroneSDK(backend="mavlink", connection="udp:127.0.0.1:14551")

    # AirSim
    sdk = DroneSDK(backend="airsim")

    # DJI (via UDP bridge)
    sdk = DroneSDK(backend="dji", connection="udp:192.168.1.100:8889")

    # Pure simulation (no hardware)
    sdk = DroneSDK(backend="sim")

    sdk.connect()
    sdk.arm()
    sdk.takeoff(altitude=3.0)

    # Send velocity commands (body frame)
    sdk.send_velocity(vx=0.5, vy=0.0, vz=0.0, yaw_rate=0.1)

    telemetry = sdk.get_telemetry()
    sdk.land()
    sdk.disconnect()
"""
import threading
import time
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Dict, Tuple
from enum import Enum


# ── Drone state ───────────────────────────────────────────────────────────────
@dataclass
class DroneState:
    # Position
    lat:         float = 0.0
    lon:         float = 0.0
    alt_msl:     float = 0.0
    alt_rel:     float = 0.0   # AGL / relative altitude

    # Attitude
    roll:        float = 0.0   # degrees
    pitch:       float = 0.0
    yaw:         float = 0.0
    heading:     int   = 0     # 0-360

    # Velocity (body frame, m/s)
    vx:          float = 0.0
    vy:          float = 0.0
    vz:          float = 0.0
    groundspeed: float = 0.0

    # Power
    battery_v:   float = 0.0
    battery_pct: int   = -1

    # GPS
    gps_fix:     int   = 0
    satellites:  int   = 0

    # Status
    armed:       bool  = False
    mode:        str   = "UNKNOWN"
    connected:   bool  = False

    # Sensors
    lidar_dist:  float = -1.0

    timestamp:   float = field(default_factory=time.time)

    def to_dict(self) -> Dict:
        return self.__dict__.copy()


class DroneBackend(Enum):
    MAVLINK = "mavlink"
    AIRSIM  = "airsim"
    DJI     = "dji"
    PARROT  = "parrot"
    SIM     = "sim"


# ── Abstract backend interface ────────────────────────────────────────────────
class _BackendBase(ABC):
    @abstractmethod
    def connect(self) -> bool: ...
    @abstractmethod
    def disconnect(self): ...
    @abstractmethod
    def arm(self) -> bool: ...
    @abstractmethod
    def disarm(self) -> bool: ...
    @abstractmethod
    def takeoff(self, altitude: float) -> bool: ...
    @abstractmethod
    def land(self) -> bool: ...
    @abstractmethod
    def return_to_launch(self) -> bool: ...
    @abstractmethod
    def send_velocity(self, vx: float, vy: float, vz: float,
                      yaw_rate: float = 0.0) -> bool: ...
    @abstractmethod
    def send_position(self, lat: float, lon: float, alt: float) -> bool: ...
    @abstractmethod
    def set_mode(self, mode: str) -> bool: ...
    @abstractmethod
    def get_state(self) -> DroneState: ...


# ── MAVLink backend ───────────────────────────────────────────────────────────
class _MAVLinkBackend(_BackendBase):
    def __init__(self, connection: str, baud: int = 57600):
        self._conn_str = connection
        self._baud     = baud
        self._master   = None
        self._state    = DroneState()
        self._lock     = threading.Lock()
        self._running  = False

    def connect(self) -> bool:
        try:
            from pymavlink import mavutil
            self._mav = mavutil
            print(f"[SDK/MAVLink] Connecting → {self._conn_str}")
            self._master = mavutil.mavlink_connection(
                self._conn_str,
                baud=self._baud if "COM" in self._conn_str else None
            )
            msg = self._master.wait_heartbeat(timeout=15)
            if msg is None:
                return False
            self._state.connected = True
            self._request_streams()
            self._running = True
            threading.Thread(target=self._telem_loop, daemon=True).start()
            print(f"[SDK/MAVLink] Connected — sysid={self._master.target_system}")
            return True
        except Exception as e:
            print(f"[SDK/MAVLink] Error: {e}")
            return False

    def _request_streams(self):
        from pymavlink import mavutil
        for sid, rate in [
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION,    10),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,      10),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,       4),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 4),
            (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,  4),
        ]:
            self._master.mav.request_data_stream_send(
                self._master.target_system,
                self._master.target_component,
                sid, rate, 1)

    def _telem_loop(self):
        from pymavlink import mavutil
        while self._running:
            try:
                msg = self._master.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue
                t = msg.get_type()
                with self._lock:
                    if t == "GLOBAL_POSITION_INT":
                        self._state.lat         = msg.lat / 1e7
                        self._state.lon         = msg.lon / 1e7
                        self._state.alt_msl     = msg.alt / 1000.0
                        self._state.alt_rel     = msg.relative_alt / 1000.0
                        self._state.heading     = msg.hdg // 100
                        self._state.vx          = msg.vx / 100.0
                        self._state.vy          = msg.vy / 100.0
                        self._state.vz          = msg.vz / 100.0
                        self._state.groundspeed = round(math.hypot(msg.vx, msg.vy) / 100.0, 2)
                    elif t == "ATTITUDE":
                        self._state.roll  = round(msg.roll  * 57.3, 1)
                        self._state.pitch = round(msg.pitch * 57.3, 1)
                        self._state.yaw   = round(msg.yaw   * 57.3, 1)
                    elif t == "SYS_STATUS":
                        self._state.battery_v   = round(msg.voltage_battery / 1000.0, 2)
                        self._state.battery_pct = msg.battery_remaining
                    elif t == "GPS_RAW_INT":
                        self._state.gps_fix    = msg.fix_type
                        self._state.satellites = msg.satellites_visible
                    elif t == "HEARTBEAT":
                        from pymavlink import mavutil as mu
                        self._state.mode  = mu.mode_string_v10(msg)
                        self._state.armed = bool(
                            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    elif t == "DISTANCE_SENSOR":
                        self._state.lidar_dist = round(msg.current_distance / 100.0, 3)
                    self._state.timestamp = time.time()
            except Exception:
                pass

    def disconnect(self):
        self._running = False
        if self._master:
            self._master.close()
        self._state.connected = False

    def arm(self) -> bool:
        self._master.arducopter_arm()
        self._master.motors_armed_wait()
        return True

    def disarm(self) -> bool:
        self._master.arducopter_disarm()
        return True

    def takeoff(self, altitude: float) -> bool:
        from pymavlink import mavutil
        self.set_mode("GUIDED")
        time.sleep(0.5)
        self.arm()
        time.sleep(1.0)
        self._master.mav.command_long_send(
            self._master.target_system, self._master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude)
        return True

    def land(self) -> bool:
        from pymavlink import mavutil
        self._master.mav.command_long_send(
            self._master.target_system, self._master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0)
        return True

    def return_to_launch(self) -> bool:
        return self.set_mode("RTL")

    def set_mode(self, mode: str) -> bool:
        from pymavlink import mavutil
        mode_id = self._master.mode_mapping().get(mode)
        if mode_id is None:
            return False
        self._master.mav.set_mode_send(
            self._master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        return True

    def send_velocity(self, vx: float, vy: float, vz: float,
                      yaw_rate: float = 0.0) -> bool:
        from pymavlink import mavutil
        self._master.mav.set_position_target_local_ned_send(
            0,
            self._master.target_system, self._master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yaw_rate)
        return True

    def send_position(self, lat: float, lon: float, alt: float) -> bool:
        from pymavlink import mavutil
        self._master.mav.set_position_target_global_int_send(
            0,
            self._master.target_system, self._master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7), int(lon * 1e7), alt,
            0, 0, 0, 0, 0, 0, 0, 0)
        return True

    def get_state(self) -> DroneState:
        with self._lock:
            return DroneState(**self._state.__dict__)


# ── AirSim backend ────────────────────────────────────────────────────────────
class _AirSimBackend(_BackendBase):
    def __init__(self):
        self._client = None
        self._state  = DroneState()
        self._running = False

    def connect(self) -> bool:
        try:
            import airsim
            self._client = airsim.MultirotorClient()
            self._client.confirmConnection()
            self._client.enableApiControl(True)
            self._state.connected = True
            self._running = True
            threading.Thread(target=self._telem_loop, daemon=True).start()
            print("[SDK/AirSim] Connected")
            return True
        except Exception as e:
            print(f"[SDK/AirSim] Error: {e}")
            return False

    def _telem_loop(self):
        while self._running:
            try:
                import airsim
                s = self._client.getMultirotorState()
                k = s.kinematics_estimated
                pos = k.position
                vel = k.linear_velocity
                ori = k.orientation
                # Convert quaternion to euler
                import numpy as np
                q = [ori.w_val, ori.x_val, ori.y_val, ori.z_val]
                roll  = math.degrees(math.atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]**2+q[2]**2)))
                pitch = math.degrees(math.asin(max(-1, min(1, 2*(q[0]*q[2]-q[3]*q[1])))))
                yaw   = math.degrees(math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2)))
                self._state.alt_rel     = -pos.z_val
                self._state.vx          = vel.x_val
                self._state.vy          = vel.y_val
                self._state.vz          = vel.z_val
                self._state.roll        = round(roll, 1)
                self._state.pitch       = round(pitch, 1)
                self._state.yaw         = round(yaw, 1)
                self._state.groundspeed = round(math.hypot(vel.x_val, vel.y_val), 2)
                self._state.armed       = True
                self._state.mode        = "GUIDED"
                self._state.timestamp   = time.time()
            except Exception:
                pass
            time.sleep(0.05)

    def disconnect(self):
        self._running = False
        if self._client:
            self._client.enableApiControl(False)

    def arm(self) -> bool:
        self._client.armDisarm(True)
        return True

    def disarm(self) -> bool:
        self._client.armDisarm(False)
        return True

    def takeoff(self, altitude: float) -> bool:
        self._client.armDisarm(True)
        self._client.takeoffAsync().join()
        self._client.moveToZAsync(-altitude, 2).join()
        return True

    def land(self) -> bool:
        self._client.landAsync().join()
        return True

    def return_to_launch(self) -> bool:
        self._client.goHomeAsync().join()
        return True

    def set_mode(self, mode: str) -> bool:
        return True   # AirSim always in API control mode

    def send_velocity(self, vx: float, vy: float, vz: float,
                      yaw_rate: float = 0.0) -> bool:
        yaw_deg = math.degrees(yaw_rate)
        self._client.moveByVelocityAsync(vx, vy, -vz, 0.1,
            yaw_mode=__import__('airsim').YawMode(True, yaw_deg))
        return True

    def send_position(self, lat: float, lon: float, alt: float) -> bool:
        # AirSim uses local NED — approximate conversion
        self._client.moveToPositionAsync(lat, lon, -alt, 3).join()
        return True

    def get_state(self) -> DroneState:
        return DroneState(**self._state.__dict__)


# ── DJI UDP bridge backend ────────────────────────────────────────────────────
class _DJIBackend(_BackendBase):
    """
    Communicates with a DJI drone via a UDP bridge app running on the
    mobile device (Android/iOS). The bridge app translates UDP JSON
    commands to DJI Mobile SDK calls.

    Bridge protocol (JSON over UDP):
      Send: {"cmd": "velocity", "vx": 0.5, "vy": 0.0, "vz": 0.0, "yaw": 0.1}
      Send: {"cmd": "takeoff", "alt": 3.0}
      Send: {"cmd": "land"}
      Recv: {"alt": 2.5, "roll": 1.2, "pitch": 0.3, "yaw": 45.0, "bat": 85}
    """
    def __init__(self, host: str = "192.168.1.100", port: int = 8889):
        import socket
        self._host    = host
        self._port    = port
        self._sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._state   = DroneState()
        self._running = False

    def connect(self) -> bool:
        try:
            self._sock.settimeout(2.0)
            self._running = True
            threading.Thread(target=self._recv_loop, daemon=True).start()
            self._state.connected = True
            print(f"[SDK/DJI] Bridge at {self._host}:{self._port}")
            return True
        except Exception as e:
            print(f"[SDK/DJI] Error: {e}")
            return False

    def _send(self, cmd: dict):
        import json
        try:
            self._sock.sendto(json.dumps(cmd).encode(), (self._host, self._port))
        except Exception:
            pass

    def _recv_loop(self):
        import json
        while self._running:
            try:
                data, _ = self._sock.recvfrom(1024)
                s = json.loads(data.decode())
                self._state.alt_rel     = s.get("alt", 0.0)
                self._state.roll        = s.get("roll", 0.0)
                self._state.pitch       = s.get("pitch", 0.0)
                self._state.yaw         = s.get("yaw", 0.0)
                self._state.battery_pct = s.get("bat", -1)
                self._state.armed       = s.get("armed", False)
                self._state.timestamp   = time.time()
            except Exception:
                pass

    def disconnect(self):
        self._running = False
        self._sock.close()

    def arm(self) -> bool:
        self._send({"cmd": "arm"})
        return True

    def disarm(self) -> bool:
        self._send({"cmd": "disarm"})
        return True

    def takeoff(self, altitude: float) -> bool:
        self._send({"cmd": "takeoff", "alt": altitude})
        return True

    def land(self) -> bool:
        self._send({"cmd": "land"})
        return True

    def return_to_launch(self) -> bool:
        self._send({"cmd": "rtl"})
        return True

    def set_mode(self, mode: str) -> bool:
        self._send({"cmd": "mode", "mode": mode})
        return True

    def send_velocity(self, vx: float, vy: float, vz: float,
                      yaw_rate: float = 0.0) -> bool:
        self._send({"cmd": "velocity", "vx": vx, "vy": vy,
                    "vz": vz, "yaw": yaw_rate})
        return True

    def send_position(self, lat: float, lon: float, alt: float) -> bool:
        self._send({"cmd": "goto", "lat": lat, "lon": lon, "alt": alt})
        return True

    def get_state(self) -> DroneState:
        return DroneState(**self._state.__dict__)


# ── Simulation backend ────────────────────────────────────────────────────────
class _SimBackend(_BackendBase):
    """
    Pure software simulation — no hardware needed.
    Integrates velocity commands to update simulated position.
    Useful for testing the full pipeline without a drone.
    """
    def __init__(self):
        self._state   = DroneState()
        self._running = False
        self._armed   = False
        self._flying  = False

    def connect(self) -> bool:
        self._state.connected = True
        self._state.mode      = "STABILIZE"
        self._state.battery_pct = 100
        self._state.battery_v   = 12.6
        self._state.gps_fix     = 3
        self._state.satellites  = 12
        self._running = True
        threading.Thread(target=self._sim_loop, daemon=True).start()
        print("[SDK/Sim] Simulation backend started")
        return True

    def _sim_loop(self):
        dt = 0.05
        while self._running:
            if self._flying:
                # Integrate velocity
                self._state.lat += self._state.vx * dt * 1e-5
                self._state.lon += self._state.vy * dt * 1e-5
                self._state.alt_rel = max(0.0, self._state.alt_rel - self._state.vz * dt)
                # Simulate gentle oscillation
                t = time.time()
                self._state.roll  = round(2.0 * math.sin(t * 0.5), 1)
                self._state.pitch = round(1.5 * math.sin(t * 0.7), 1)
                self._state.battery_pct = max(0, self._state.battery_pct)
            self._state.timestamp = time.time()
            time.sleep(dt)

    def disconnect(self):
        self._running = False

    def arm(self) -> bool:
        self._armed = True
        self._state.armed = True
        print("[SDK/Sim] Armed")
        return True

    def disarm(self) -> bool:
        self._armed = False
        self._state.armed = False
        return True

    def takeoff(self, altitude: float) -> bool:
        self._flying = True
        self._state.alt_rel = altitude
        self._state.mode    = "GUIDED"
        print(f"[SDK/Sim] Takeoff → {altitude}m")
        return True

    def land(self) -> bool:
        self._flying = False
        self._state.alt_rel = 0.0
        self._state.vx = self._state.vy = self._state.vz = 0.0
        print("[SDK/Sim] Landed")
        return True

    def return_to_launch(self) -> bool:
        return self.land()

    def set_mode(self, mode: str) -> bool:
        self._state.mode = mode
        return True

    def send_velocity(self, vx: float, vy: float, vz: float,
                      yaw_rate: float = 0.0) -> bool:
        self._state.vx = vx
        self._state.vy = vy
        self._state.vz = vz
        self._state.groundspeed = round(math.hypot(vx, vy), 2)
        return True

    def send_position(self, lat: float, lon: float, alt: float) -> bool:
        self._state.lat     = lat
        self._state.lon     = lon
        self._state.alt_rel = alt
        return True

    def get_state(self) -> DroneState:
        return DroneState(**self._state.__dict__)


# ── Universal SDK facade ──────────────────────────────────────────────────────
class DroneSDK:
    """
    Universal drone SDK — single interface for any backend.

    Examples:
        sdk = DroneSDK("mavlink", connection="udp:127.0.0.1:14551")
        sdk = DroneSDK("airsim")
        sdk = DroneSDK("dji",    connection="192.168.1.100:8889")
        sdk = DroneSDK("sim")

    All backends expose the same methods:
        connect() / disconnect()
        arm() / disarm()
        takeoff(altitude) / land() / return_to_launch()
        send_velocity(vx, vy, vz, yaw_rate)
        send_position(lat, lon, alt)
        set_mode(mode_string)
        get_state() → DroneState
        get_telemetry() → dict   (HUD-compatible format)
    """

    def __init__(self, backend: str = "sim", connection: str = "",
                 baud: int = 57600):
        self.backend_name = backend.lower()
        self._backend: _BackendBase

        if self.backend_name == "mavlink":
            self._backend = _MAVLinkBackend(connection, baud)
        elif self.backend_name == "airsim":
            self._backend = _AirSimBackend()
        elif self.backend_name == "dji":
            host, _, port = connection.partition(":")
            self._backend = _DJIBackend(host, int(port) if port else 8889)
        elif self.backend_name == "sim":
            self._backend = _SimBackend()
        else:
            raise ValueError(f"Unknown backend: {backend}. "
                             f"Choose: mavlink, airsim, dji, sim")

        print(f"[DroneSDK] Backend: {self.backend_name}")

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def connect(self) -> bool:
        return self._backend.connect()

    def disconnect(self):
        self._backend.disconnect()

    # ── Flight commands ───────────────────────────────────────────────────────
    def arm(self) -> bool:
        return self._backend.arm()

    def disarm(self) -> bool:
        return self._backend.disarm()

    def takeoff(self, altitude: float = 3.0) -> bool:
        return self._backend.takeoff(altitude)

    def land(self) -> bool:
        return self._backend.land()

    def return_to_launch(self) -> bool:
        return self._backend.return_to_launch()

    def set_mode(self, mode: str) -> bool:
        return self._backend.set_mode(mode)

    # ── Motion commands ───────────────────────────────────────────────────────
    def send_velocity(self, vx: float = 0.0, vy: float = 0.0,
                      vz: float = 0.0, yaw_rate: float = 0.0) -> bool:
        """
        Body-frame velocity command.
        vx = forward (m/s)
        vy = right   (m/s)
        vz = down    (m/s)  — positive = descend
        yaw_rate = rotation (rad/s) — positive = clockwise
        """
        return self._backend.send_velocity(vx, vy, vz, yaw_rate)

    def send_position(self, lat: float, lon: float, alt: float) -> bool:
        return self._backend.send_position(lat, lon, alt)

    def hover(self) -> bool:
        return self.send_velocity(0, 0, 0, 0)

    # ── Telemetry ─────────────────────────────────────────────────────────────
    def get_state(self) -> DroneState:
        return self._backend.get_state()

    def get_telemetry(self) -> dict:
        """Returns HUD-compatible telemetry dict (same format as MAVLinkController)."""
        s = self._backend.get_state()
        return {
            "lat":         s.lat,
            "lon":         s.lon,
            "alt_msl":     s.alt_msl,
            "alt_rel":     s.alt_rel,
            "heading":     s.heading,
            "groundspeed": s.groundspeed,
            "airspeed":    s.groundspeed,
            "battery_v":   s.battery_v,
            "battery_pct": s.battery_pct,
            "mode":        s.mode,
            "armed":       s.armed,
            "gps_fix":     s.gps_fix,
            "satellites":  s.satellites,
            "roll":        s.roll,
            "pitch":       s.pitch,
            "yaw":         s.yaw,
            "vx":          s.vx,
            "vy":          s.vy,
            "vz":          s.vz,
            "lidar_dist":  s.lidar_dist,
        }

    @property
    def connected(self) -> bool:
        return self._backend.get_state().connected

    def __repr__(self):
        s = self.get_state()
        return (f"DroneSDK({self.backend_name}) "
                f"alt={s.alt_rel:.1f}m bat={s.battery_pct}% "
                f"mode={s.mode} armed={s.armed}")
