"""
DroneAdapter — abstract base class for drone integration.

To integrate any drone, subclass DroneAdapter and implement the methods.
The SDK calls these methods automatically during operation.

Supported integrations (examples included in sdk/adapters/):
  - MAVLinkAdapter    — ArduCopter / PX4 via pymavlink
  - AirSimAdapter     — Microsoft AirSim simulator
  - DJIAdapter        — DJI SDK via MSDK/OSDK bridge
  - CustomAdapter     — any drone with a velocity API

Example:
    class MyDrone(DroneAdapter):
        def send_velocity(self, cmd): ...
        def get_telemetry(self): ...
        def arm(self): ...
        def takeoff(self, alt): ...
        def land(self): ...
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class VelocityCommand:
    """
    Body-frame velocity setpoint sent to the drone every control cycle.
    All values normalised to [-1.0, 1.0] by the PID controller.
    Scale to your drone's actual m/s or deg/s in the adapter.
    """
    yaw:      float = 0.0   # + = rotate right
    forward:  float = 0.0   # + = move forward
    vertical: float = 0.0   # + = move up
    lateral:  float = 0.0   # + = move right (optional)


@dataclass
class TelemetryData:
    """
    Telemetry snapshot returned by the drone adapter.
    Fill only the fields your drone supports — rest stay at defaults.
    """
    # Attitude
    roll:        float = 0.0   # degrees
    pitch:       float = 0.0   # degrees
    yaw:         float = 0.0   # degrees
    roll_rate:   float = 0.0   # deg/s
    pitch_rate:  float = 0.0   # deg/s
    yaw_rate:    float = 0.0   # deg/s

    # Position
    lat:         float = 0.0
    lon:         float = 0.0
    alt_rel:     float = 0.0   # metres AGL
    alt_msl:     float = 0.0   # metres MSL
    heading:     float = 0.0   # degrees 0-360

    # Velocity
    vx:          float = 0.0   # m/s north
    vy:          float = 0.0   # m/s east
    vz:          float = 0.0   # m/s down
    groundspeed: float = 0.0   # m/s

    # Power
    battery_v:   float = 0.0
    battery_pct: int   = -1    # -1 = unknown

    # GPS
    gps_fix:     int   = 0     # 0=none 3=3D 4=DGPS
    satellites:  int   = 0

    # State
    armed:       bool  = False
    mode:        str   = "UNKNOWN"
    connected:   bool  = False

    # Rangefinder / LiDAR
    lidar_dist:  float = -1.0  # metres, -1 = no sensor


class DroneAdapter(ABC):
    """
    Abstract drone adapter. Implement this to integrate any drone.
    All methods have safe no-op defaults except send_velocity and get_telemetry.
    """

    # ── Required ──────────────────────────────────────────────────────────────

    @abstractmethod
    def send_velocity(self, cmd: VelocityCommand) -> None:
        """Send velocity setpoint to the drone. Called every control cycle (~30Hz)."""

    @abstractmethod
    def get_telemetry(self) -> TelemetryData:
        """Return latest telemetry snapshot. Called every display frame."""

    # ── Optional — override as needed ─────────────────────────────────────────

    def connect(self) -> bool:
        """Connect to the drone. Return True on success."""
        return True

    def disconnect(self) -> None:
        """Disconnect cleanly."""

    def arm(self) -> bool:
        """Arm motors. Return True on success."""
        return False

    def disarm(self) -> bool:
        """Disarm motors."""
        return False

    def takeoff(self, altitude_m: float) -> bool:
        """Takeoff to altitude_m metres AGL."""
        return False

    def land(self) -> bool:
        """Command landing."""
        return False

    def return_to_launch(self) -> bool:
        """Return to launch / home point."""
        return False

    def set_mode(self, mode: str) -> bool:
        """Set flight mode (e.g. 'GUIDED', 'LOITER', 'AUTO')."""
        return False

    def is_connected(self) -> bool:
        """Return True if drone is connected and responsive."""
        return False

    def stop(self) -> None:
        """Emergency stop — zero all velocities."""
        self.send_velocity(VelocityCommand())
