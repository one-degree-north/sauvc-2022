from dataclasses import dataclass
from utils import *


@dataclass
class BNOStatus:
    calib_sys: int
    calib_gyr: int
    calib_acc: int
    calib_mag: int
    sys_status: int
    sys_err: int

    def __init__(self):
        self.calib_sys = self.calib_mag = self.calib_gyr = self.calib_acc = 0
        self.sys_status = 0
        self.sys_err = 0


@dataclass
class SensorData:
    accel: Vector3
    mag: Vector3
    gyro: Vector3
    orientation: Vector3
    quaternion: Quaternion
    linaccel: Vector3
    gravity: Vector3
    status: BNOStatus
    temperature: float
    voltage: float
    depth: float
    killswitch: bool

    def __init__(self):
        accel = Vector3.new()
        mag = Vector3.new()
        gyro = Vector3.new()
        orientation = Vector3.new()
        quaternion = Quaternion.new()
        linaccel = Vector3.new()
        gravity = Vector3.new()
        status = BNOStatus()
        temperature = 0.0
        voltage = 12  # hopefully!
        depth = 0
        killswitch = False


@dataclass
class PWMValue:
    # intended to store values of microsecond PWM
    # 1000-2000
    value: int


@dataclass
class ThrusterServoData:
    thrusters: tuple[PWMValue]
    servos: tuple[PWMValue]

    def __init__(self):
        thrusters = tuple([PWMValue(1500) for i in range(6)])
        servos = tuple([PWMValue(1500) for i in range(2)])
