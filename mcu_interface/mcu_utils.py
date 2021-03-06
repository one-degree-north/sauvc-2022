from dataclasses import dataclass
from utils import *
from mcu_interface.mcu_constants import *


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
class SystemStatus:
    sys_enable: int
    failed: int

    def __init__(self):
        self.sys_enable = 1
        self.failed = 0


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
        self.thrusters = tuple([PWMValue(1500) for i in range(6)])
        self.servos = tuple([PWMValue(1500) for i in range(2)])

    def update(self, param: int, value: int):
        if THRUSTER_ONE <= param <= THRUSTER_SIX:
            self.thrusters[param - THRUSTER_ONE].value = value
        if SERVO_LEFT <= param <= SERVO_RIGHT:
            self.servos[param - SERVO_LEFT].value = value

    def update_all_thrusters(self, values):
        for i in range(6):
            self.thrusters[i].value = values[i]

    def update_all_servos(self, values):
        for i in range(2):
            self.servos[i].value = values[i]


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
    system: SystemStatus
    outputs: ThrusterServoData
    temperature: float
    voltage: float
    depth: float
    killswitch: bool
    other: str

    def __init__(self):
        self.accel = Vector3.new()
        self.mag = Vector3.new()
        self.gyro = Vector3.new()
        self.orientation = Vector3.new()
        self.quaternion = Quaternion.new()
        self.linaccel = Vector3.new()
        self.gravity = Vector3.new()
        self.status = BNOStatus()
        self.system = SystemStatus()
        self.outputs = ThrusterServoData()
        self.temperature = 0.0
        self.voltage = 12  # hopefully!
        self.depth = 0
        self.killswitch = False
        self.other = ''


class Packet:
    header: int
    cmd: int
    param: int
    len: int
    data: list[int]
    lrc: int
    footer: int
    size: int

    def __init__(self, bytes):
        self.header = bytes[0]
        self.cmd = bytes[1]
        self.param = bytes[2]
        self.len = bytes[3]
        self.data = list(bytes[4:4 + self.len])
        self.lrc = bytes[4 + self.len]
        self.footer = bytes[4 + self.len + 1]
        self.size = len(bytes)


@dataclass
class IncompletePacket:
    header: int
    cmd: int
    param: int
    len: int
    data: list[int]
    lrc: int
    footer: int
    curr_size: int

    def __init__(self):
        self.curr_size = 0

    def lrc_check(self, expected: int) -> bool:
        lrc = 0
        lrc = (lrc + self.cmd) & 0xFF
        lrc = (lrc + self.param) & 0xFF
        lrc = (lrc + self.len) & 0xFF
        for byte in self.data:
            lrc = (lrc + byte) & 0xFF
        lrc = (((lrc ^ 0xFF) + 1) & 0xFF)
        return lrc == expected

    def is_complete(self) -> bool:
        return self.header and self.cmd and self.param and \
               self.len and self.data and self.lrc and self.footer and \
               len(self.data) == self.len

    def to_packet(self) -> Packet:
        return Packet(bytes([self.header, self.cmd, self.param, self.len, *self.data, self.lrc, self.footer]))

    def clear(self):
        self.header = self.cmd = self.param = self.len = self.lrc = self.footer = 0
        self.data = []
        self.curr_size = 0

    def add_byte(self, byte: int):
        byte &= 0xFF

        if self.curr_size == 0:
            if byte == HEADER_RECV:
                self.header = byte
        elif self.curr_size == 1:
            self.cmd = byte
        elif self.curr_size == 2:
            self.param = byte
        elif self.curr_size == 3:
            if byte >= 24:
                self.clear()
                return
            self.len = byte
        elif 4 <= self.curr_size < 4 + self.len:
            self.data.append(byte)
        elif self.curr_size == 4 + self.len:
            self.lrc = byte
            lrc_success = self.lrc_check(LRC(bytes([self.cmd, self.param, self.len, *self.data])))
            if not lrc_success:
                print("LRC failed on packet! continuing anyways...")
        elif self.curr_size == 4 + self.len + 1:
            if byte == FOOTER_RECV:
                self.footer = byte
            else:
                self.clear()
        elif self.curr_size >= 4 + self.len + 2:
            self.clear()
        else:
            self.clear()


# dummy boolean container class for signaling purposes
class Signal:
    enabled: bool

    def __init__(self, enabled: bool):
        self.enabled = enabled


def LRC(data: Union[list[int], bytes]) -> int:
    if type(data) == list:
        data = bytes(data)
    lrc = 0
    for byte in data:
        lrc = (lrc + byte) & 0xFF
    lrc = (lrc ^ 0xFF) + 1
    lrc &= 0xFF
    return lrc

def depth_mapping(value: int) -> float:
    volts = value / 0xFFFF * 3.3
    kPa = (volts - 0.5) * 400 + 100
    meters = (kPa - 100) / 9.78
    return meters


SENSOR_TYPES = {
        SENSOR_ACCEL: Vector3,
        SENSOR_MAG: Vector3,
        SENSOR_GYRO: Vector3,
        SENSOR_EULER: Vector3,
        SENSOR_QUAT: Quaternion,
        SENSOR_LINACCEL: Vector3,
        SENSOR_GRAVITY: Vector3,
        SENSOR_CALIB: int,
        SENSOR_SYSTEM: int,
        SENSOR_TEMP: int,
        SENSOR_VOLT: float,
        SENSOR_DEPTH: int,
        SENSOR_KILLSWITCH: int
}
