from abc import ABC, abstractmethod, abstractproperty
from typing import Union
import serial
import struct
from queue import Queue
import threading
from utils import *
from mcu_utils import *
from mcu_constants import *


class MCUInterface(ABC):
    sensor_data: SensorData

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def send_bytes(self, data: bytes):
        pass

    @abstractmethod
    def send_data(self, data: Union[list[Union[int, str]], str]):
        pass


class UARTMCUInterface(MCUInterface):
    def __init__(self, port):
        # create the serial object with no port since we do not want to connect
        self.ser = serial.Serial(None, 230400)
        self.ser.port = port
        self.write_thread = threading.Thread(target=self._write)
        self.write_queue = Queue()  # of Packets
        self.read_thread = threading.Thread(target=self._read)
        self.build_packet = IncompletePacket()
        self.enable_signal = Signal(False)
        self.data = super().sensor_data

    def _write(self):
        while self.enable_signal.enabled:
            if self.write_queue.not_empty:
                pkt: Packet = self.write_queue.get_nowait()
                if pkt:
                    self.ser.write(pkt.data)

    def _read(self):
        while self.enable_signal.enabled:
            new_bytes = self.ser.read_all()
            for byte in new_bytes:
                self.build_packet.add_byte(byte)
                if self.build_packet.is_complete():
                    self._parse(self.build_packet.to_packet())

    def _parse(self, packet: Packet):
        # I wish I could use match case here...
        # too bad we need to maintain Py3.9 compatibility
        if packet.cmd == 0x00:
            # echo or hello
            self.data.other = bytes(packet.data).decode('latin')
        elif packet.cmd == 0x0A:
            # get system attribute
            if not packet.data:
                return
            self.data.system.sys_enable = packet.data[0]
        elif packet.cmd == 0x0F:
            # success
            if not packet.data:
                self.data.system.failed = 1
                return
            self.data.system.failed = packet.data[0]
        elif packet.cmd == 0x1A:
            # thruster positions
            if packet.len == 2 and THRUSTER_ONE <= packet.param <= THRUSTER_SIX:
                self.data.outputs.update(packet.param, struct.unpack('>H', packet.data)[0])
            if packet.len == 12 and packet.param == THRUSTER_ALL:
                self.data.outputs.update_all_thrusters(struct.unpack('>HHHHHH', packet.data))
        elif packet.cmd == 0x2A:
            # servo positions
            if packet.len == 2 and SERVO_LEFT <= packet.param <= SERVO_RIGHT:
                self.data.outputs.update(packet.param, struct.unpack('>H', packet.data)[0])
            if packet.len == 4 and packet.param == SERVO_ALL:
                self.data.outputs.update_all_servos(struct.unpack('>HH', packet.data))
        elif packet.cmd == 0x33:
            # some type of sensor data
            new_data = None
            if packet.len == 2:
                new_data = struct.unpack('>H', packet.data)[0]
            elif packet.len == 4:
                new_data = struct.unpack('>f', packet.data)[0]
            elif packet.len == 12:
                new_data = Vector3.from_arr(struct.unpack('>fff', packet.data))
            elif packet.len == 16:
                new_data = Quaternion.from_arr(struct.unpack('>ffff', packet.data)

            # make sure type is correct
            if packet.param in SENSOR_TYPES:
                assert type(new_data) == SENSOR_TYPES[packet.param], f"wrong type for packet type {packet.param}, expected {SENSOR_TYPES[packet.param]}, got {type(new_data)}"
            else:
                print("?????????")

            # every possible packet...
            if packet.param == SENSOR_ACCEL:
                self.data.accel = new_data
            elif packet.param == SENSOR_MAG:
                self.data.mag = new_data
            elif packet.param == SENSOR_GYRO:
                self.data.gyro = new_data
            elif packet.param == SENSOR_EULER:
                self.data.orientation = new_data
            elif packet.param == SENSOR_QUATERNION:
                self.data.quaternion = new_data
            elif packet.param == SENSOR_LINACCEL:
                self.data.linaccel = new_data
            elif packet.param == SENSOR_GRAVITY:
                self.data.gravity = new_data
            elif packet.param == SENSOR_CALIBRATION:
                self.data.status.calib_sys = new_data & 0b11000000
                self.data.status.calib_gyr = new_data & 0b00110000
                self.data.status.calib_acc = new_data & 0b00001100
                self.data.status.calib_mag = new_data & 0b00000011
            elif packet.param == SENSOR_SYSTEM:
                self.data.status.sys_status = new_data & 0xFF00
                self.data.status.sys_err    = new_data & 0x00FF
            elif packet.param == SENSOR_TEMP:
                self.data.temperature = new_data
            elif packet.param == SENSOR_VOLT:
                self.data.voltage = new_data
            elif packet.param == SENSOR_DEPTH:
                self.data.depth = depth_mapping(new_data)
            elif packet.param == SENSOR_KILLSWITCH:
                self.data.killswitch = new_data

    def start(self):
        self.enable_signal.enabled = True
        if not self.ser.is_open:
            self.ser.open()

    def stop(self):
        if self.ser.is_open:
            self.ser.close()

    def send_bytes(self, data: bytes):
        self.ser.write(data)

    def send_packet(self, command: int, param: int, length: int, data: Union[bytes, list[int]]):
        to_lrc = bytes([command, param, length]) + bytes(data)
        lrc = LRC(to_lrc)

        trmt = bytes([HEADER_TRMT, command, param, length]) + bytes(data) + bytes([lrc, FOOTER_TRMT])

        self.send_bytes(trmt)

    def send_data(self, data: Union[list[Union[int, str]], str]):
        if type(data) == str:
            self.send_bytes(str.encode('latin'))
            return
        if data:
            if type(data[0]) == str:
                self.send_bytes(bytes("".join(data), 'latin'))
            elif type(data[0]) == int:
                self.send_bytes(bytes(data))
