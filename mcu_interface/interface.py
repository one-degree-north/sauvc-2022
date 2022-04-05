from abc import ABC, abstractmethod, abstractproperty
from typing import Union
import serial
from queue import Queue
import threading
from utils import *
from mcu_utils import *


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
        self.write_queue = Queue()
        self.read_thread = threading.Thread(target=self._read)
        self.build_packet = Packet
        self.enable_signal = Signal(False)
        self.data = super().sensor_data

    def _write(self):
        while self.enable_signal.enabled:
            if self.write_queue.not_empty:
                pkt: Packet = self.write_queue.get_nowait()
                self.ser.write(pkt.data)

    def _read(self):
        while self.enable_signal.enabled:
            new_bytes = self.ser.read_all()
            # TODO: DO DO DO DO DO

    def start(self):
        self.enable_signal.enabled = True
        if not self.ser.is_open:
            self.ser.open()

    def stop(self):
        if self.ser.is_open:
            self.ser.close()

    def send_bytes(self, data: bytes):
        self.ser.write(data)

    def send_data(self, data: Union[list[Union[int, str]], str]):
        if type(data) == str:
            self.send_bytes(str.encode('latin'))
            return
        if data:
            if type(data[0]) == str:
                self.send_bytes(bytes("".join(data), 'latin'))
            elif type(data[0]) == int:
                self.send_bytes(bytes(data))
