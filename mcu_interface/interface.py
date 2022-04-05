from abc import ABC, abstractmethod, abstractproperty
from typing import Union
import serial
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

    def start(self):
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
