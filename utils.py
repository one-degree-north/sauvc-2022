# mcu_utils.py
# various utilities for the other scripts.

from typing import Union, List, Tuple

INVALID = 0x82D3F2


class Vector3:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def set_axis(self, axis: int, value: float):
        if axis == 0:
            self.x = value
        elif axis == 1:
            self.y = value
        elif axis == 2:
            self.z = value
        else:
            raise IndexError("Invalid Axis")

    def get_axis(self, axis: int):
        if axis == 0:
            return self.x
        elif axis == 1:
            return self.y
        elif axis == 2:
            return self.z
        else:
            raise IndexError("Invalid Axis")

    def is_valid(self):
        return self.x != INVALID and self.y != INVALID and self.z != INVALID

    def __str__(self):
        return f"({self.x:.4f}, {self.y:.4f}, {self.z:.4f})"

    @staticmethod
    def new():
        return Vector3(0, 0, 0)

    @staticmethod
    def invalid():
        return Vector3(INVALID, INVALID, INVALID)

    @staticmethod
    def from_arr(i: Union[List, Tuple]):
        return Vector3(i[0], i[1], i[2])


class Vector2:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    @staticmethod
    def new():
        return Vector2(0, 0)


class Quaternion:
    def __init__(self, w: float, x: float, y: float, z: float):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    @staticmethod
    def new():
        return Quaternion(0, 0, 0, 0)
