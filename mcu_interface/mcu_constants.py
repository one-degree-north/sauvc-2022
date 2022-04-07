THRUSTER_ONE = 0x10
THRUSTER_TWO = 0x11
THRUSTER_THREE = 0x12
THRUSTER_FOUR = 0x13
THRUSTER_FIVE = 0x14
THRUSTER_SIX = 0x15
THRUSTER_ALL = 0x1F

SERVO_LEFT = 0x20
SERVO_RIGHT = 0x21
SERVO_ALL = 0x2F

SENSOR_ACCEL = SENSOR_ACC = 0x30
SENSOR_MAG = SENSOR_MAGNET = 0x31
SENSOR_GYRO = SENSOR_GYR = 0x32
SENSOR_ORIENTATION = SENSOR_EULER = 0x33
SENSOR_QUATERNION = SENSOR_QUAT = 0x34
SENSOR_LINACCEL = SENSOR_LINEAR_ACCEL = 0x35
SENSOR_GRAVITY = 0x36
SENSOR_CALIBRATION = SENSOR_CALIB = 0x38
SENSOR_SYSTEM = 0x39
SENSOR_TEMP = SENSOR_TEMPERATURE = 0x3A
SENSOR_VOLT = SENSOR_VOLTAGE = 0x3B
SENSOR_DEPTH = 0x3C
SENSOR_BITMASK = 0x3D
SENSOR_KILLSWITCH = 0x3E
SENSOR_ALL = 0x3F

HEADER_RECV = 0x5c
HEADER_TRMT = 0x4d
FOOTER_RECV = 0xc5
FOOTER_TRMT = 0xd4

SUCCESS = 1
FAILURE = 0

FSM_OPERATIONAL = 1
FSM_STOPPED = 0

SYSTEM_ENABLED = 0x00
SYSTEM_FSM_STATE = 0x01

PWM_MIN = 1000
PWM_MID = PWM_NEUTRAL = 1500
PWM_MAX = 2000
