#ifndef BNO055_H_
#define BNO055_H_

#ifndef INT_SHORTHAND
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define u64 uint64_t
#define i8 int8_t
#define i16 int16_t
#define i32 int32_t
#define i64 int64_t
#endif

void bno_setup(u8 addr);
void bno_configure();
void bno_read();
float* bno_accel();
float* bno_magnet();
float* bno_gyro();
float* bno_orientation();
float* bno_quaternion();
float* bno_lin_accel();
float* bno_gravity();
float bno_temperature();
u8 bno_calibration();
u8 bno_sys_status();
u8 bno_sys_error();
void bno_accel_offset(float x, float y, float z);
void bno_magnet_offset(float x, float y, float z);
void bno_gyro_offset(float x, float y, float z);
void bno_accel_radius(float r);
void bno_magnet_radius(float r);

#endif /* BNO055_H_ */

