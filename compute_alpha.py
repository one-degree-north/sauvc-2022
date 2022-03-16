import serial
import time

class ComputeAlpha:
    def __init__(self, ser: Serial):
        self.ser = ser
        
    def read_input(self):
        return self.ser.read(3)
    
    def write_derivative(self, alpha):
        self.ser.write(alpha)
    
    def compute_derivative(self):
        omega_x1, omega_y1, omega_z1 = self.read_input()
        t1 = time.time()
        time.sleep(0.25)
        omega_x2, omega_y2, omega_z2 = self.read_input()
        t2 = time.time()

        alpha_x = -(omega_x2 - omega_x1)/(t2 - t1)
        alpha_y = -(omega_y2 - omega_y1)/(t2 - t1)
        alpha_z = -(omega_z2 - omega_z1)/(t2 - t1)
        self.write_derivative([alpha_x, alpha_y, alpha_z])