import serial
import time

class ComputeAlpha:
    def __init__(self, ser: Serial, f: int, theta_max: int):
        self.ser = ser
        self.f = f
        self.theta_max = theta_max
        
    def read_input(self):
        omega = None
        theta = None
        return omega, theta
    
    def write(self, alpha):
        self.ser.write(alpha)
    
    def compute_natural(self, omega1, omega2, t1, t2):
        return((omega2 - omega1)/(t2 - t1))
    
    def compute_alpha(self, omega1, omega2, ideal_theta, theta2, t1, t2):
        d_theta = ideal_theta - theta2
        delta_theta = min(abs(d_theta), self.theta_max)*d_theta/abs(d_theta)
        time = 1.0/self.f
        omega = omega2
        omega_ideal = delta_theta / time
        alpha = 2*(omega_ideal - omega)/time
        return alpha
        
    def run(self):
        omega1, theta1 = self.read_input()
        t1 = time.time()
        omega2, theta2 = self.read_input()
        t2 = time.time()
        
        omega_yaw1, omega_pitch1, omega_roll1 = omega1
        omega_yaw2, omega_pitch2, omega_roll2 = omega2
        theta_yaw1, theta_pitch1, theta_roll1 = theta1
        theta_yaw2, theta_pitch2, theta_roll2 = theta2
        
        ideal_pitch, ideal_roll = 0
        
        alpha_yaw = 0
        alpha_pitch = self.compute_alpha(omega_pitch1, 
                                         omega_pitch2, 
                                         ideal_pitch, 
                                         theta_pitch2,
                                         t1,
                                         t2
                                        )
        alpha_roll = self.compute_alpha(omega_roll1,
                                        omega_roll2,
                                        ideal_roll,
                                        theta_roll2,
                                        t1,
                                        t2
                                       )
        
        self.write([alpha_yaw, alpha_pitch, alpha_roll])