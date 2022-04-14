import serial
import time
from mcu_interface.commands import *
from utils import *

class ComputeAlpha:
    def __init__(self, mcu, f: int, theta_max: int):
        self.mcu = mcu
        self.f = f
        self.theta_max = theta_max
        self.ideal_yaw = 0
        self.ideal_pitch = 0
        self.ideal_roll = 0
        self.read_input()
        
    def read_input(self):
        self.omega = None
        self.theta = None
        self.t = time.time()
        
    def map(self, value, imin, imax, omin, omax):
        return (value - imin) / (imax - imin) * (omax - omin) + omin 
    
    def write(self, alpha):
        for i in range(len(alpha)):
            alpha[i] = self.map(alpha[i], -9, 9, 1000, 2000)
        alpha[0] = alpha[0] * 3
        alpha[1] = alpha[1] * 3
        cmd_all_thrusters(mcu, alpha)
    
    def compute_natural(self, omega1, omega2, t1, t2):
        return((omega2 - omega1)/(t2 - t1))
    
    def compute_alpha(self, omega1, omega2, ideal_theta, theta2, time_diff):
        d_theta = ideal_theta - theta2
        delta_theta = min(abs(d_theta), self.theta_max)*d_theta/abs(d_theta)
        omega_ideal = delta_theta / time_diff
        alpha = 2*(omega_ideal - omega2)/time_diff
        alpha = min(3, max(-3, alpha)) 
        return alpha
        
    def run(self):
        while True:
            time.sleep(1/self.f)
            thrusters = [0, 0, 0, 0, 0, 0] # horizontal, vertical
            omega1, theta1 = self.omega, self.theta
            t_temp = self.t
            self.read_input()
            omega2, theta2 = self.omega, self.theta
            t_diff = self.t - t_temp

            omega_yaw1, omega_pitch1, omega_roll1 = omega1
            omega_yaw2, omega_pitch2, omega_roll2 = omega2
            theta_yaw1, theta_pitch1, theta_roll1 = theta1
            theta_yaw2, theta_pitch2, theta_roll2 = theta2

            alpha_yaw = self.compute_alpha(omega_yaw1, 
                                           omega_yaw2, 
                                           self.ideal_yaw, 
                                           theta_yaw2,
                                           t_diff
                                          )
            thrusters[0] += alpha_yaw
            thrusters[1] += alpha_yaw

            alpha_pitch = self.compute_alpha(omega_pitch1, 
                                             omega_pitch2, 
                                             self.ideal_pitch, 
                                             theta_pitch2,
                                             t_diff
                                            )
            # Positive: up, negative: down
            thrusters[2] += alpha_pitch
            thrusters[3] += alpha_pitch
            thrusters[4] -= alpha_pitch
            thrusters[5] -= alpha_pitch

            alpha_roll = self.compute_alpha(omega_roll1,
                                            omega_roll2,
                                            self.ideal_roll,
                                            theta_roll2,
                                            t_diff
                                           )
            thrusters[2] -= alpha_roll
            thrusters[3] += alpha_roll
            thrusters[4] -= alpha_roll
            thrusters[5] += alpha_roll
            self.write(thrusters)
