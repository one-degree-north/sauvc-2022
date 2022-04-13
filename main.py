from compute_alpha import ComputeAlpha
from cv_resource_manager import CvResourceManager
from gate_detection import GateDetection
from mcu_interface.interface import *

import threading

class Main:
  def __init__(self):
    self.mcu = None
    self.f = 50
    self.theta_max = 0.2          # radians
    self.navigate = ComputeAlpha(mcu, f, theta_max)
    
    camera_bottom_id = 1
    camera_front_id = 0
    self.cv_manager = CvResourceManager(camera_bottom_id, camera_front_id)
    cam_frame = self.cv_manager.front_camera_read()
    
    self.gate_detect = GateDetection(cam_frame)
    y, x, gate_length = self.gate_detect.run()
    center_gate = (int(y), int(x))
    height, width = img.shape[:2]
    center_frame = (int(height/2), int(width/2))
    
    distance_vertical, distance_horizontal = center_gate - center_frame
    distance_vertical = (150.0/gate_length)*distance_vertical
    distance_horizontal = (150.0/gate_length)*distance_horizontal
    distance_to = 100.0*10 # 10m
    
    self.navigate.ideal_yaw = np.arctan(distance_horizontal/distance_to)
    self.navigate.ideal_pitch = np.arctan(distance_vertical/distance_to)
    self.navigate.ideal_roll = 0
    
  def run(self):
    navigate_thread = threading.Thread(target = self.navigate.run)
    navigate_thread.start()
    
