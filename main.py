from compute_alpha import ComputeAlpha
from cv_resource_manager import CvResourceManager
from gate_detection import GateDetection
from mcu_interface import *
from mcu_interface.interface import *

import time
# import threading

mcu = None
f = 50
theta_max = 0.2          # radians
speed_ms = 1750

camera_bottom_id = 1
camera_front_id = 0

def start():
  navigate = ComputeAlpha(mcu, f, theta_max)
  cv_manager = CvResourceManager(camera_bottom_id, camera_front_id)
  cam_frame = cv_manager.front_camera_read()

  gate_detect = GateDetection(cam_frame)
  y, x, gate_length = gate_detect.gate_info()
  center_gate = (int(y), int(x))
  height, width = img.shape[:2]
  center_frame = (int(height/2), int(width/2))

  distance_vertical, distance_horizontal = center_gate - center_frame
  distance_vertical = (150.0/gate_length)*distance_vertical
  distance_horizontal = (150.0/gate_length)*distance_horizontal
  distance_to = 100.0*10 # 10m

  navigate.ideal_yaw = np.arctan(distance_horizontal/distance_to)    # This is pretty useless because yaw code commented
  # self.navigate.ideal_pitch = np.arctan(distance_vertical/distance_to)
  navigate.ideal_pitch = 0
  navigate.ideal_roll = 0
  
  cmd_horizontal_thrusters(mcu, [speed_ms, speed_ms])
  navigate.run()
  # navigate_thread = threading.Thread(target = navigate.run)
  # navigate_thread.start()
  
if __name__ == '__main__':
  start()
