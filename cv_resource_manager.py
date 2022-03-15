import cv2

class CvResourceManager:
    def __init__(self, camera_bottom_id, camera_front_id):
        self.camera_bottom_id = camera_bottom_id
        self.camera_front_id = camera_front_id
        self.camera_bottom = cv2.VideoCapture(self.camera_bottom_id)
        self.camera_front = cv2.VideoCapture(self.camera_front_id)
        
    def bottom_camera_read():
        _, frame = self.camera_bottom.read()
        return frame
    
    def front_camera_read():
        _, frame = self.caamera_front.read()
        return frame
    
    def close_cameras():
        