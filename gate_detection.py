import cv2
import numpy as np

class GateDetection:
    def __init__(self, img):
        img = cv2.blur(img, (7, 7))
        self.img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
    def calculate_center(self):
        rectangle1, rectangle2 = self.orange_rectangles
        left_x = np.min(rectangle1[0], rectangle2[0])
        right_x = np.max(rectangle1[0], rectangle2[0]) + rectangle2[2]
        rectangle1_y = rectangle1[1] + 0.5*rectangle1[3]
        rectangle2_y = rectangle2[1] + 0.5*rectangle2[3]
        return (np.mean([left_x, right_x]), np.mean([rectangle1_y, rectangle2_y]))
        
    def find_orange(self):
        lower_orange = np.array([5, 50, 50])
        upper_orange = np.array([35,255,255])
        mask_orange = cv2.inRange(self.img_hsv, lower_orange, upper_orange)
        self.colored = self.img_hsv.copy()
        self.colored[np.where(mask_orange==0)] = 0
    
    def orange_contour_info(self);
        gray = cv2.cvtColor(self.colored, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return [0, 0, 0, 0]
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[0:2]
        bounding_rectangles = [np.asarray(cv2.bounding_rectangle(contours[0])),
                               np.asarray(cv2.bounding_rectangle(contours[1]))]
        return bounding_rectangles
    
    def run(self):
        self.find_orange()
        self.orange_rectangles = self.orange_contour_info()
        return self.calculate_center()
