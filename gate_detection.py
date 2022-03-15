import cv2
import numpy as np

class GateDetection:
    def __init__(self, img):
        #img = cv2.imread(path)
        img = cv2.blur(img, (7, 7))
        self.img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
    def find_orange(self):
        lower_orange = np.array([5, 50, 50])
        upper_orange = np.array([35,255,255])
        mask_orange = cv2.inRange(self.img_hsv, lower_orange, upper_orange)
        output_hsv = self.img_hsv.copy()
        output_hsv[np.where(mask_orange==0)] = 0
        return output_hsv
    
    def orange_contour_info(self, colored);
        gray = cv2.cvtColor(colored, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return [0, 0, 0, 0]
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[0:2]
        bounding_rectangles = [np.asarray(cv2.bounding_rectangle(contours[0])),
                               np.asarray(cv2.bounding_rectangle(contours[1]))]
        return bounding_rectangles
    
    def run(self):
        orange_rectangles = self.orange_contour_info(self.find_orange(self.image_hsv))
        rectangle1, rectangle2 = orange_rectangles[0], orange_rectangles[1]
        rectangle_coord = calculate_center(rectangle1, rectangle2)
        return rectangle_coord