import numpy as np
import cv2 as cv
from cv_bridge import CvBridge

class imageProcessing:
    def __init__(self, image_msg):
        bridge = CvBridge()
        self.original = bridge.imgmsg_to_cv2(image_msg)
        self.image = cv.cvtColor(self.original, cv.COLOR_BGR2HSV)
        self.w = 0
        
        #[hue, saturation, value]
        self.lower = np.array([170, 100, 0], dtype = "uint8")
        self.upper = np.array([180, 255, 255], dtype = "uint8")
        self.mask_h = cv.inRange(self.image, self.lower, self.upper)
        #
        self.lower_l = np.array([0, 100, 150], dtype = "uint8")
        self.upper_l = np.array([10, 255, 255], dtype = "uint8")
        self.mask_l= cv.inRange(self.image, self.lower_l, self.upper_l)
        #masks
        self.mask = self.mask_h | self.mask_l
        
        
    def find_centroid(self):
        h, self.w, d = self.original.shape
        search_top = int(5*h/6)
        search_bot = search_top + 30
        self.mask[0:search_top, 0:self.w] = 0
        self.mask[search_bot:h, 0:self.w] = 0
        M = cv.moments(self.mask)
        return M
    
    
    def show_image(self):
        #cv.imshow('mask', self.mask)
        cv.imshow('original', self.original)
        cv.waitKey(1)

