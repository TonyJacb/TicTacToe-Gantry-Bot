#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from std_srvs.srv import SetBool, SetBoolResponse
import cv2
from cv_bridge import CvBridge
import numpy as np


class Vision:
    def __init__(self) -> None:
        '''
        Initializes the publishers, subscribers, camera and service server.
        '''
        rospy.loginfo_once("Started Vision Node")
        rospy.Service("bots_move", SetBool, self.service_handler)
        self.isitBot_move = False
        self.array_pub = rospy.Publisher("/positions", Int16MultiArray, queue_size=1)
        self.original_frame = rospy.Publisher("/image_raw", Image, queue_size=1)
        self.masked = rospy.Publisher("/masked_image", Image, queue_size=1)
        
        self.bridge = CvBridge()
        self.array = Int16MultiArray()
        self.vid = cv2.VideoCapture(0)
        self.find_zeros()
        
    def service_handler(self, request):
        if request.data:
            self.isitBot_move = request.data
            resp = SetBoolResponse(True, "Bot's move")
            return resp
        else:
            self.isitBot_move = request.data
            resp = SetBoolResponse(False, "Player Move")
            return resp
        
    def check_zeroes(self, sector):
        ''''
        Calc the white / black pixel ratio
        '''
        number_of_white_pix = np.sum(sector == 255)
        number_of_black_pix = np.sum(sector == 0)

        ratio = 100 * number_of_white_pix/ number_of_black_pix
        # print(ratio)
        return ratio

    def find_zeros(self):
        '''
        Finds out where the zeroes are in the play area.
        '''
        while True:
            ret, frame = self.vid.read()
            frame = frame[29:350, 149:409, :]
            hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            bound_lower = np.array([90, 30, 0])
            bound_upper = np.array([122, 118, 98])
            mask_zeros = cv2.inRange(hsvImage, bound_lower, bound_upper)
            
            mask_zeros = cv2.erode(mask_zeros, (3,3))
            mask_zeros = cv2.dilate(mask_zeros,(15,15))
            
            mask_zeros = cv2.line(mask_zeros, (87,237), (87,50), (255,255,255),3)
            mask_zeros = cv2.line(mask_zeros, (159,237), (159,50), (255,255,255),3)
            mask_zeros = cv2.line(mask_zeros, (14, 160), (242,160), (255,255,255), 3)
            mask_zeros = cv2.line(mask_zeros, (14, 100), (242,100), (255,255,255), 3)
            
            SECTOR_0 = mask_zeros[163:237, 14:87]
            SECTOR_1 = mask_zeros[163:237, 87:162]
            SECTOR_2 = mask_zeros[163:237, 162:242]
            
            SECTOR_3 = mask_zeros[100:163, 14:87]
            SECTOR_4 = mask_zeros[100:163, 87:162]
            SECTOR_5 = mask_zeros[100:163, 162:242]

            SECTOR_6 = mask_zeros[50:100, 14:87]
            SECTOR_7 = mask_zeros[50:100, 87:162]
            SECTOR_8 = mask_zeros[50:100, 162:242]
            
            sectors = [SECTOR_0, SECTOR_1, SECTOR_2, SECTOR_3,
                    SECTOR_4, SECTOR_5, SECTOR_6, SECTOR_7, SECTOR_8]
            
            result = [-1,-1,-1,-1,-1,-1,-1,-1,-1]
            for i in range(len(sectors)):
                ratio = self.check_zeroes(sectors[i])
                if i == 0 and ratio > 5:
                    result[i] = 0
                elif i == 1 and ratio > 12:
                    result[i] = 0
                elif i == 2 and ratio > 2:
                    result[i] = 0
                elif i == 3 and ratio > 18 + 4:
                    result[i] = 0
                elif i == 4 and ratio > 29:
                    result[i] = 0
                elif i == 5 and ratio > 15:
                    result[i] = 0
                elif i == 6 and ratio > 7.5 + 2:
                    result[i] = 0
                elif i == 7 and ratio > 17 + 3:
                    result[i] = 0
                elif i == 8 and ratio > 5:
                    result[i] = 0

            self.array.data = result
            if self.isitBot_move:
                self.array_pub.publish(self.array)
            
            self.masked.publish(self.bridge.cv2_to_imgmsg(mask_zeros))
            self.original_frame.publish(self.bridge.cv2_to_imgmsg(frame))
            

if __name__ == "__main__":
    rospy.init_node("vision_node")
    Vision()
    rospy.spin()
    