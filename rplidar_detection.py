#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import math
import numpy as np
import cv2
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class rplidarDetector:
    def __init__(self):
        rospy.init_node('rplidar_detection', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        rospy.Subscriber('/gameStage',Int32,self.gamestage_callback)
        self.angle_offset = 0
        self.gamestage = 0
        self.obscale_flag = 0
        self.pub2 = rospy.Publisher('/lidar_angle', Int32, queue_size=10)
        self.pub3 = rospy.Publisher('/has_obs',Bool,queue_size = 1)
        self.pub4 = rospy.Publisher('/park',Bool,queue_size=1)
        print('LiDAR is OK')
    def gamestage_callback(self,msg):
        self.gamestage = msg.data
    def road_detection(self, msg):
        angle = 0
        right_min = 25
        left_min = 25
        has_obs = False
        park_done=False
        print(self.gamestage)
#桥洞
        if self.gamestage == 6:
            #limit_distances_right = 0.67
            #limit_distances_left = 0.70
#            limit_distances = 0.67
#            right_min_index = 720
            angle = 0
            for i in range(720, 360, -1):#1 degree per 4
                if 1.1 > msg.ranges[i] < right_min and msg.ranges[i] > 0.05:
                    right_min = msg.ranges[i]
#                    right_min_index = i
#            left_min_index = 720
            for i in range(720, 1080):
                if 1.1 > msg.ranges[i] < left_min and msg.ranges[i] > 0.05:
                    left_min = msg.ranges[i]
#                    left_min_index = i
            if left_min < right_min and (right_min - left_min) > 0.08:
                angle = 25
                print('LidarTurnRight')
            elif left_min > right_min and (left_min - right_min) > 0.08:
                angle = -3
                print('LidarTurnLeft')
            else:
                angle = 0
            self.pub2.publish(angle)
        if self.gamestage == 12 or self.gamestage == 13:
            #limit_distances_right = 0.67
            #limit_distances_left = 0.70
#            limit_distances = 0.67
#            right_min_index = 720
            angle = 0
            for i in range(720, 540, -1):#1 degree per 4
                if 1.1 > msg.ranges[i] < right_min and msg.ranges[i] > 0.05:
                    right_min = msg.ranges[i]
#                    right_min_index = i
#            left_min_index = 720
            for i in range(720, 900):
                if 1.1 > msg.ranges[i] < left_min and msg.ranges[i] > 0.05:
                    left_min = msg.ranges[i]
#                    left_min_index = i
            if left_min < right_min and (right_min - left_min) > 0.05:
                angle = 25
                print('LidarTurnRight')
            elif left_min > right_min and (left_min - right_min) > 0.05:
                angle = -10
                print('LidarTurnLeft')
            else:
                angle = 0
            self.pub2.publish(angle)
        if self.gamestage == 11 or self.gamestage == 7:
            limit_distance_obscale = 1.0
            obscale_min  = 2
#            obscale_min_index = 720
            for i in range(640,800):
                if msg.ranges[i] < obscale_min and msg.ranges[i] > 0.05:
                    obscale_min = msg.ranges[i]
#                    obscale_min_index = i
            if obscale_min < limit_distance_obscale :
                has_obs = True
                print('has_obs')
            else:
                has_obs = False
            self.pub3.publish(has_obs)
        if self.gamestage == 103:
            num = 0
            sum = 0.0
            for i in range(0,7):
                if msg.ranges[i] < 9:
                    sum = sum + msg.ranges[i]
                    num += 1
            aver = sum/(float(num))
            if aver < 4.73:
                park_done=True
                print("---直行结束!-----")
            else:
                park_done=False
            self.pub4.publish(park_done)



        '''
        if self.parkFlag == True:
            sawYellow = True
            print('sawyellow')        
        if sawYellow:
            limit_distances_big = 0.35
            
            limit_distances_small = 0.28
            right_min_yellow = 25
            right_min_index_yellow = 720
            for i in range(600,480,-1):
                if msg.ranges[i] < right_min_yellow and msg.ranges[i] > 0.05:
                    right_min_yellow = msg.ranges[i]
                    right_min_index_yellow = i
                right_dist_yellow = right_min_yellow
            if right_dist_yellow > limit_distances_big:
                angle = 77
                print 'turnright(stop)'
            elif right_dist_yellow < limit_distances_small:
                #angle = 33
                print 'turnleft(stop)'
            else:
                angle = 50
                print('stright(stop)')
        '''
        sample = np.zeros((720, 1280))

        cv2.waitKey(1)
        return 0
        
    def callback(self, msg):
        offset = self.road_detection(msg)

if __name__ == '__main__':
    try:
        detector = rplidarDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
