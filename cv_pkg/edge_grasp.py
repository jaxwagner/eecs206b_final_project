#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import imutils
import time
from cv_pkg.msg import grasp_msg

import matplotlib.pyplot as plt

# citation: https://stackoverflow.com/questions/55377442/how-to-subscribe-and-publish-images-in-ros



class our_node(object):
    def __init__(self):
        # Params
        self.rgb_img = None
        self.depth_img = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(100)

        print('in init')

        # Publishers
        self.pub = rospy.Publisher('grasp_info', grasp_msg, queue_size = 10)

        # Subscribers
        rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.depth_callback)
        rospy.Subscriber("/camera/color/image_raw",Image,self.rgb_callback)
        self.start_time = time.time()
        self.current_rgb_ts = None
        self.current_depth_ts = None

        # orange true = 43, 98.4, 98.8
        self.orange_lower = np.array([10, 100, 100])
        self.orange_upper = np.array([25, 255, 255])

        self.purple_lower = np.array([250/2, 50, 50])
        self.purple_upper = np.array([290/2, 255, 255])

        self.yellow_lower = np.array([50/2, 50, 50])
        self.yellow_upper = np.array([70/2, 255, 255])

    def rgb_callback(self, msg):
        self.rgb_img = self.br.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.current_rgb_ts = time.time()

    def depth_callback(self, msg):
        self.depth_img = self.br.imgmsg_to_cv2(msg, desired_encoding = '16UC1')
        self.current_depth_ts = time.time()


    def start(self):
        # rospy.loginfo("Timing images")
        # count = self.count_limit
        #rospy.spin()
        while not rospy.is_shutdown():
            # rospy.loginfo('publishing image')
            # br = CvBridge() #JH commented it out 1202            
            tic = time.time()
            # PUBLISHING SYNTAX
            m = grasp_msg()
            

            # print(time.time() - tic)

            # cv2.imshow('image',self.rgb_img)
            if self.depth_img is not None:
                # print(self.depth_img.shape)
                print(type(self.depth_img))
                point, theta = self.classical_grasp_method()
                # key = cv2.waitKey(300)
                m.grasp_x_px = point[0]
                m.grasp_y_px = point[1]
                m.grasp_theta = theta
                self.pub.publish(m)
                print('##### PUBLISHED ######')

    def classical_grasp_method(self):
        edges = self.get_edges()
        center, radius = self.getCenter(self.orange_lower, self.orange_upper)

        #for testing
        # radius = 40
        # center = [762,323]

        best_grasp = self.get_grasp_from_edges(edges, center, radius)
        point_theta = (best_grasp[0], self.convert_slope_to_theta(best_grasp[1]))
        return point_theta

    def convert_slope_to_theta(self, slope):
        return np.arctan2(slope, 1)

    def get_grasp_from_edges(self, edges, center, radius):
        contours = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        
        print('contour shape = ', contours[0].shape)
        contours_list = []

        countour_image = cv2.drawContours(self.rgb_img, contours, -1, (0, 255, 0), 3)
        cv2.circle(countour_image,(center[0],center[1]), 3, (0,0,255), -1)

        for c in contours:
            failed = False
            M = cv2.moments(c)
            try:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                center_contour = (cx, cy)
                dist_to_ball = ((center[0] - cx)**2 + (center[1]-cy)**2)**0.5
                area = cv2.contourArea(c)
                perimeter = cv2.arcLength(c,True)
                c = c.reshape((-1,2))
                coefs = np.polyfit(c[:,0], c[:, 1], 1, rcond=None, full=False, w=None, cov=False)
                slope = coefs[0]
                print('#####################')
                print('dist to ball = ', dist_to_ball)
                print('perimeter = ', perimeter)
                print('area = ', area)
                print('center = ', center_contour)
                print('slope = ', slope)

                contours_list.append([dist_to_ball, perimeter, area, center_contour, slope])


            except:
                failed = True
                print('moments failed')

            # if not failed:
            #     # testing code to see if lines make sense
            #     start_point = (cx, cy)
            #     print(start_point)
            #     n = int(perimeter*0.1)
            #     end_point = (int(cx + n), int(cy + n*slope))
            #     thickness = 2
            #     color = (255, 255, 0)
            #     img = cv2.line(countour_image, start_point, end_point, color, thickness)
            #     cv2.circle(img,(cx,cy), 3, (0,0,255), -1)
            #     cv2.imshow('contour line onpython rgb', img)
            #     key = cv2.waitKey(0)

                # img2 = cv2.line(edges, start_point, end_point, color, thickness)
                # cv2.circle(img2,(cx,cy), 3, (255,0,0), -1)
                # cv2.imshow('contour line on edges', img2)
                # key = cv2.waitKey(0)

            ### return the best grasp point and angle
        best_dist = 100000
        min_perimeter = 200
        best_grasp = None

        if not failed:
            best_grasp = (contours_list[0][3], contours_list[0][4])

            for c in contours_list:
                dist_to_ball = c[0]
                dist_func = dist_to_ball - 2*radius
                perimeter = c[1]
                if perimeter > min_perimeter and dist_func < best_dist and dist_func > 0:
                    best_dist = dist_func
                    best_grasp = (c[3], c[4])

            # #### testing code to see if lines make sense
            # start_point = (best_grasp[0][0], best_grasp[0][1])
            # print(start_point)
            # n = int(perimeter)*5
            # end_point = (int(best_grasp[0][0] + n), int(best_grasp[0][1] + n*best_grasp[1]))
            # thickness = 2
            # color = (255, 255, 0)
            # img = cv2.line(countour_image, start_point, end_point, color, thickness)
            # cv2.circle(img,(best_grasp[0][0],best_grasp[0][1]), 3, (0,0,255), -1)
            # cv2.imshow('contour line onpython rgb', img)
            # key = cv2.waitKey(0)

        return best_grasp


        

    def get_edges(self):
        # print(self.rgb_img)
        blurred = cv2.GaussianBlur(self.rgb_img, (7,7), 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)

        # cv2.imshow('edges', edges)
        # key = cv2.waitKey(0)
        return edges


    def getCenter(self, lower, upper):
        #edits img to add a circle around the largest contour of the color seen inbetween lower and upper
        #returns [x,y] coordinates of center

        # cv2.imshow('image', self.rgb_img)
        # key = cv2.waitKey(0)
        #apply median blur, 15 means it's smoothing image 15x15 pixels
        blur = cv2.medianBlur(self.rgb_img,15)

        #convert to hsv
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        color_mask = cv2.inRange(hsv, lower, upper)
        color_mask = cv2.erode(color_mask, None, iterations = 2)
        color_mask = cv2.dilate(color_mask, None, iterations = 2)

        color_isolated = cv2.bitwise_and(self.rgb_img, self.rgb_img, mask=color_mask)

        ## FOR TESTING
        # cv2.imshow('image',color_isolated)
        # key = cv2.waitKey(0)

        gray = cv2.cvtColor(color_isolated, cv2.COLOR_BGR2GRAY)
        ret,thresh1 = cv2.threshold(gray,50,255,cv2.THRESH_BINARY)

        contours = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        centers_list = []

        if len(contours) > 0:
            # cv2.drawContours(masked, contours, -1, 255, 3)
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            try:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                centers_list.append(center)
                if radius > 20:
                    return([int(x), int(y)]), radius
            except:
                print("error")



if __name__ == '__main__':
    rospy.init_node("our_node", anonymous=True)
    print('Is this working')
    my_node = our_node()
    my_node.start()