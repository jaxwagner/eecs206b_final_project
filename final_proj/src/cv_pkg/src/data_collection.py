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

import cv2
import numpy as np
import matplotlib.pyplot as plt
import imutils
import math

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

      
        self.grasp_list = []
        self.rgb_frames = []
        self.depth_frames = []

        self.px_per_cm = 22

        self.img_resized = None##
        self.depth_resized = None

        self.iter = 1

    def rgb_callback(self, msg):
        self.rgb_img = self.br.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.current_rgb_ts = time.time()

    def depth_callback(self, msg):
        self.depth_img = self.br.imgmsg_to_cv2(msg, desired_encoding = '16UC1')
        self.current_depth_ts = time.time()

    def on_click(self, event, x, y, flags, param):
        # print('I am here')
        if event == cv2.EVENT_LBUTTONDOWN:
            print(self.iter, '-th point: ', x, ' ', y)
            t = input('input theta (degrees): ')
            self.grasp_list.append(np.array([float(x),float(y),float(t)]))
            self.rgb_frames.append(self.img_resized)
            self.depth_frames.append(self.depth_resized)

            self.iter += 1    
            
    def start(self):

        while not rospy.is_shutdown():         
            tic = time.time()
            # PUBLISHING SYNTAX
            m = grasp_msg()
            

            # print(time.time() - tic)

            if self.rgb_img is not None and self.depth_img is not None:
                print(type(self.rgb_img))

                img = None
                depth = None

                print('I am waiting')
                go = input('press enter to continue, q to quit: ')
                if go == 'q':
                    self.rgb_frames = np.array(self.rgb_frames)
                    self.depth_frames = np.array(self.depth_frames)
                    self.grasp_list = np.array(self.grasp_list)

                    np.save('rgb_frames', self.rgb_frames)
                    np.save('depth_frames', self.depth_frames)
                    np.save('grasp_targets', self.grasp_list)
                    break

                img = self.rgb_img
                depth = self.depth_img

                height, width, channels = img.shape
                d_height, d_width = depth.shape

                # cv2.imshow('img', img)
                key = cv2.waitKey(0)
                
                # Calculate the coordinates of the middle third
                crop_left = width // 4
                crop_right = crop_left * 3

                cropped_top = height // 4
                cropped_bottom = cropped_top * 3

                # Crop the middle third of the image
                cropped_image = img[cropped_top:cropped_bottom, crop_left:crop_right]

                # Resize the cropped image to 32 by 64 pixels
                resized_image = cv2.resize(cropped_image, (256, 144))

                
                ## DEPTH CROPPING
                d_left = d_width // 4
                d_right = d_left * 3

                d_top = d_height // 4
                d_bottom = d_top * 3
                cropped_depth = depth[d_top:d_bottom, d_left:d_right]

                resized_depth = cv2.resize(cropped_depth, (256, 144))

                self.img_resized = resized_image
                self.depth_resized = resized_depth


                # cv2.imshow('cropped', resized_image)
                # cv2.imshow('aa',cropped_image)
                cv2.namedWindow("Resized_Window", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Resized_Window", 600,400)
                cv2.imshow("Resized_Window", resized_image)
                cv2.setMouseCallback('Resized_Window', self.on_click)
                key = cv2.waitKey(0)

if __name__ == '__main__':
    rospy.init_node("our_node", anonymous=True)
    print('Is this working')
    my_node = our_node()
    my_node.start()