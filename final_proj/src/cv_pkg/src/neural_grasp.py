#!/usr/bin/env python
import rospy
import torch
import numpy as np

import cv2
import os
import time

import matplotlib.pyplot as plt


from cv_pkg.msg import grasp_msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from network_model import GraspNet
from network_model_small import GraspNetSmall



# citation: https://stackoverflow.com/questions/55377442/how-to-subscribe-and-publish-images-in-ros



class our_node(object):
    def __init__(self, neural = 'small'):
        # Params
        self.rgb_img = None
        self.depth_img = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(100)

        print('in init')

        self.plotting = False

        # Publishers
        self.pub = rospy.Publisher('grasp_info', grasp_msg, queue_size = 10)

        # Subscribers
        rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.depth_callback)
        rospy.Subscriber("/camera/color/image_raw",Image,self.rgb_callback)
        self.start_time = time.time()
        self.current_rgb_ts = None
        self.current_depth_ts = None

        if neural == 'small':
            self.mean_grasp = np.array([0.0, 0.0, 0.0])
            self.std_grasp =  np.array([1.0, 1.0, 1.0])
            self.model = GraspNetSmall()
            self.model.load_state_dict(torch.load('src/cv_pkg/src/grasp_nn_itr_45.pt'))
            self.model.eval()

        elif neural == 'large':
            self.mean_grasp = np.array([31.931873, 16.615572, 71.92214 ])
            self.std_grasp =  np.array([15.939826, 8.817323, 44.98397 ])
            self.model = GraspNet()
            self.model.load_state_dict(torch.load('src/cv_pkg/src/saved_model_25.pt'))
            self.model.eval()
        # dir = '/home/cc/ee106b/ros_worspace/final_proj/src/cv_pkg/src'
        

        self.px_per_cm = 22

    def rgb_callback(self, msg):
        self.rgb_img = self.br.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.current_rgb_ts = time.time()

    def depth_callback(self, msg):
        self.depth_img = self.br.imgmsg_to_cv2(msg, desired_encoding = '16UC1')
        self.current_depth_ts = time.time()


    def start(self):
        while not rospy.is_shutdown():         
            tic = time.time()
            # PUBLISHING SYNTAX
            m = grasp_msg()
            

            # print(time.time() - tic)

            # cv2.imshow('image',self.rgb_img)
            if self.rgb_img is not None:
                # print(self.depth_img.shape)
                print(type(self.rgb_img))
                point, theta = self.neural_grasp_method()
                # key = cv2.waitKey(300)
                m.grasp_x_cm = (point[0]-32) * 10 / self.px_per_cm 
                m.grasp_y_cm = (point[1]-18) * 10 / self.px_per_cm
                m.grasp_theta = theta*np.pi/180
                self.pub.publish(m)
                print('##### PUBLISHED ######')

    def classical_grasp_method(self):
        edges = self.get_edges()
        center, radius = self.getCenter(self.orange_lower, self.orange_upper) ###NOTE CHANGE COLOR HERE

        #for testing
        # radius = 40
        # center = [762,323]

        best_grasp = self.get_grasp_from_edges(edges, center, radius)
        point_theta = (best_grasp[0], self.convert_slope_to_theta(best_grasp[1]))
        return point_theta

    def neural_grasp_method(self):
        img = self.rgb_img
        depth = self.depth_img

        height, width, channels = img.shape
        d_height, d_width = depth.shape

        crop_left = width // 4
        crop_right = crop_left * 3

        cropped_top = height // 4
        cropped_bottom = cropped_top * 3

        cropped_image = img[cropped_top:cropped_bottom, crop_left:crop_right]

        # Resize the cropped image to 32 by 64 pixels
        resized_image = cv2.resize(cropped_image, (64, 36))

        
        ## DEPTH CROPPING
        d_left = d_width // 4
        d_right = d_left * 3

        d_top = d_height // 4
        d_bottom = d_top * 3
        cropped_depth = depth[d_top:d_bottom, d_left:d_right]

        resized_depth = cv2.resize(cropped_depth, (64, 36))

        #TODO see the type of r and d and reshape it and turn it into the torch
        # print('the size of the resized is:', resized_image.shape, resized_depth.shape)
        resized_image = resized_image.astype(np.int16)
        resized_depth = resized_depth.astype(np.int16)

        r = torch.tensor(resized_image, dtype = torch.float32).reshape(1, 3, 36, 64)
        d = torch.tensor(resized_depth, dtype = torch.float32).reshape(1, 1, 36, 64)
        out = self.model(r,d) #(x,y,theta in pixel)
        

        out = out.detach().numpy() * self.std_grasp + self.mean_grasp

        print('output value is:', out)

        point = (out[0,0], out[0,1])
        theta = out[0,2]

        if self.plotting:
            x = float(out[0][0])
            y = float(out[0][1])
            t_g = float(out[0][2])
            rgb_img = resized_image.reshape((36,64, 3))
            rgb_img = rgb_img.astype(int)
            plt.imshow(rgb_img)
            plt.plot(x,y, 'ro')
            tg_rad = t_g / 360.0 * np.pi * 2.0
            scale = 6
            tg_x = np.cos(tg_rad)*scale
            tg_y = -np.sin(tg_rad)*scale
            plt.plot(x+tg_x, y + tg_y, 'go')
            plt.show()

            depth_img = resized_depth.reshape((36,64))
            depth_img = depth_img.astype(int)
            plt.imshow(depth_img)
            plt.plot(x,y, 'ro')
            plt.plot(x+tg_x, y + tg_y, 'go')
            plt.show()

        # quit

        return point,theta



if __name__ == '__main__':
    rospy.init_node("our_node", anonymous=True)
    print('Is this working')
    my_node = our_node(neural = 'small')
    my_node.start()