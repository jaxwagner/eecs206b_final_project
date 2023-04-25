import cv2
import numpy as np
import matplotlib.pyplot as plt
import imutils
import math



def make_dataset(lower, upper):
    vid = cv2.VideoCapture(0)
    
    frame_list = []
    target_points = []
    original_img_dims = None 

    for i in range(20):
        print('####### ITERATION ', i, ' ###############')
        ret, img = vid.read()
        if i == 0:
            height, width, channels = img.shape
            original_img_dims = (height, width)
        #cv2.imshow('image', img)
        #apply median blur, 15 means it's smoothing image 15x15 pixels
        
        blur = cv2.medianBlur(img,15)
        


        #convert to hsv
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        # print(hsv)
        # input()

        #color definition
        red_lower = lower
        red_upper = upper

        #red color mask (sort of thresholding, actually segmentation)
        mask = cv2.inRange(hsv, red_lower, red_upper)
        # mask = cv2.erode(mask, None, iterations = 2)
        # mask = cv2.dilate(mask, None, iterations = 2)

        masked = cv2.bitwise_and(img, img, mask=mask)

        gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        ret,thresh1 = cv2.threshold(gray,50,255,cv2.THRESH_BINARY)

        # cv2.imshow('image', thresh1)

        contours = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        # print(contours)

        centers_list = []

        if len(contours) > 0:
            for i in range(len(contours)):
                # cv2.drawContours(masked, contours, -1, 255, 3)
                # c = max(contours, key=cv2.contourArea)
                c = contours[i]
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                if M['m00'] != 0 and radius > 5:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    centers_list.append(center)
                    cv2.circle(img, center, 5, (0, 0, 255), -1)
                    cv2.circle(img, (int(x), int(y)), int(radius),(0, 255, 255), 2)

        #cv2.imshow('image', img)

        if len(centers_list) >= 2: # want to pick frames where we have two detections
            # pick two detections that are close to each other
            pair = closest_points(centers_list)
            print('### CENTERS LIST = ', centers_list)
            print('### CLOSEST PAIR = ', pair)
            resized = cv2.resize(img, (64,32), interpolation = cv2.INTER_AREA)
            cv2.imshow('image', resized)
            frame_list.append(resized)
            target_points.append(pair)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

    print('frame list shape = ', np.array(frame_list).shape)
    print('target points shape = ', np.array(target_points).shape)

    return np.array(frame_list), np.array(target_points), original_img_dims

def closest_points(points):
    closest_pair = None
    min_distance = float('inf')
    n = len(points)
    
    for i in range(n):
        for j in range(i+1, n):
            # Compute Euclidean distance between points i and j
            distance = math.sqrt((points[i][0] - points[j][0])**2 + (points[i][1] - points[j][1])**2)
            
            # Update closest pair and distance if this is the smallest distance so far
            if distance < min_distance:
                closest_pair = (points[i], points[j])
                min_distance = distance
    
    return closest_pair

def replace_color_with_avg_border_color(img, lower_color, upper_color):

    cv2.imshow('original image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Convert input image to HSV color space
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    mask = np.ones((img.shape[0], img.shape[1]))
    thresh = cv2.inRange(hsv_img, lower_color, upper_color)
    thresh = thresh / np.max(thresh)

    cv2.imshow('thresh', thresh)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    cv2.imshow('white mask', mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    mask = mask - thresh
    cv2.imshow('mask - thresh', mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    mask = cv2.GaussianBlur(mask, (5,5), 0)
    cv2.imshow('blurred_mask', mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    _,mask = cv2.threshold(mask,0.99,255,cv2.THRESH_BINARY)

    cv2.imshow('threshold_mask', mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    mask = mask / np.max(mask)

    img[:,:,0] = img[:,:,0]*mask 
    img[:,:,1] = img[:,:,1]*mask 
    img[:,:,2] = img[:,:,2]*mask 

    img_2 = cv2.GaussianBlur(img, (5,5), 0)
    for i in range(10):
        img_2 = cv2.GaussianBlur(img_2, (5,5), 0)
    
    cv2.imshow('blurred_hole_img', img_2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    mask_2 = np.ones_like(mask) - mask
    cv2.imshow('mask_2', mask_2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    img_2[:,:,0] = img_2[:,:,0]*mask_2 
    img_2[:,:,1] = img_2[:,:,1]*mask_2 
    img_2[:,:,2] = img_2[:,:,2]*mask_2 
    cv2.imshow('image_2', img_2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    img = img + img_2

    cv2.imshow('image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


                
    return img

if __name__ == '__main__':
    red_lower = np.array([1*255/100,63*255/100,50*255/100])
    red_upper = np.array([20*2.55,255,255])
    frame_list, target_points, original_img_dims = make_dataset(red_lower, red_upper)
    # print('frame_list = ', frame_list)
    # print('target points = ', target_points)
    for f in frame_list:
        f = replace_color_with_avg_border_color(f, red_lower, red_upper)
    cv2.imshow('image', f[0])
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print(f[0].shape)

        
    
