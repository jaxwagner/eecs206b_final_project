import cv2
import numpy as np
import matplotlib.pyplot as plt
import imutils
import math



def make_dataset(lower, upper):
    vid = cv2.VideoCapture(0)
    
    frame_list = []
    target_points = []
    target_theta = []

    for i in range(20):
        print('####### ITERATION ', i, ' ###############')
        ret, img = vid.read()
        height, width, channels = img.shape
        original_img_dims = (height, width)

        cv2.imshow('img', img)
        key = cv2.waitKey(0)
        
        # Calculate the coordinates of the middle third
        crop_left = width // 3
        crop_right = crop_left * 2

        cropped_top = height // 3
        cropped_bottom = cropped_top * 2

        # Crop the middle third of the image
        cropped_image = img[cropped_top:cropped_bottom, crop_left:crop_right]

        # Resize the cropped image to 32 by 64 pixels
        resized_image = cv2.resize(cropped_image, (64, 32))

        cv2.imshow('cropped', resized_image)
        key = cv2.waitKey(0)
        
        target_x = input('input target x')
        target_y = input('input target y')
        terget_theta = input('input target theta')

        frame_list.append(resized_image)
        target_points.append((target_x, target_y))
        target_theta.append(target_theta)
    
    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()




if __name__ == '__main__':
    red_lower = np.array([1*255/100,63*255/100,50*255/100])
    red_upper = np.array([20*2.55,255,255])
    make_dataset(red_lower, red_upper)
   

        
    
