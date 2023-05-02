import numpy as np

depth = np.load('depth_frames.npy')
grasp = np.load('grasp_targets.npy')
img = np.load('rgb_frames.npy')

print(depth, depth.shape)
print(img, img.shape)
print(grasp, grasp.shape, (grasp[0,0].type))