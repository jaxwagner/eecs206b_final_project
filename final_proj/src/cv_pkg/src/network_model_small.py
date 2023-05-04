import torch
import torch.nn as nn
import torch.nn.functional as F

class DepthConv(nn.Module):
  def __init__(self):
    super(DepthConv, self).__init__()
    self.conv1 = nn.Conv2d(1, 16, kernel_size=3, padding=1)
    self.pool1 = nn.MaxPool2d(kernel_size=2, stride=2)
    self.conv2 = nn.Conv2d(16, 32, kernel_size=3, padding=1)
    # self.conv3 = nn.Conv2d(128, 256, kernel_size=3, padding=1)
    self.flatten = nn.Flatten()
    self.fc1 = nn.Linear(4608, 100)
    self.fc2 = nn.Linear(100, 50)

  def forward(self,x):
    x = self.conv1(x)
    x = nn.functional.relu(x)
    x = self.pool1(x)
    x = self.conv2(x)
    x = nn.functional.relu(x)
    x = self.pool1(x)
    # x = self.conv3(x)
    # x = nn.functional.relu(x)
    # x = self.pool1(x)
    x = self.flatten(x)
    x = self.fc1(x)
    x = nn.functional.relu(x)
    x = self.fc2(x)
    x = nn.functional.relu(x)
    return x

class RBGConv(nn.Module):
  def __init__(self):
    super(RBGConv, self).__init__()
    self.conv1 = nn.Conv2d(3, 16, kernel_size=3, padding=1)
    self.pool1 = nn.MaxPool2d(kernel_size=2, stride=2)
    self.conv2 = nn.Conv2d(16, 32, kernel_size=3, padding=1)
    # self.conv3 = nn.Conv2d(128, 256, kernel_size=3, padding=1)
    self.flatten = nn.Flatten()
    self.fc1 = nn.Linear(4608, 100)
    self.fc2 = nn.Linear(100, 50)

  def forward(self,x):
    x = self.conv1(x)
    x = nn.functional.relu(x)
    x = self.pool1(x)
    x = self.conv2(x)
    x = nn.functional.relu(x)
    x = self.pool1(x)
    # x = self.conv3(x)
    # x = nn.functional.relu(x)
    # x = self.pool1(x)
    x = self.flatten(x)
    x = self.fc1(x)
    x = nn.functional.relu(x)
    x = self.fc2(x)
    x = nn.functional.relu(x)
    return x

class GraspNetSmall(nn.Module):
  def __init__(self):
    super(GraspNetSmall, self).__init__()
    self.rgb_net = RBGConv()
    self.depth_net = DepthConv()
    self.fc1 = nn.Linear(50, 50)
    self.fc2 = nn.Linear(50, 25)
    self.fc3 = nn.Linear(25, 1)

    self.xy1 = nn.Linear(50, 50)
    self.xy2 = nn.Linear(50, 25)
    self.xy3 = nn.Linear(25, 2)

  def forward(self, rgb, depth):
    d = self.depth_net(depth)
    r = self.rgb_net(rgb)
    x_in = d + r
    x = self.fc1(x_in)
    x = nn.functional.relu(x)
    x = self.fc2(x)
    x = nn.functional.relu(x)
    x = self.fc3(x)
    # x = nn.functional.relu(x) 

    y = self.xy1(x_in)
    y = nn.functional.relu(y)
    y = self.xy2(y)
    y = nn.functional.relu(y)
    y = self.xy3(y)
    # y = nn.functional.relu(y)

 
    z = torch.cat((y,x), 1)
    return z