import torch
import torch.nn as nn
import torch.nn.functional as F

class DepthConv(nn.Module):
    def __init__(self):
        super(DepthConv, self).__init__()
        num_filters1 = 64
        self.conv1 = nn.Conv2d(1, num_filters1, kernel_size=3, padding=1)
        self.bn1 = nn.BatchNorm2d(num_features = num_filters1)
        self.pool1 = nn.MaxPool2d(kernel_size=2) #original version stride = 2
        self.relu1 = nn.ReLU()

        num_filters2 = 128
        self.conv2 = nn.Conv2d(num_filters1, num_filters2, kernel_size=3, padding=1)
        self.bn2 = nn.BatchNorm2d(num_features = num_filters2)
        self.pool2 = nn.MaxPool2d(kernel_size=2)#stride = 2
        self.relu2 = nn.ReLU()

        num_filters3 = 256
        self.conv3 = nn.Conv2d(num_filters2, num_filters3, kernel_size=3, padding=1)
        self.bn3 = nn.BatchNorm2d(num_features = num_filters3)
        self.pool3 = nn.MaxPool2d(kernel_size=2)#stride = 2
        self.relu3 = nn.ReLU()
        
        self.flatten = nn.Flatten()
        self.fc1 = nn.Linear(256*8*4, 100)
        self.fc2 = nn.Linear(100, 50)

    def forward(self,x):
        # print(x.shape)
        x = self.conv1(x)
        x = self.bn1(x)
        x = self.pool1(self.relu1(x))

        x = self.conv2(x)
        x = self.bn2(x)
        x = self.pool2(self.relu2(x))

        x = self.conv3(x)
        x = self.bn3(x)
        x = self.pool3(self.relu3(x))

        x = self.flatten(x)
        x = self.fc1(x)
        x = nn.functional.relu(x)
        x = self.fc2(x)
        x = nn.functional.relu(x)
        return x

class RBGConv(nn.Module):
    def __init__(self):
        super(RBGConv, self).__init__()
        num_filters1 = 64
        self.conv1 = nn.Conv2d(3, num_filters1, kernel_size=3, padding=1)
        self.bn1 = nn.BatchNorm2d(num_features = num_filters1)
        self.pool1 = nn.MaxPool2d(kernel_size=2)#stride = 2
        self.relu1 = nn.ReLU()

        num_filters2 = 128
        self.conv2 = nn.Conv2d(num_filters1, num_filters2, kernel_size=3, padding=1)
        self.bn2 = nn.BatchNorm2d(num_features = num_filters2)
        self.pool2 = nn.MaxPool2d(kernel_size=2)#stride = 2
        self.relu2 = nn.ReLU()

        num_filters3 = 256
        self.conv3 = nn.Conv2d(num_filters2, num_filters3, kernel_size=3, padding=1)
        self.bn3 = nn.BatchNorm2d(num_features = num_filters3)
        self.pool3 = nn.MaxPool2d(kernel_size=2)#stride = 2
        self.relu3 = nn.ReLU()
        
        self.flatten = nn.Flatten()
        self.fc1 = nn.Linear(256*8*4, 100)
        self.fc2 = nn.Linear(100, 50)

    def forward(self,x):
        # print(x.shape)
        x = self.conv1(x)
        x = self.bn1(x)
        x = self.pool1(self.relu1(x))

        x = self.conv2(x)
        x = self.bn2(x)
        x = self.pool2(self.relu2(x))

        x = self.conv3(x)
        x = self.bn3(x)
        x = self.pool3(self.relu3(x))
        
        x = self.flatten(x)
        x = self.fc1(x)
        x = nn.functional.relu(x)
        x = self.fc2(x)
        x = nn.functional.relu(x)
        return x

class GraspNet(nn.Module):
    def __init__(self):
        super(GraspNet, self).__init__()
        self.rgb_net = RBGConv()
        self.depth_net = DepthConv()

        self.fc1 = nn.Linear(100, 50)
        self.fc2 = nn.Linear(50, 25)
        self.fc3 = nn.Linear(25, 1)

        self.p_dropout = 0.1

        self.drop = nn.Dropout()

        self.relu1 = nn.ReLU()
        self.relu2 = nn.ReLU()

        self.xy1 = nn.Linear(100, 50)
        self.xy2 = nn.Linear(50, 25)
        self.xy3 = nn.Linear(25, 2)

    def forward(self, rgb, depth):

        d = self.depth_net(depth)
        r = self.rgb_net(rgb)
        x_in = torch.cat((d,r), dim = 1)
        x = self.relu1(self.fc1(x_in))
        x = self.drop(x)
        x = self.relu2(self.fc2(x))
        x = self.drop(x)
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