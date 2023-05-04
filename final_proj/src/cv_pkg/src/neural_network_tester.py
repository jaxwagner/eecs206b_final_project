import torch
import numpy as np
from network_model import GraspNet

model = GraspNet()
model.load_state_dict(torch.load('saved_model.pt'))
model.eval()

r = torch.randn((1,3,36,64))
d = torch.randn((1,1,36,64))

mean = np.array([31.931873, 16.615572, 71.92214 ])
std =  np.array([15.939826, 8.817323, 44.98397 ])

out = model(r,d)

print(out.detach().numpy())