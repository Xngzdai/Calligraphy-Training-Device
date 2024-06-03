import numpy as np
import matplotlib.pyplot as plt
import os

cur_path = os.getcwd()
map = np.load(os.path.join(cur_path, "map/beng.npy"))

print(map)
plt.imshow(map[...,0])
plt.waitforbuttonpress()
plt.imshow(map[...,1])
plt.waitforbuttonpress()