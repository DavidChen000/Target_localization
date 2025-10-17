import cv2
import math
import numpy as np
from natsort import natsorted

# Set initial state
class AppState:
    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)


# output:x_av, y_av, dis, count
# count:effective point number
def pointxyz(x, y, verts, cls, xyxy):
    dis = 0
    x_av = 0
    y_av = 0
    num = 0
    zxy_list = []
    for m in range(-5, 5):
        for n in range(-5, 5):
            my = y
            wx = x + m
            wy = my + n
            if wx < 0:
                wx = 0
            if wx >= 640:
                wx = 639
            if wy < 0:
                wy = 0
            if wy >= 480:
                wy = 479
            px = verts[(wy) * 640 + wx][0]
            py = verts[(wy) * 640 + wx][1]
            pz = verts[(wy) * 640 + wx][2]
            if (pz > 0.3) and (pz < 15):
                zxy_list.append([pz, px, py])
                num += 1
    count = 0
    if num < 10:
        x_av = 0
        y_av = 0
        dis = 0
    else:
        zxy_list_sorted = natsorted(zxy_list) 
        z_list = []
        for i in zxy_list_sorted:
            z_list.append(i[0])
        z_mean = np.mean(z_list)  
        z_var = np.var(z_list) 
        z_std = np.std(z_list) 

        for k in zxy_list_sorted:
            if abs(k[0] - z_mean) < (1 * z_std):
                x_av = x_av + k[1]
                y_av = y_av + k[2]
                dis = dis + k[0]
                count += 1
        if count != 0:
            x_av = x_av / count
            y_av = y_av / count
            dis = dis / count
        else:
            x_av = 0
            y_av = 0
            dis = 0

        return x_av, y_av, dis, num

    return x_av, y_av, dis, count
