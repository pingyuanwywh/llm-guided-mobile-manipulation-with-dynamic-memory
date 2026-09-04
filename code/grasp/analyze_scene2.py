#!/usr/bin/env python3
# encoding: utf-8
# 凸起检测 v2: RANSAC 拟合桌面平面 -> 到平面距离分割 -> 最大连通块=罐 -> 抓取点.
import numpy as np
from scipy import ndimage
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

D = np.load("scene_depth.npy").astype(float) / 1000.0
RGB = np.load("scene_rgb.npy")
K = np.load("scene_K.npy")
fkpos = np.load("scene_fkpos.npy")
fkq = np.load("scene_fkquat.npy")
H, W = D.shape
fx, fy, cx, cy = K[0], K[4], K[2], K[5]


def quat_to_mat(pos, q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w),   pos[0]],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w),   pos[1]],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y), pos[2]],
        [0, 0, 0, 1]], float)


hand2cam = np.array([[0, 0, 1, -0.101], [-1, 0, 0, 0.011],
                     [0, -1, 0, 0.045], [0, 0, 0, 1]], float)
T = quat_to_mat(fkpos, fkq) @ hand2cam

v, u = np.mgrid[0:H, 0:W]
X = (u - cx) * D / fx; Y = (v - cy) * D / fy
P = np.stack([X, Y, D, np.ones_like(D)], -1).reshape(-1, 4) @ T.T
bx, by, bz = P[:, 0], P[:, 1], P[:, 2]
valid = D.reshape(-1) > 0
work = valid & (bx > 0.12) & (bx < 0.50) & (np.abs(by) < 0.20) & (bz > -0.15) & (bz < 0.40)
pts = np.stack([bx, by, bz], -1)[work]
idx_work = np.where(work)[0]
print("in-workspace px:", pts.shape[0])

# ---- RANSAC 平面 ax+by+cz+d=0, 法线单位化 ----
rng = np.random.default_rng(0)
best_in, best_plane = 0, None
N = pts.shape[0]
for _ in range(120):
    s = pts[rng.integers(0, N, 3)]
    n = np.cross(s[1]-s[0], s[2]-s[0])
    nn = np.linalg.norm(n)
    if nn < 1e-6:
        continue
    n = n/nn; d = -n.dot(s[0])
    dist = np.abs(pts @ n + d)
    ninl = int((dist < 0.008).sum())
    if ninl > best_in:
        best_in, best_plane = ninl, (n, d)
n, d = best_plane
# 让法线朝上(+z)
if n[2] < 0:
    n, d = -n, -d
# 用内点最小二乘精修
inl = np.abs(pts @ n + d) < 0.01
c = pts[inl].mean(0)
uu, ss, vh = np.linalg.svd(pts[inl] - c, full_matrices=False)
n = vh[2]
if n[2] < 0:
    n = -n
d = -n.dot(c)
print("plane normal=%s  inliers=%d/%d  tilt_from_vertical=%.1fdeg"
      % (np.round(n, 3), int(inl.sum()), N, np.degrees(np.arccos(abs(n[2])))))

signed = pts @ n + d               # 到平面的有符号距离(+ = 朝上/朝相机)
above = signed > 0.03              # 桌面以上 3cm
# 映射回全图掩码
full_above = np.zeros(H*W, bool)
full_above[idx_work[above]] = True
maskimg = full_above.reshape(H, W)
# 最大连通块 = 罐
lab, num = ndimage.label(maskimg)
if num == 0:
    print("NO protrusion found"); raise SystemExit
counts = np.bincount(lab.ravel())
counts[0] = 0                      # 忽略背景
big = int(np.argmax(counts))
canmask = lab == big
print("components:", num, " biggest px:", int(counts[big]))

# 罐点(base系)
sel = np.zeros(H*W, bool); sel[idx_work] = True
canflat = canmask.reshape(-1)
cansel = canflat & valid
cpts = np.stack([bx, by, bz], -1)[cansel]
ox, oy, oz = cpts[:, 0], cpts[:, 1], cpts[:, 2]
plane_at = lambda x, y: (-d - n[0]*x - n[1]*y)/n[2]
height = np.median(oz) - plane_at(np.median(ox), np.median(oy))
print("CAN extent base: x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f" %
      (ox.min(), ox.max(), oy.min(), oy.max(), oz.min(), oz.max()))
# 抓取点: x/y 用罐质心(前后取中位, 左右取中位), 高度取罐中部
gx = float(np.median(ox)); gy = float(np.median(oy))
gz = float(plane_at(gx, gy) + max(0.03, height*0.5))
# 像素质心(供瞄准/可视化)
ys, xs = np.where(canmask)
print(">>> GRASP base: x=%.3f y=%.3f z=%.3f | can height=%.3f | px_center=(%d,%d)"
      % (gx, gy, gz, height, int(xs.mean()), int(ys.mean())))

fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
ax[0].imshow(RGB); ax[0].set_title("RGB"); ax[0].axis("off")
ov = np.zeros((H, W, 4)); ov[canmask] = (1, 0, 0, 0.55)
ax[1].imshow(RGB); ax[1].imshow(ov)
ax[1].plot(xs.mean(), ys.mean(), 'c+', ms=18, mew=3)
ax[1].set_title("red=detected can (largest protrusion)"); ax[1].axis("off")
plt.tight_layout(); plt.savefig("scene_analysis2.png", dpi=95)
print("saved scene_analysis2.png")
