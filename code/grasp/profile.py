#!/usr/bin/env python3
# 不设阈值, 直接把帧差曲线按秒打出来 + 画条形图, 转折点(车起步)肉眼一看就有。
import sys, glob
import numpy as np
from PIL import Image

d, fps, t0 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
files = sorted(glob.glob(d + '/*.png'))
arr = [np.asarray(Image.open(f), dtype=np.int16) for f in files]
diffs = np.array([float(np.abs(arr[i] - arr[i - 1]).mean()) for i in range(1, len(arr))])

n = int(round(fps))
lo, hi = diffs.min(), diffs.max()
print('%d 帧, 帧差 %.2f ~ %.2f' % (len(files), lo, hi))
print('%-9s %-7s %s' % ('时刻(s)', '均值', '条形图(按整段最大值归一)'))
for k in range(0, len(diffs), n):
    chunk = diffs[k:k + n]
    t = t0 + k / fps
    m = chunk.mean()
    bar = '#' * int(round(40 * (m - lo) / (hi - lo + 1e-9)))
    print('%8.2f  %6.2f  %s' % (t, m, bar))

# 最细粒度: 帧差最大的 8 帧在哪
idx = np.argsort(diffs)[-8:][::-1]
print('\n帧差最大的 8 帧:')
for i in sorted(idx):
    print('  t=%8.3f s   diff=%.2f' % (t0 + i / fps, diffs[i]))
