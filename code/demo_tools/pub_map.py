#!/usr/bin/env python3
"""
pub_map.py -- 【本机】把 map_saver 存的 pgm+yaml 当 /map 发出去 (latched)。
本机没装 map_server, 而离线重渲染上帝视角必须有底图(bag 里为省盘没录 /map)。
yaml 里的 image 路径是车上的绝对路径, 这里按"和 yaml 同目录的同名 pgm"找。

用法: /usr/bin/python3 ~/pub_map.py ~/map_0808c.yaml
"""
import os
import sys

import numpy as np
import rospy
import yaml
from nav_msgs.msg import OccupancyGrid


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5', 'only binary PGM'
        ln = f.readline()
        while ln.startswith(b'#'):
            ln = f.readline()
        w, h = map(int, ln.split())
        maxv = int(f.readline())
        data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    return w, h, data


def main():
    ypath = os.path.expanduser(sys.argv[1])
    m = yaml.safe_load(open(ypath))
    pgm = os.path.join(os.path.dirname(ypath),
                       os.path.basename(m['image']))
    w, h, img = read_pgm(pgm)
    res = m['resolution']
    ox, oy, _ = m['origin']
    occ_t, free_t = m.get('occupied_thresh', 0.65), m.get('free_thresh', 0.196)

    # pgm: 255=free 0=occupied; ROS: 0=free 100=occupied -1=unknown
    p = (255.0 - img.astype(np.float32)) / 255.0      # 占据概率
    grid = np.full(p.shape, -1, dtype=np.int8)
    grid[p > occ_t] = 100
    grid[p < free_t] = 0
    grid = np.flipud(grid)                             # pgm 第一行是最上面

    g = OccupancyGrid()
    g.header.frame_id = 'map'
    g.info.resolution = res
    g.info.width, g.info.height = w, h
    g.info.origin.position.x, g.info.origin.position.y = ox, oy
    g.info.origin.orientation.w = 1.0
    g.data = grid.reshape(-1).tolist()

    rospy.init_node('pub_map', anonymous=True)
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    rospy.loginfo('地图 %dx%d res=%.3f origin=(%.2f,%.2f) 已发布(latched)', w, h, res, ox, oy)
    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        g.header.stamp = rospy.Time.now()
        pub.publish(g)
        r.sleep()


if __name__ == '__main__':
    main()
