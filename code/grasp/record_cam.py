#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""录小车相机视角(RGB + 深度伪彩)成视频, 给 demo 视频用。

默认出 **H.264 的标准 mp4**(手机能播、视频平台能传), 靠车上 OpenCV 链接的 libavcodec ——
车上没有 ffmpeg **命令行**, 但 OpenCV 编进了 FFMPEG 后端, 所以不用装东西。
`--codec mjpg` 可退回逐帧 JPEG 的 avi: 大 13 倍, 但抗截断(被硬杀/断电也能救回已写部分)。

抓取节点在 /track_and_grab/vis_info 上发的检测框/中心点/距离会画到**这两路**上
(它们和抓取用的是同一份检测结果, 不是事后重算)。

固定帧率写盘(定时器取"最新一帧"), 所以 视频时长 == 墙上时钟时长, 和手机/天花板机位
对齐时不用管丢帧。每帧的墙上时间和 ROS 时间戳另存 CSV, 要和文字日志对时也够用。

用法(车上):
    python3 ~/record_cam.py --tag can5              # 录到 ~/demo_videos/, Ctrl-C 停
    python3 ~/record_cam.py --tag can5 --fps 15 --hud
    python3 ~/record_cam.py --duration 20 --tag test # 定时自停(空跑验证用)
停止: Ctrl-C 或 kill <pid>(SIGTERM 也会正常收尾)
"""

import argparse
import json
import os
import signal
import sys
import threading
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool

RGB_TOPIC = "/depth_cam/rgb/image_raw"
DEPTH_TOPIC = "/depth_cam/depth/image_raw"
# 抓取节点发的标注数据(检测框/中心点/距离, 几十字节的 JSON), 画到 RGB 和深度两路上。
# 抓取节点没起(建图/纯导航阶段)就没人发, 自动不画。
VIS_TOPIC = "/track_and_grab/vis_info"
# 标注多久没更新就不再画 —— 免得导航段还挂着一个早就过期的框
VIS_STALE_S = 0.5
# 目标颜色 -> BGR
COLOR_BGR = {"red": (60, 60, 255), "green": (60, 230, 60), "blue": (255, 140, 60)}


def ldp_off():
    """LDP(激光近距保护)开着时深度整帧是 0, 录出来是全黑。抓取节点起来时会关它,
    但录制可能先起, 所以自己也关一次(幂等)。"""
    try:
        rospy.wait_for_service("/depth_cam/set_ldp", timeout=3.0)
        resp = rospy.ServiceProxy("/depth_cam/set_ldp", SetBool)(False)
        rospy.loginfo("set_ldp(False) -> %s", resp.success)
    except Exception as exc:
        rospy.logwarn("关 LDP 失败(%s) —— 深度可能整帧为 0", exc)


def open_writer(base, stream, size, fps, codec):
    """h264: 直接出标准 mp4(手机/视频平台通吃), 靠车上 OpenCV 链接的 libavcodec, 不需要 ffmpeg 命令行。
       mjpg: 逐帧 JPEG 的 avi, 大 13 倍, 但**抗截断**(进程被硬杀/断电也能救回已写部分)。"""
    if codec == "h264":
        path = "%s_%s.mp4" % (base, stream)
        w = cv2.VideoWriter(path, cv2.CAP_FFMPEG, cv2.VideoWriter_fourcc(*"avc1"), fps, size)
    else:
        path = "%s_%s.avi" % (base, stream)
        w = cv2.VideoWriter(path, cv2.CAP_OPENCV_MJPEG, cv2.VideoWriter_fourcc(*"MJPG"), fps, size)
    if not w.isOpened():
        raise SystemExit("[FATAL] 打不开视频写入器: %s (codec=%s)" % (path, codec))
    return w, path


def rgb_msg_to_bgr(msg):
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    if msg.encoding in ("rgb8", "RGB8"):
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    return arr.copy()


def depth_msg_to_mm(msg):
    return np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))


def colorize_depth(depth_mm, near_mm, far_mm, cmap):
    """毫米深度 -> 伪彩 BGR。无效点(0 / 超界)画成黑色, 一眼能看出深度空洞。"""
    valid = (depth_mm > 0) & (depth_mm >= near_mm) & (depth_mm <= far_mm)
    span = max(1, far_mm - near_mm)
    norm = np.clip((depth_mm.astype(np.float32) - near_mm) * (255.0 / span), 0, 255)
    color = cv2.applyColorMap(norm.astype(np.uint8), cmap)
    color[~valid] = (0, 0, 0)
    return color


def draw_hud(img, text):
    cv2.putText(img, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)


def draw_annotation(img, info):
    """把抓取节点报的检测框/中心点/距离画上去。RGB 和深度是同一台相机、同分辨率,
    像素坐标通用, 所以两路画的是同一个框。"""
    box = info.get("box")
    center = info.get("c")
    color = COLOR_BGR.get(info.get("color"), (200, 200, 200))
    if box:
        x, y, w, h = [int(v) for v in box]
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 0), 4)      # 黑边, 保证在任何底色上都看得见
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
    if center:
        cx, cy = int(center[0]), int(center[1])
        cv2.circle(img, (cx, cy), 6, (0, 0, 0), -1)
        cv2.circle(img, (cx, cy), 4, (255, 255, 255), -1)
    txt = str(info.get("color", ""))
    if info.get("dist_mm"):
        txt += "  d=%.3fm" % (info["dist_mm"] / 1000.0)
    h_img = img.shape[0]
    cv2.putText(img, txt, (10, h_img - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, txt, (10, h_img - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)


class Recorder(object):
    def __init__(self, args):
        self.args = args
        self.rgb = None
        self.depth = None
        self.vis_info = None      # 最近一条标注(dict)
        self.vis_info_t = 0.0     # 收到它的墙上时刻, 过期就不画
        self.rgb_stamp = 0.0
        self.depth_stamp = 0.0
        self.rgb_msgs = 0
        self.depth_msgs = 0
        self.vis_msgs = 0
        self.annotated = 0        # 画上了标注的帧数
        self.frames = 0
        self.t0 = None
        self.closed = False
        self.lock = threading.Lock()  # 定时器线程写盘 vs 主线程收尾, 不加锁会 write on closed file

        os.makedirs(args.out_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = "%s_%s" % (stamp, args.tag) if args.tag else stamp
        self.base = os.path.join(args.out_dir, base)
        self.cmap = getattr(cv2, "COLORMAP_" + args.colormap.upper(), cv2.COLORMAP_JET)

        if not args.keep_ldp:
            ldp_off()

        # 先拿一帧确定分辨率, 拿不到就别开录(比录出个空文件强)
        rospy.loginfo("等第一帧 %s / %s ...", args.rgb_topic, args.depth_topic)
        m_rgb = rospy.wait_for_message(args.rgb_topic, Image, timeout=10.0)
        m_depth = rospy.wait_for_message(args.depth_topic, Image, timeout=10.0)
        self.size = (m_rgb.width, m_rgb.height)
        self.depth_size = (m_depth.width, m_depth.height)
        valid = 100.0 * (depth_msg_to_mm(m_depth) > 0).mean()
        print("[REC] 首帧深度有效率 %.1f%%" % valid, flush=True)
        if valid < 5.0:
            print("[WARN] 深度几乎全无效 -> 深度视频会是全黑。检查 LDP / 相机是否贴太近", flush=True)

        self.w_rgb, self.rgb_path = open_writer(self.base, "rgb", self.size, args.fps, args.codec)
        self.w_depth, self.depth_path = open_writer(self.base, "depth", self.depth_size, args.fps, args.codec)
        self.csv = open(self.base + "_frames.csv", "w")
        self.csv.write("frame,wall_epoch,elapsed_s,rgb_stamp,depth_stamp\n")

        # 标注数据。抓取节点只在有订阅者时才发, 订上去它就开始发了;
        # 节点没起(建图/纯导航阶段)也不影响录制, 只是没框可画。
        if not args.no_vis:
            rospy.Subscriber(args.vis_topic, String, self.on_vis, queue_size=2)

        rospy.Subscriber(args.rgb_topic, Image, self.on_rgb, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(args.depth_topic, Image, self.on_depth, queue_size=1, buff_size=2 ** 24)
        rospy.loginfo("录制中 -> %s (%dx%d @ %d fps, %s)",
                      self.base, self.size[0], self.size[1], args.fps, args.codec)
        print("[REC] %s" % self.rgb_path, flush=True)
        print("[REC] %s" % self.depth_path, flush=True)

    def on_rgb(self, msg):
        self.rgb = rgb_msg_to_bgr(msg)
        self.rgb_stamp = msg.header.stamp.to_sec()
        self.rgb_msgs += 1

    def on_depth(self, msg):
        self.depth = depth_msg_to_mm(msg)
        self.depth_stamp = msg.header.stamp.to_sec()
        self.depth_msgs += 1

    def on_vis(self, msg):
        try:
            info = json.loads(msg.data)
        except ValueError:
            return
        self.vis_info = info if info.get("ok") else None
        self.vis_info_t = time.time()
        self.vis_msgs += 1

    def tick(self, _evt=None):
        with self.lock:
            self._tick_locked()

    def _tick_locked(self):
        if self.closed or self.rgb is None or self.depth is None:
            return
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        elapsed = now - self.t0

        rgb = self.rgb.copy()
        depth = colorize_depth(self.depth, self.args.near, self.args.far, self.cmap)

        # 检测框/中心点/距离: 两路画同一个(同相机同分辨率, 像素坐标通用)
        info = self.vis_info
        if info is not None and (now - self.vis_info_t) < VIS_STALE_S:
            draw_annotation(rgb, info)
            draw_annotation(depth, info)
            self.annotated += 1

        hud = "t=%06.2fs  f=%05d" % (elapsed, self.frames)
        if self.args.hud:
            draw_hud(rgb, hud)
            draw_hud(depth, hud + "  %.1f-%.1fm" % (self.args.near / 1000.0, self.args.far / 1000.0))

        self.w_rgb.write(rgb)
        self.w_depth.write(depth)
        self.csv.write("%d,%.3f,%.3f,%.3f,%.3f\n"
                       % (self.frames, now, elapsed, self.rgb_stamp, self.depth_stamp))
        self.frames += 1
        if self.frames % (self.args.fps * 30) == 0:
            self.csv.flush()
            print("[REC] %.0fs  %d frames  rgb_msgs=%d depth_msgs=%d"
                  % (elapsed, self.frames, self.rgb_msgs, self.depth_msgs), flush=True)

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self._close_locked()

    def _close_locked(self):
        try:
            self.w_rgb.release()
            self.w_depth.release()
            self.csv.close()
        except Exception as exc:  # 收尾别抛, 文件能落多少是多少
            print("[WARN] 收尾异常: %s" % exc, flush=True)
        dur = 0.0 if self.t0 is None else (time.time() - self.t0)
        meta = {
            "base": self.base,
            "codec": self.args.codec,
            "rgb": self.rgb_path,
            "depth": self.depth_path,
            "frames_csv": self.base + "_frames.csv",
            "fps": self.args.fps,
            "size": list(self.size),
            "depth_size": list(self.depth_size),
            "frames": self.frames,
            "wall_start_epoch": self.t0,
            "wall_start_iso": None if self.t0 is None else time.strftime(
                "%Y-%m-%d %H:%M:%S", time.localtime(self.t0)),
            "wall_duration_s": round(dur, 3),
            "rgb_msgs": self.rgb_msgs,
            "depth_msgs": self.depth_msgs,
            "vis_msgs": self.vis_msgs,
            "annotated_frames": self.annotated,
            "rgb_topic": self.args.rgb_topic,
            "depth_topic": self.args.depth_topic,
            "depth_near_mm": self.args.near,
            "depth_far_mm": self.args.far,
            "colormap": self.args.colormap,
            "hud": self.args.hud,
        }
        with open(self.base + "_meta.json", "w") as fh:
            json.dump(meta, fh, indent=2)
        print("[DONE] %d 帧 / %.1fs, 其中 %d 帧画了标注" % (self.frames, dur, self.annotated),
              flush=True)
        for path in (self.rgb_path, self.depth_path):
            if os.path.exists(path):
                print("       %s  %.1f MB" % (path, os.path.getsize(path) / 1e6), flush=True)


def main():
    p = argparse.ArgumentParser(description="录 JetRover 相机 RGB + 深度视角")
    p.add_argument("--out-dir", default=os.path.expanduser("~/demo_videos"))
    p.add_argument("--tag", default="", help="文件名后缀, 例如 run5can")
    p.add_argument("--fps", type=int, default=15, help="写盘帧率(相机 30Hz, 15 够顺且省一半盘)")
    p.add_argument("--duration", type=float, default=0.0, help=">0 则录满这么多秒自停")
    # 2026-07-29 实测: 臂在观察姿势时相机朝前下方看, 整帧深度只有 0.35~0.64m
    # (p5=350 p50=438 p95=600 max=639), 原来的 0.25~2.5m 把 90% 的色阶浪费在没有像素的区间,
    # 录出来整片蓝紫。0.30~0.90 对比度好且给"看远一点"留了余量。
    p.add_argument("--near", type=int, default=300, help="深度伪彩近端(mm)")
    p.add_argument("--far", type=int, default=900, help="深度伪彩远端(mm)")
    p.add_argument("--colormap", default="TURBO", help="TURBO / JET / VIRIDIS ...")
    p.add_argument("--hud", action="store_true", help="左上角画时间戳(对时方便, 但没那么好看)")
    p.add_argument("--keep-ldp", action="store_true", help="别动 LDP(默认会关, 不关深度整帧为 0)")
    p.add_argument("--rgb-topic", default=RGB_TOPIC)
    p.add_argument("--depth-topic", default=DEPTH_TOPIC)
    p.add_argument("--vis-topic", default=VIS_TOPIC, help="抓取节点发的标注数据")
    p.add_argument("--no-vis", action="store_true", help="不画检测框/中心点")
    p.add_argument("--codec", default="h264", choices=("h264", "mjpg"),
                   help="h264=标准 mp4(小 13 倍, 手机/平台直接播) / mjpg=avi(大但抗截断)")
    args = p.parse_args(rospy.myargv()[1:])

    rospy.init_node("record_cam", anonymous=True, disable_signals=True)
    rec = Recorder(args)

    def bye(*_):
        rec.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, bye)
    signal.signal(signal.SIGTERM, bye)

    period = rospy.Duration(1.0 / args.fps)
    rospy.Timer(period, rec.tick)
    if args.duration > 0:
        time.sleep(args.duration)
        bye()
    else:
        rospy.spin()
        rec.close()


if __name__ == "__main__":
    main()
