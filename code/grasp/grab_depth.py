#!/usr/bin/env python3
# encoding: utf-8
# grab_depth.py —— 深度凸起识别抓取 (不看颜色).
#   观察姿势拍深度 -> RANSAC 拟合桌面 -> 桌面以上连通块=物体 -> 取最大块中心轴为抓取点
#   -> 复用 grab_can 的进程内 FK/IK + /board 舵机 + pick 序列.
#   感知稳定, 抓取点靠三维几何而非标签位置. 偏移 X_ADJ/Y_ADJ/Z_ADJ 现场微调.
# 用法: python3 grab_depth.py _dry_run:=true _z_adj:=0.0 _x_adj:=0.0
# 安全: dry_run 默认 True, 只检测/算IK/打印, 不下爪.
import sys, time
DIST_HW = "/home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/lib/python3/dist-packages"
DIST_CAR = "/home/uavg/ros_car/devel/lib/python3/dist-packages"
for _p in (DIST_HW, DIST_CAR):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import numpy as np
from scipy import ndimage
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool
import hiwonder_kinematics.transform as tf
from hiwonder_kinematics.forward_kinematics import ForwardKinematics
from hiwonder_kinematics.inverse_kinematics import get_ik, set_link, set_joint_range
from ros_robot_controller.srv import GetBusServoState, GetBusServoStateRequest
from ros_robot_controller.msg import GetBusServoCmd, SetBusServoState, BusServoState

OBSERVE_POSE = [(1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 200)]
HAND2CAM = np.array([[0, 0, 1, -0.101], [-1, 0, 0, 0.011],
                     [0, -1, 0, 0.045], [0, 0, 0, 1]], float)


def quat_to_mat(pos, q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w),   pos[0]],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w),   pos[1]],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y), pos[2]],
        [0, 0, 0, 1]], float)


class GrabDepth:
    def __init__(self):
        rospy.init_node("grab_depth", anonymous=True)
        self.dry_run = bool(rospy.get_param('~dry_run', True))
        self.x_adj = float(rospy.get_param('~x_adj', 0.0))
        self.y_adj = float(rospy.get_param('~y_adj', 0.0))
        self.z_adj = float(rospy.get_param('~z_adj', 0.0))
        self.above = float(rospy.get_param('~above', 0.03))   # 桌面以上阈值(m)
        self.radius = float(rospy.get_param('~radius', 0.027))  # 前沿->中心轴补偿(括号法甜点)
        self.grasp_h = float(rospy.get_param('~grasp_h', 0.10))  # 桌面平面以上抓取高度(m), 与位置无关
        self.depth_span = float(rospy.get_param('~depth_span', 0.09))  # 前沿往后保留进深, 砍拖尾
        self.nframes = int(rospy.get_param('~nframes', 5))
        self.aim = bool(rospy.get_param('~aim', True))            # 先转基座把瓶子转到画面中心再测量
        self.pix_gain = float(rospy.get_param('~pix_gain', 0.6))  # servo1 脉冲/像素

        set_link(tf.base_link, tf.link1, tf.link2, tf.link3, tf.tool_link)
        set_joint_range(tf.joint1, tf.joint2, tf.joint3, tf.joint4, tf.joint5, 'deg')
        self.fk = ForwardKinematics('')
        self.fk.set_link(tf.base_link, tf.link1, tf.link2, tf.link3, tf.tool_link)
        self.fk.set_joint_range(tf.joint1, tf.joint2, tf.joint3, tf.joint4, tf.joint5, 'deg')

        self.board = rospy.Publisher('/board/bus_servo/set_state', SetBusServoState, queue_size=1, latch=True)
        rospy.wait_for_service('/board/bus_servo/get_state', timeout=10)
        self.get_state = rospy.ServiceProxy('/board/bus_servo/get_state', GetBusServoState)
        t = time.time()
        while self.board.get_num_connections() == 0 and time.time()-t < 3:
            rospy.sleep(0.1)
        rospy.loginfo("grab_depth: dry_run=%s x_adj=%.3f y_adj=%.3f z_adj=%.3f above=%.3f",
                      self.dry_run, self.x_adj, self.y_adj, self.z_adj, self.above)

    # ---- 舵机 I/O (走 /board, 同 grab_can) ----
    @staticmethod
    def _st(sid, pulse):
        s = BusServoState()
        s.present_id = [1, int(sid)]; s.target_id = [0, 0]
        s.position = [1, int(round(pulse))]; s.offset = [0, 0]
        s.voltage = [0]; s.temperature = [0]; s.position_limit = [0, 0, 0]
        s.voltage_limit = [0, 0, 0]; s.max_temperature_limit = [0, 0]
        s.enable_torque = [0, 0]; s.save_offset = [0, 0]; s.stop = [0, 0]
        return s

    def set_servos(self, dur, pairs):
        m = SetBusServoState(); m.duration = float(dur)
        for sid, pos in pairs:
            m.state.append(self._st(sid, pos))
        self.board.publish(m)

    def endpoint(self):
        req = GetBusServoStateRequest()
        req.cmd = [GetBusServoCmd(id=s, get_position=1) for s in range(1, 6)]
        resp = self.get_state(req)
        pulses = [int(it.position[0]) for it in resp.state]
        ang = tf.pulse2angle(pulses)
        pos, quat = self.fk.get_fk(list(ang))
        return quat_to_mat(pos, quat)

    def solve_ik(self, position, pitch):
        sols = get_ik(list(position), float(pitch), [-180, 180], 1)
        if not sols:
            return None
        return [int(round(p)) for p in tf.angle2pulse([sols[0][0][0]])[0]]

    # ---- 深度凸起检测 -> base系抓取点 ----
    def detect(self, depth, K, T):
        H, W = depth.shape
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        Dm = depth.astype(float)/1000.0
        v, u = np.mgrid[0:H, 0:W]
        X = (u-cx)*Dm/fx; Y = (v-cy)*Dm/fy
        P = np.stack([X, Y, Dm, np.ones_like(Dm)], -1).reshape(-1, 4) @ T.T
        bx, by, bz = P[:, 0], P[:, 1], P[:, 2]
        valid = Dm.reshape(-1) > 0
        work = valid & (bx > 0.12) & (bx < 0.50) & (np.abs(by) < 0.20) & (bz > -0.15) & (bz < 0.40)
        idx = np.where(work)[0]
        pts = np.stack([bx, by, bz], -1)[work]
        if pts.shape[0] < 500:
            return None, "too few workspace points"
        # RANSAC 平面
        rng = np.random.default_rng(0)
        best_in, best = 0, None
        for _ in range(120):
            s = pts[rng.integers(0, pts.shape[0], 3)]
            nrm = np.cross(s[1]-s[0], s[2]-s[0]); L = np.linalg.norm(nrm)
            if L < 1e-6:
                continue
            nrm = nrm/L; dd = -nrm.dot(s[0])
            ninl = int((np.abs(pts @ nrm + dd) < 0.008).sum())
            if ninl > best_in:
                best_in, best = ninl, (nrm, dd)
        nrm, dd = best
        if nrm[2] < 0:
            nrm, dd = -nrm, -dd
        inl = np.abs(pts @ nrm + dd) < 0.01
        c = pts[inl].mean(0)
        _, _, vh = np.linalg.svd(pts[inl]-c, full_matrices=False)
        nrm = vh[2]
        if nrm[2] < 0:
            nrm = -nrm
        dd = -nrm.dot(c)
        tilt = np.degrees(np.arccos(abs(nrm[2])))
        # 桌面以上 -> 连通块
        above = (pts @ nrm + dd) > self.above
        fa = np.zeros(H*W, bool); fa[idx[above]] = True
        mask = fa.reshape(H, W)
        lab, num = ndimage.label(mask)
        if num == 0:
            return None, "no protrusion"
        cnt = np.bincount(lab.ravel()); cnt[0] = 0
        big = int(np.argmax(cnt))
        if cnt[big] < 150:
            return None, "protrusion too small (%d px)" % cnt[big]
        cm = (lab == big).reshape(-1) & valid
        cm_idx = np.where(cm)[0]
        ox, oy, oz = bx[cm], by[cm], bz[cm]
        # 砍掉后方拖尾(线缆/背景): 只留近前沿往后 depth_span 内的点
        front = float(np.percentile(ox, 5))
        keep = ox <= front + self.depth_span
        ox, oy, oz = ox[keep], oy[keep], oz[keep]
        if ox.size < 100:
            return None, "object too small after tail-cut (%d)" % ox.size
        kept_idx = cm_idx[keep]
        px_cx = float((kept_idx % W).mean())   # 罐子像素中心(供对准)
        px_cy = float((kept_idx // W).mean())
        meas_r = 0.5 * (float(np.percentile(oy, 95)) - float(np.percentile(oy, 5)))  # 实测半径(仅供参考)
        gx = front + self.radius          # 前沿 + 半径(括号法甜点值) = 中心轴
        gy = float(np.median(oy))         # 左右中心
        z_plane = float((-dd - nrm[0]*gx - nrm[1]*gy)/nrm[2])  # 桌面在(gx,gy)处高度
        gz = z_plane + self.grasp_h       # 锚定桌面以上固定高度(与可见罐身多少无关)
        info = dict(tilt=tilt, inl=int(inl.sum()), N=pts.shape[0], px=int(ox.size), front=front,
                    meas_r=meas_r, zplane=z_plane, oz_med=float(np.median(oz)), px_cx=px_cx, px_cy=px_cy,
                    xr=(ox.min(), ox.max()), yr=(oy.min(), oy.max()), zr=(oz.min(), oz.max()))
        return (gx, gy, gz), info

    def capture_depth(self, n):
        # 取 n 帧逐像素中位数, 填抖动空洞
        buf = []
        for _ in range(n):
            m = rospy.wait_for_message('/depth_cam/depth/image_raw', Image, timeout=5)
            buf.append(np.ndarray((m.height, m.width), np.uint16, m.data).copy())
            rospy.sleep(0.05)
        stack = np.stack(buf).astype(float)
        stack[stack == 0] = np.nan
        med = np.nanmedian(stack, axis=0)
        med[np.isnan(med)] = 0
        return med.astype(np.uint16)

    # ---- 走一个笛卡尔路点(IK) ----
    def _go(self, pos, pitch, dur, grip=None):
        p = self.solve_ik(pos, pitch)
        if not p:
            rospy.logwarn("路点无解 %s pitch=%d -> 跳过", [round(v, 3) for v in pos], pitch)
            return False
        cmd = [(i+1, p[i]) for i in range(5)]
        if grip is not None:
            cmd.append((10, int(grip)))
        self.set_servos(dur, cmd); rospy.sleep(dur+0.3)
        return True

    # ---- 抓取 + 安全放置(全程高位转身, 避开车体) ----
    def pick(self, position, pitch):
        gx, gy, gz = position[0], position[1], position[2]
        pulses = self.solve_ik([gx, gy, gz], pitch)
        if pulses:
            self.set_servos(1.0, [(1, pulses[0])]); rospy.sleep(1.0)                       # 先转基座
            self.set_servos(1.5, [(i+1, pulses[i]) for i in range(5)]); rospy.sleep(1.5)   # 下探到抓取点
        self.set_servos(0.6, [(10, 600)]); rospy.sleep(1.2)                                # 合爪(慢, 夹稳)
        # 安全放置路径(全程 z>=0.24 高位转身, servo1 平滑 ->右侧; 放慢减少甩动)
        self._go([gx, gy, gz+0.10], pitch, 1.6, grip=600)    # W1 原地直上抬起
        self._go([0.24,  0.00, 0.26], 50, 2.0, grip=600)     # W2 高位前伸-居中
        self._go([0.24, -0.18, 0.24], 50, 2.4, grip=600)     # W3 高位前伸-转右(最慢, 避甩)
        self._go([0.26, -0.20, 0.12], 30, 1.8, grip=600)     # W4 右侧下放
        self.set_servos(0.5, [(10, 200)]); rospy.sleep(0.8)  # 松爪放下
        self._go([0.24, -0.18, 0.24], 50, 1.4)               # 空爪高位收回
        self._go([0.24,  0.00, 0.26], 50, 1.4)               # 高位居中
        self.set_servos(1.4, OBSERVE_POSE); rospy.sleep(2.0)  # 回观察

    def run(self):
        # 掐激光
        try:
            rospy.wait_for_service('/depth_cam/set_ldp', timeout=10)
            rospy.ServiceProxy('/depth_cam/set_ldp', SetBool)(False)
        except Exception as e:
            rospy.logwarn("set_ldp: %s", e)
        # 观察姿势
        self.set_servos(1.5, OBSERVE_POSE); rospy.sleep(2.5)
        info_msg = rospy.wait_for_message('/depth_cam/depth/camera_info', CameraInfo, timeout=5)
        K = list(info_msg.K)
        # ---- 先转基座把瓶子转到画面水平中心再测量(中心畸变最小, 且像素对准不依赖标定) ----
        cx_img = K[2]
        servo1 = 500
        res = info = None
        for it in range(5):
            T = self.endpoint() @ HAND2CAM
            depth = self.capture_depth(self.nframes)
            res, info = self.detect(depth, K, T)
            if res is None:
                rospy.logerr("检测失败: %s", info); return
            px_err = info['px_cx'] - cx_img          # +=瓶子在画面右, -=在左
            rospy.loginfo("对准 it=%d: 像素x=%.0f 误差=%.0fpx y=%.3f servo1=%d",
                          it, info['px_cx'], px_err, res[1], servo1)
            if (not self.aim) or abs(px_err) < 18:
                break
            servo1 = int(min(max(servo1 - px_err * self.pix_gain, 200), 800))
            self.set_servos(0.7, [(1, servo1)]); rospy.sleep(1.0)
        gx, gy, gz = res
        rospy.loginfo("检测: tilt=%.1fdeg 块=%dpx front=%.3f meas_r=%.3f zplane=%.3f  x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]",
                      info['tilt'], info['px'], info['front'], info['meas_r'], info['zplane'],
                      info['xr'][0], info['xr'][1], info['yr'][0], info['yr'][1], info['zr'][0], info['zr'][1])
        gx += self.x_adj; gy += self.y_adj; gz += self.z_adj
        pitch = 80 if gz < 0.2 else 30
        pulses = self.solve_ik([gx, gy, gz], pitch)
        rospy.loginfo(">>> 抓取点(含微调) base=[%.3f,%.3f,%.3f] pitch=%d IK=%s",
                      gx, gy, gz, pitch, ("OK "+str(pulses)) if pulses else "无解")
        if self.dry_run:
            rospy.loginfo("DRY-RUN: 不下爪."); return
        if not pulses:
            rospy.logerr("IK无解, 不动."); return
        rospy.loginfo("执行抓取...")
        self.pick([gx, gy, gz], pitch)
        rospy.loginfo("抓取序列完成.")


if __name__ == "__main__":
    GrabDepth().run()
