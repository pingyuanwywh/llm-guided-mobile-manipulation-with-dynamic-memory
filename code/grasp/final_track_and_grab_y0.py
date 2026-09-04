#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/12/19
# 机械前超前看识别追踪空中指定颜色物品
# 通过深度相机识别计算物品的空间位置
# 完成抓取并放到指定位置
import cv2
import json
import math
import time
import rospy
import queue
import signal
import threading
import numpy as np
import message_filters
from std_srvs.srv import SetBool
from std_msgs.msg import String
from hiwonder_sdk import pid, common, fps
from sensor_msgs.msg import Image, CameraInfo
from hiwonder_interfaces.srv import SetString
from hiwonder_interfaces.srv import GetRobotPose
from ros_robot_controller.msg import BusServoState, SetBusServoState, GetBusServoCmd
from ros_robot_controller.srv import GetBusServoState, GetBusServoStateRequest
from std_srvs.srv import Trigger, TriggerResponse
from hiwonder_kinematics import kinematics_control


# ROI(占画面宽/高的比例)。注意 proc() 里是在半分辨率图上比较, 所以代码里还要再 /2。
# 这四个数决定"哪些轮廓算数": 轮廓的最小外接圆圆心不在 ROI 内, 整条丢弃(不是裁剪, 是丢弃)。
#
# ROI_Y_TOP —— **决定检测距离上限**。罐子越远在画面里越靠上, 轮廓中心越过上边界就被丢掉。
#   2026-07-23 实测: 罐子在 0.42m 时轮廓中心 y=99(全分辨率), 旧上边界 0.25 对应 y=90, 只剩 9 像素余量。
#   同一帧的色块面积是 620, 而面积门限只有 10(62倍余量) ⇒ 限制距离的是这条边界, 不是面积。
#   0.25 -> 0.10 就是为了把 standoff 能录的距离拉远。
# ROI_Y_BOTTOM —— **千万别动**。它拦的是车自己的绿色底盘: 实测底盘轮廓 area=1051、
#   中心在 y=355(画面高 360)处, 全靠这条边界丢掉。放宽下边界 = 机械臂去抓自己的底盘。
ROI_X_LEFT, ROI_X_RIGHT = 0.35, 0.72
ROI_Y_TOP, ROI_Y_BOTTOM = 0.10, 0.78


def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])

def make_bus_servo_state(servo_id, position):
    state = BusServoState()
    state.present_id = [1, int(servo_id)]
    state.target_id = [0, 0]
    state.position = [1, int(position)]
    state.offset = [0, 0]
    state.voltage = [0, 0]
    state.temperature = [0, 0]
    state.position_limit = [0, 0, 0]
    state.voltage_limit = [0, 0, 0]
    state.max_temperature_limit = [0, 0]
    state.enable_torque = [0, 0]
    state.save_offset = [0]
    state.stop = [0]
    return state

def set_servos(pub, duration, pos_s):
    msg = SetBusServoState()
    msg.duration = float(duration)
    msg.state = [make_bus_servo_state(servo_id, position) for servo_id, position in pos_s]
    pub.publish(msg)

class ColorTracker:
    def __init__(self, target_color):
        self.target_color = target_color
        self.pid_yaw = pid.PID(20.5, 1.0, 1.2)
        self.pid_pitch = pid.PID(20.5, 1.0, 1.2)
        self.yaw = 500
        self.pitch = 150
        self.last_box = None   # 最近一帧的外接框(全分辨率 x,y,w,h), 只给录像标注用
    
    def proc(self, source_image, result_image, color_ranges):
        h, w = source_image.shape[:2]
        color = color_ranges['lab']['Stereo'][self.target_color]

        img = cv2.resize(source_image, (int(w/2), int(h/2)))
        img_blur = cv2.GaussianBlur(img, (3, 3), 3) # 高斯模糊
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间
        mask = cv2.inRange(img_lab, tuple(color['min']), tuple(color['max'])) # 二值化

        # 平滑边缘，去除小块，合并靠近的块。魔方红面被黑色间隙分成九块，
        # 所以这里把工作区内的红色小轮廓合并后再取整体中心。
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        candidate_contours = []
        roi_x_min, roi_x_max = int(w * ROI_X_LEFT / 2), int(w * ROI_X_RIGHT / 2)
        roi_y_min, roi_y_max = int(h * ROI_Y_TOP / 2), int(h * ROI_Y_BOTTOM / 2)
        for c in contours:
            area = math.fabs(cv2.contourArea(c))
            if area < 10:
                continue
            (center_x, center_y), radius = cv2.minEnclosingCircle(c) # 最小外接圆
            if roi_x_min <= center_x <= roi_x_max and roi_y_min <= center_y <= roi_y_max:
                candidate_contours.append(c)

        # 如果有符合要求的轮廓
        if candidate_contours:
            all_points = np.vstack(candidate_contours)
            x, y, box_w, box_h = cv2.boundingRect(all_points)
            center_x = x + box_w / 2
            center_y = y + box_h / 2
            radius = max(box_w, box_h) / 2

            # 圈出识别的的要追踪的色块
            circle_color = common.range_rgb[self.target_color] if self.target_color in common.range_rgb else (0x55, 0x55, 0x55)
            # 2026-07-29 加: 把外接框存下来(全分辨率), 给录 demo 视频的标注用。
            # boundingRect 是在半分辨率图上算的, 所以这里 ×2 才是原图坐标。
            self.last_box = (int(x * 2), int(y * 2), int(box_w * 2), int(box_h * 2))
            cv2.rectangle(result_image, (int(x * 2), int(y * 2)), (int((x + box_w) * 2), int((y + box_h) * 2)), circle_color, 2)
            cv2.circle(result_image, (int(center_x * 2), int(center_y * 2)), 5, circle_color, -1)

            center_x = center_x * 2
            center_x_1 = center_x / w
            if abs(center_x_1 - 0.5) > 0.02: # 相差范围小于一定值就不用再动了
                self.pid_yaw.SetPoint = 0.5 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置
                self.pid_yaw.update(center_x_1)
                self.yaw = min(max(self.yaw + self.pid_yaw.output, 0), 1000)
            else:
                self.pid_yaw.clear() # 如果已经到达中心了就复位一下 pid 控制器

            center_y = center_y * 2
            center_y_1 = center_y / h
            if abs(center_y_1 - 0.5) > 0.02:
                self.pid_pitch.SetPoint = 0.5
                self.pid_pitch.update(center_y_1)
                self.pitch = min(max(self.pitch + self.pid_pitch.output, 100), 720)
            else:
                self.pid_pitch.clear()
            # rospy.loginfo("x:{:.2f}\ty:{:.2f}".format(self.x , self.y))
            return (result_image, (self.pitch, self.yaw), (center_x, center_y), radius * 2)
        else:
            self.last_box = None
            return (result_image, None, None, 0)


class TrackAndGrapNode:
    hand2cam_tf_matrix = [
    [0.0, 0.0, 1.0, -0.101],
    [-1.0, 0.0, 0.0, 0.011],
    [0.0, -1.0, 0.0, 0.045],
    [0.0, 0.0, 0.0, 1.0]
]

    def __init__(self, name):
        rospy.init_node(name, anonymous=True, log_level=rospy.INFO)
        self.fps = fps.FPS()
        self.moving = False
        self.count = 0
        self.start = False
        self.running = True
        self.last_pitch_yaw = (0, 0)
        self.last_center = None
        self.last_debug = 0
        self.last_grasp_target = None
        self.low_grasp_pitch = rospy.get_param('~low_grasp_pitch', 10)
        self.high_grasp_pitch = rospy.get_param('~high_grasp_pitch', 30)
        self.last_grasp_pitch = self.low_grasp_pitch
        self.first_time = time.time()

        self.enable_disp = 1
        signal.signal(signal.SIGINT, self.shutdown)
        self.lab_config_path = rospy.get_param(
            '~lab_config_path',
            '/home/uavg/JetRover-Jetson_nano_ros1/ros_ws/src/hiwonder_example/scripts/rgbd_function/lab_config_can.yaml')
        self.lab_data = common.get_yaml_data(self.lab_config_path)
        self.last_position = (0, 0, 0)
        self.stamp = time.time()
        self.servos_pub = rospy.Publisher('/board/bus_servo/set_state', SetBusServoState, queue_size=1)
        # 2026-07-29 加: 给录 demo 视频用的**标注数据**(不是图)。
        # 只发 检测框/中心点/距离 这几十字节, 由 record_cam.py 画到它自己录的 RGB 和深度两路上。
        # 为什么不发合成图: 那张图 = RGB+深度横着拼一遍, 和另外两路重复, 白白多一倍码率;
        # 而且它的深度用的是 JET 0~2m 固定配色, 比录制脚本自己调的那套难看。
        # 用绝对名而不是 '~vis_info': 本节点 anonymous=True, 私有名会带随机后缀。
        # 只在有订阅者时才发 ⇒ 没人录的时候零开销, 不影响抓取。
        self.vis_pub = rospy.Publisher('/track_and_grab/vis_info', String, queue_size=1)
        self.vis_period = 1.0 / max(1.0, float(rospy.get_param('~vis_fps', 15.0)))
        self.last_vis = 0.0
        # 读真实舵机位置(走 board_node, 不抢串口)。servo_manager 的 servo_states 是假数据, 别用。
        self.servo_get = rospy.ServiceProxy('/board/bus_servo/get_state', GetBusServoState)
        # 夹爪位置 0=全开 420=全合。实测: 张开~63 / 夹住易拉罐~250 / 空夹~418。
        # "夹住"是区间判据 [grip_open_max, grip_empty_thresh], 单边阈值会把"张开"误判成"夹住"。
        self.grip_open_max = rospy.get_param('~grip_open_max', 150)
        self.grip_empty_thresh = rospy.get_param('~grip_empty_thresh', 350)
        self.grab_timeout = rospy.get_param('~grab_timeout', 25.0)
        self.place_timeout = rospy.get_param('~place_timeout', 40.0)
        self.grab_done = threading.Event()
        self.grab_result = (False, 'no result')
        self.grab_deadline = None
        self.last_depth_fail = 0.0
        self.last_depth_fail_px = None
        self.place_done = threading.Event()
        self.place_result = (False, 'no result')
        # measure: 只测量不动作, 供逼近环闭环用。走的是和 grab 完全相同的检测管线,
        # 所以逼近环收敛到的坐标就是 pick 会用的坐标, 不会两套检测打架。
        # 抓取高度(臂基座系, 米)。>=0 表示用这个常数, 负数表示退回"跟随检测 z"的旧行为。
        # 0.12 取自 2026-07-23 四次成功抓取的实测高度 0.106/0.107/0.136/0.141。
        self.grasp_height = rospy.get_param('~grasp_height', 0.12)
        self.measure_timeout = rospy.get_param('~measure_timeout', 8.0)
        self.measure_only = False
        self.measure_done = threading.Event()
        self.measure_result = None
        rospy.sleep(0.2)
        set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 0)))
        rospy.sleep(1)
        self.target_color = rospy.get_param('~color', 'green')
        rospy.wait_for_service('/depth_cam/set_ldp')
       
        rospy.Service('~start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法
        rospy.Service('~grab', Trigger, self.grab_srv_callback)
        rospy.Service('~place', Trigger, self.place_srv_callback)
        rospy.Service('~measure', Trigger, self.measure_srv_callback)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)
        # tracker 总是建好(否则 measure/grab 都用不了, 还得先手动调 set_color——
        # 而调 set_color 正是上面那个自动抓取事故的触发点)。
        # 是否"待命抓取"单独由 ~start 决定, 默认沿用原来的 True。
        self.tracker = None
        msg = SetString()
        msg.data = self.target_color
        self.set_color_srv_callback(msg)
        self.start = bool(rospy.get_param('~start', True))

        self.image_queue = queue.Queue(maxsize=2)
        self.endpoint = None

        self.ttt = time.time() + 3
        rospy.ServiceProxy('/depth_cam/set_ldp', SetBool)(False)
        rgb_sub = message_filters.Subscriber('/depth_cam/rgb/image_raw', Image, queue_size=1)
        depth_sub = message_filters.Subscriber('/depth_cam/depth/image_raw', Image, queue_size=1)
        info_sub = message_filters.Subscriber('/depth_cam/depth/camera_info', CameraInfo, queue_size=1)

        # 同步时间戳, 时间允许有误差在0.03s
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.02)
        sync.registerCallback(self.multi_callback) #执行反馈函数

        common.loginfo("TrackAndGrapNode initailized")
        self.image_proc()

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def set_color_srv_callback(self, msg):
        rospy.loginfo("set_color")
        self.target_color = msg.data
        self.tracker = ColorTracker(self.target_color)
        # 这里原本有 self.start = True —— 已删除, 这是个安全问题。
        # 后果(2026-07-23 实测): 调 set_color 只是想建 tracker, 结果节点立刻进入待命抓取态,
        # 1 秒后就把视野里的罐子自动夹了起来。而且这条路径**没有 grab_deadline 兜底**
        # (那个只在调 grab 服务时才设), 会无限期挂着 —— 谁把绿色物体放进 ROI 就抓谁, 手也会被夹。
        # 现在"建 tracker"与"待命"彻底解耦: 待命只能由 ~start 服务或 ~grab 服务显式开启。
        return [True, 'set_color']

    def start_srv_callback(self, msg):
        rospy.loginfo("start")
        self.start = True
        return TriggerResponse(success=True)

    def grab_srv_callback(self, msg):
        rospy.loginfo("grab")
        if self.target_color is None:
            return TriggerResponse(success=False, message='target color is not set')
        if self.moving:
            return TriggerResponse(success=False, message='arm is moving')
        self.grab_done.clear()
        self.grab_result = (False, 'no result')
        self.tracker = ColorTracker(self.target_color)
        self.grab_deadline = time.time() + self.grab_timeout
        self.start = True
        # 阻塞等真实结果(抓到/IK失败/超时没找到目标都会唤醒)。
        # 服务回调在独立线程里跑, 不会挡住 image_proc。
        if not self.grab_done.wait(self.grab_timeout + 15):
            self.start = False
            self.grab_deadline = None
            return TriggerResponse(success=False, message='grab timed out waiting for result')
        success, message = self.grab_result
        return TriggerResponse(success=success, message=message)

    def place_srv_callback(self, msg):
        rospy.loginfo("place")
        if self.moving:
            return TriggerResponse(success=False, message='arm is moving')
        # 手上没东西就别白跑一趟完整放置动作
        holding, pos = self.gripper_holding()
        if not holding:
            return TriggerResponse(success=False, message='nothing to place (gripper=%s)' % pos)
        self.place_done.clear()
        self.place_result = (False, 'no result')
        self.start = False
        self.moving = True
        threading.Thread(target=self.place).start()
        if not self.place_done.wait(self.place_timeout):
            return TriggerResponse(success=False, message='place timed out waiting for result')
        success, message = self.place_result
        return TriggerResponse(success=success, message=message)

    def note_depth_fail(self, cx, cy):
        '''记下"色块找到了但那个位置没有深度"。

        以前这条路径是静默 continue, 于是 grab/measure 超时后只会报 "no target in ROI",
        完全误导 —— 实际目标看得清清楚楚, 是深度没返回。2026-07-23 为此查了很久颜色和 ROI,
        最后靠直接读深度图才发现罐子上是个空洞(整帧 89% 有值, 罐子中心 0/100)。
        结构光对反光/光滑/透明表面无返回是本机的头号老坑。
        '''
        self.last_depth_fail = time.time()
        self.last_depth_fail_px = (cx, cy)

    def no_target_reason(self, timeout):
        '''超时没出数时, 区分"没看到色块"和"看到了但没深度"。'''
        if time.time() - self.last_depth_fail < 2.0 and self.last_depth_fail_px is not None:
            return ('target seen at px=(%.0f, %.0f) but depth is invalid there '
                    '(reflective/glossy surface returns no depth) - rotate the can or wrap it in paper'
                    % self.last_depth_fail_px)
        return 'no target found in ROI within %.0fs' % timeout

    def measure_srv_callback(self, msg):
        '''
        测一次目标位置, 不动臂也不动车。返回的 message 是 JSON 字符串。
        字段: dist=深度读数(m), x/y/z=raw_pose_t(臂基座系), cx/cy/cz=pick 实际会用的 pose_t,
             px/py=像素中心。逼近环用 dist 或 x 收敛, 两者不等价, 首跑要对照一次。
        '''
        if self.moving:
            return TriggerResponse(success=False, message='busy')
        if self.tracker is None:
            return TriggerResponse(success=False, message='no color set, call set_color first')
        self.measure_result = None
        self.measure_done.clear()
        # 清掉上一次的中心, 强制这次重新等"连续两帧稳定"再出数, 避免读到陈旧值
        self.last_center = None
        self.stamp = time.time()
        self.measure_only = True
        self.start = True
        got = self.measure_done.wait(self.measure_timeout)
        self.measure_only = False
        self.start = False
        if not got or self.measure_result is None:
            return TriggerResponse(
                success=False,
                message=self.no_target_reason(self.measure_timeout))
        return TriggerResponse(success=True, message=json.dumps(self.measure_result))

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop')
        self.start = False
        self.moving = False
        self.count = 0
        self.last_pitch_yaw = (0, 0)
        self.last_center = None
        self.last_position = (0, 0, 0)
        set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 0)))
        return TriggerResponse(success=True)

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info))

    def get_endpoint(self):
        endpoint = kinematics_control.set_joint_value_target([500, 720, 100, 120, 500]).pose
        self.endpoint = common.xyz_quat_to_mat([endpoint.position.x, endpoint.position.y, endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])
        return self.endpoint

    def read_servo(self, servo_id):
        '''读单个舵机的真实位置, 读不到返回 None。注意返回的 position 只有一个元素。'''
        try:
            cmd = GetBusServoCmd()
            cmd.id = int(servo_id)
            cmd.get_position = 1
            res = self.servo_get(GetBusServoStateRequest(cmd=[cmd]))
            if res.success and res.state and len(res.state[0].position) > 0:
                return int(res.state[0].position[0])
        except Exception as e:
            rospy.logwarn("read_servo %s failed: %s", servo_id, e)
        return None

    def gripper_holding(self):
        '''
        夹爪是否夹住了东西。返回 (是否夹住, 读数)。读不到时保守地当作没夹住。
        位置语义: 0=全开, 420=全合。实测 张开~63 / 夹住易拉罐~250 / 空夹~418。
        所以"夹住"是一个区间, 不是单边阈值——张开时读数同样小于 350。
        '''
        pos = self.read_servo(10)
        if pos is None:
            return False, None
        return self.grip_open_max <= pos <= self.grip_empty_thresh, pos

    def go_home(self, gripper=0):
        '''回到初始观察姿势。失败路径也必须调用, 否则手臂会被晾在外面。'''
        set_servos(self.servos_pub, 2, ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, gripper)))
        rospy.sleep(2)

    def finish_grab(self, success, message):
        '''记录本次 grab 的真实结果并唤醒等待中的服务调用。'''
        self.start = False
        self.grab_deadline = None
        self.grab_result = (success, message)
        self.grab_done.set()
        if success:
            rospy.loginfo("grab result: SUCCESS %s", message)
        else:
            rospy.logwarn("grab result: FAILED %s", message)

    def finish_place(self, success, message):
        '''记录本次 place 的真实结果并唤醒等待中的服务调用。'''
        self.place_result = (success, message)
        self.place_done.set()
        if success:
            rospy.loginfo("place result: SUCCESS %s", message)
        else:
            rospy.logwarn("place result: FAILED %s", message)

    def pick(self, position):
        try:
            self.start = False
            target = position.copy()
            target[0] += 0.025
            # 抓取高度用常数, 不用检测值。
            # 检测的 z = "被判成目标颜色那片区域的外接框中心"的反投影, 会随反光/阴影/色块碎裂上下跑:
            # 2026-07-23 实测同一个罐子六次 target z 在 0.049~0.141 之间, 差 9cm,
            # 最低那次 0.049 直接把 IK 干成 insert unreachable。
            # 而臂基座是刚性装在车上的、罐子站在地上 ⇒ 地面在臂基座系里的高度是个常数,
            # 压根不需要每次去测。设成负数即可退回原来的"跟随检测值"行为。
            if self.grasp_height >= 0:
                rospy.loginfo("grasp height: detected z=%.3f -> fixed %.3f", target[2], self.grasp_height)
                target[2] = self.grasp_height
            # pitch 的判据要用最终高度, 不能用检测值, 否则会出现"按 z=0.25 选了 pitch=30
            # 却在 z=0.12 处抓"这种自相矛盾
            if target[2] < 0.2:
                grasp_pitch = self.low_grasp_pitch
            else:
                grasp_pitch = self.high_grasp_pitch
            self.last_grasp_target = target.copy()
            self.last_grasp_pitch = grasp_pitch
            pre_grasp = target.copy()
            pre_grasp[0] -= 0.07
            rospy.loginfo("pick start target=(%.3f, %.3f, %.3f) pre_grasp=(%.3f, %.3f, %.3f) pitch=%s",
                          target[0], target[1], target[2], pre_grasp[0], pre_grasp[1], pre_grasp[2], grasp_pitch)
            set_servos(self.servos_pub, 1, ((10, 0),))
            rospy.sleep(0.5)
            ret = kinematics_control.set_pose_target(pre_grasp, grasp_pitch)
            rospy.loginfo("pick ik pre_grasp ret=%s", ret)
            if len(ret[1]) > 0:
                cmd = ((1, ret[1][0]), (2, ret[1][1]), (3, ret[1][2]), (4, ret[1][3]), (5, ret[1][4]))
                rospy.loginfo("servo cmd pre_grasp duration=3 ids=%s", cmd)
                set_servos(self.servos_pub, 3, cmd)
                rospy.sleep(3)
            else:
                rospy.logwarn("pick pre_grasp ik failed")
                self.go_home()
                self.finish_grab(False, 'ik failed: pre_grasp unreachable (%.3f, %.3f, %.3f)' % (
                    pre_grasp[0], pre_grasp[1], pre_grasp[2]))
                self.moving = False
                return
            ret = kinematics_control.set_pose_target(target, grasp_pitch)
            rospy.loginfo("pick ik horizontal_insert ret=%s", ret)
            if len(ret[1]) > 0:
                cmd = ((1, ret[1][0]), (2, ret[1][1]), (3, ret[1][2]), (4, ret[1][3]), (5, ret[1][4]))
                rospy.loginfo("servo cmd horizontal_insert duration=2 ids=%s", cmd)
                set_servos(self.servos_pub, 2, cmd)
                rospy.sleep(2)
            else:
                rospy.logwarn("pick horizontal_insert ik failed")
                self.go_home()
                self.finish_grab(False, 'ik failed: insert unreachable (%.3f, %.3f, %.3f)' % (
                    target[0], target[1], target[2]))
                self.moving = False
                return
            cmd = ((10, 420),)
            rospy.loginfo("servo cmd close gripper duration=1 ids=%s", cmd)
            set_servos(self.servos_pub, 1, cmd)
            rospy.sleep(1)
            lift = target.copy()
            lift[2] += 0.05
            ret = kinematics_control.set_pose_target(lift, grasp_pitch)
            rospy.loginfo("pick ik lift ret=%s", ret)
            if len(ret[1]) > 0:
                cmd = ((1, ret[1][0]), (2, ret[1][1]), (3, ret[1][2]), (4, ret[1][3]), (5, ret[1][4]))
                rospy.loginfo("servo cmd lift duration=2 ids=%s", cmd)
                set_servos(self.servos_pub, 2, cmd)
                rospy.sleep(2)
            cmd = ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 420))
            rospy.loginfo("servo cmd return holding duration=2 ids=%s", cmd)
            set_servos(self.servos_pub, 2, cmd)
            rospy.sleep(2)
            rospy.loginfo("pick done")
            # 回读夹爪判断真的夹住没有: 空夹会走到 ~418, 夹住易拉罐卡在 ~250。
            holding, pos = self.gripper_holding()
            if holding:
                self.finish_grab(True, 'grasped (gripper=%s)' % pos)
            else:
                self.go_home()
                self.finish_grab(False, 'nothing grasped (gripper=%s, empty>=%s)' % (pos, self.grip_empty_thresh))
        except Exception as e:
            rospy.logerr("pick exception: %s", e)
        finally:
            self.tracker.yaw = 500
            self.tracker.pitch = 150
            self.tracker.pid_yaw.clear()
            self.tracker.pid_pitch.clear()
            self.stamp = time.time()
            self.first_time = time.time() + 2
            self.moving = False
            # 任何异常路径都要给调用方一个答复, 否则服务会一直挂着。
            if not self.grab_done.is_set():
                self.finish_grab(False, 'pick aborted unexpectedly')

    def place(self):
        try:
            # 落点优先级: 显式设定 > 上次抓取处 > 默认。
            # 显式落点走 rosparam 传, 例如:
            #   rospy.set_param('/track_and_grab/place_pose', [0.38, 0.08, 0.09])
            # 五个瓶子放收集区时用它把落点横向错开, 否则会全落在同一点摞起来。
            explicit = rospy.get_param('~place_pose', None)
            if isinstance(explicit, (list, tuple)) and len(explicit) == 3:
                target = np.array(explicit, dtype=float)
                pitch = rospy.get_param('~place_pitch', self.low_grasp_pitch)
                src = 'explicit'
            elif self.last_grasp_target is not None:
                target = np.array(self.last_grasp_target, dtype=float)
                pitch = self.last_grasp_pitch
                src = 'last_grasp'
            else:
                target = np.array(rospy.get_param('~default_place_pose', [0.38, 0.0, 0.09]), dtype=float)
                pitch = self.low_grasp_pitch
                src = 'default'
            rospy.loginfo("place pose source=%s", src)
            pre_place = target.copy()
            pre_place[2] += 0.05
            retreat = target.copy()
            retreat[0] -= 0.07
            rospy.loginfo("place start target=(%.3f, %.3f, %.3f) pre_place=(%.3f, %.3f, %.3f) retreat=(%.3f, %.3f, %.3f) pitch=%s",
                          target[0], target[1], target[2], pre_place[0], pre_place[1], pre_place[2],
                          retreat[0], retreat[1], retreat[2], pitch)
            set_servos(self.servos_pub, 1, ((10, 420),))
            rospy.sleep(0.5)
            ret = kinematics_control.set_pose_target(pre_place, pitch)
            rospy.loginfo("place ik pre_place ret=%s", ret)
            if len(ret[1]) > 0:
                cmd = ((1, ret[1][0]), (2, ret[1][1]), (3, ret[1][2]), (4, ret[1][3]), (5, ret[1][4]))
                rospy.loginfo("servo cmd place pre_place duration=3 ids=%s", cmd)
                set_servos(self.servos_pub, 3, cmd)
                rospy.sleep(3)
            else:
                rospy.logwarn("place pre_place ik failed")
                self.go_home(gripper=420)  # 还夹着东西, 保持合拢
                self.finish_place(False, 'ik failed: pre_place unreachable (%.3f, %.3f, %.3f)' % (
                    pre_place[0], pre_place[1], pre_place[2]))
                return
            ret = kinematics_control.set_pose_target(target, pitch)
            rospy.loginfo("place ik lower ret=%s", ret)
            if len(ret[1]) > 0:
                cmd = ((1, ret[1][0]), (2, ret[1][1]), (3, ret[1][2]), (4, ret[1][3]), (5, ret[1][4]))
                rospy.loginfo("servo cmd place lower duration=2 ids=%s", cmd)
                set_servos(self.servos_pub, 2, cmd)
                rospy.sleep(2)
            else:
                rospy.logwarn("place lower ik failed")
                self.go_home(gripper=420)  # 还夹着东西, 保持合拢
                self.finish_place(False, 'ik failed: lower unreachable (%.3f, %.3f, %.3f)' % (
                    target[0], target[1], target[2]))
                return
            cmd = ((10, 0),)
            rospy.loginfo("servo cmd open gripper duration=1 ids=%s", cmd)
            set_servos(self.servos_pub, 1, cmd)
            rospy.sleep(1)
            ret = kinematics_control.set_pose_target(retreat, pitch)
            rospy.loginfo("place ik retreat ret=%s", ret)
            if len(ret[1]) > 0:
                cmd = ((1, ret[1][0]), (2, ret[1][1]), (3, ret[1][2]), (4, ret[1][3]), (5, ret[1][4]))
                rospy.loginfo("servo cmd place retreat duration=2 ids=%s", cmd)
                set_servos(self.servos_pub, 2, cmd)
                rospy.sleep(2)
            cmd = ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 0))
            rospy.loginfo("servo cmd place home open duration=2 ids=%s", cmd)
            set_servos(self.servos_pub, 2, cmd)
            rospy.sleep(2)
            rospy.loginfo("place done")
            # 确认爪子真的张开了, 东西没有卡在爪里被一路带走
            pos = self.read_servo(10)
            if pos is not None and pos > 150:
                self.finish_place(False, 'gripper did not open (gripper=%s), object may be stuck' % pos)
            else:
                self.finish_place(True, 'placed at (%.3f, %.3f, %.3f) source=%s' % (
                    target[0], target[1], target[2], src))
        except Exception as e:
            rospy.logerr("place exception: %s", e)
        finally:
            self.first_time = time.time() + 2
            self.moving = False
            # 任何异常路径都要给调用方一个答复, 否则服务会一直挂着。
            if not self.place_done.is_set():
                self.finish_place(False, 'place aborted unexpectedly')

    def publish_vis(self, target):
        '''发一帧标注数据给录像用。target 为 None 表示这一帧没检测到目标。
        坐标全是**全分辨率像素**, 和 /depth_cam/rgb|depth/image_raw 同一套(两路同为 640x360)。'''
        # 没人订阅就直接返回(录制没开时不花一点 CPU)。限流到 vis_fps, 相机是 30Hz。
        if self.vis_pub.get_num_connections() <= 0:
            return
        now = time.time()
        if now - self.last_vis < self.vis_period:
            return
        self.last_vis = now
        info = {'t': round(now, 3), 'color': self.target_color, 'tracking': bool(self.start)}
        if target is None:
            info['ok'] = False
        else:
            info.update(target)
            info['ok'] = True
        self.vis_pub.publish(String(data=json.dumps(info)))

    def image_proc(self):
        while self.running:
            ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True)
            try:
                rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
                result_image = np.copy(rgb_image)
                vis_target = None   # 这一帧的标注(给录像用), 检测到才填

                h, w = depth_image.shape[:2]
                depth = np.copy(depth_image).reshape((-1, ))
                depth[depth<=0] = 55555

                sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64)

                sim_depth_image = sim_depth_image / 2000.0 * 255.0
                bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

                depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

                # grab 是一次性的: 超时还没找到目标就解除待命。
                # 否则节点会无限期挂在追踪状态, 谁把绿色物体放进 ROI 就抓谁(手也会被夹)。
                if self.start and not self.moving and self.grab_deadline is not None and time.time() > self.grab_deadline:
                    self.finish_grab(False, self.no_target_reason(self.grab_timeout))

                if self.tracker is not None and self.moving == False and time.time() > self.ttt and self.start:
                    result_image, p_y, center, r = self.tracker.proc(rgb_image, result_image, self.lab_data)
                    if p_y is not None:
                        center_x, center_y = center
                        if center_x > w:
                            center_x = w
                        if center_y > h:
                            center_y = h
                        center_stable = (
                            self.last_center is not None and
                            abs(self.last_center[0] - center_x) < 45 and
                            abs(self.last_center[1] - center_y) < 45
                        )
                        if center_stable:
                            if time.time() - self.last_debug > 1:
                                rospy.loginfo("target center=(%.1f, %.1f) stable_for=%.2fs", center_x, center_y, time.time() - self.stamp)
                                self.last_debug = time.time()
                            if time.time() - self.stamp > 0.8:
                                self.stamp = time.time()
                                roi = [int(center_y) - 5, int(center_y) + 5, int(center_x) - 5, int(center_x) + 5]
                                if roi[0] < 0:
                                    roi[0] = 0
                                if roi[1] > h:
                                    roi[1] = h
                                if roi[2] < 0:
                                    roi[2] = 0
                                if roi[3] > w:
                                    roi[3] = w
                                roi_distance = depth_image[roi[0]:roi[1], roi[2]:roi[3]]
                                try:
                                    dist = round(float(np.mean(roi_distance[np.logical_and(roi_distance>0, roi_distance<10000)])/1000.0), 3)
                                except BaseException as e:
                                    print(e)
                                    txt = "DISTANCE ERROR !!!"
                                    self.note_depth_fail(center_x, center_y)
                                    continue  # 原为 return: 会直接退出 image_proc 把整个节点弄死
                                if np.isnan(dist):
                                    txt = "DISTANCE ERROR !!!"
                                    self.note_depth_fail(center_x, center_y)
                                    continue  # 同上
                                dist += 0.015 # 物体半径补偿
                                dist += 0.015 # 误差补偿
                                K = depth_camera_info.K
                                self.get_endpoint()
                                position = depth_pixel_to_camera((center_x, center_y), dist, (K[0], K[4], K[2], K[5]))
                                position[0] -= 0.01  # rgb相机和深度相机tf有1cm偏移
                                pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标
                                world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标
                                pose_t, pose_R = common.mat_to_xyz_euler(world_pose)
                                raw_pose_t = pose_t.copy()
                                pose_t[1] -= 0.0   # 原为 -0.02: 该常数使抓取无条件右偏约2cm, 本次实验取消
                                pose_t[2] -= 0.04
                                rospy.loginfo("grab target center=(%.1f, %.1f) dist=%.3f raw_pose=(%.3f, %.3f, %.3f) corrected_pose=(%.3f, %.3f, %.3f)",
                                              center_x, center_y, dist, raw_pose_t[0], raw_pose_t[1], raw_pose_t[2], pose_t[0], pose_t[1], pose_t[2])
                                self.stamp = time.time()
                                if self.measure_only:
                                    # 逼近环调的 measure: 出数就返回, 不动臂
                                    self.measure_result = {
                                        'dist': round(float(dist), 4),
                                        'x': round(float(raw_pose_t[0]), 4),
                                        'y': round(float(raw_pose_t[1]), 4),
                                        'z': round(float(raw_pose_t[2]), 4),
                                        'cx': round(float(pose_t[0]), 4),
                                        'cy': round(float(pose_t[1]), 4),
                                        'cz': round(float(pose_t[2]), 4),
                                        'px': round(float(center_x), 1),
                                        'py': round(float(center_y), 1),
                                    }
                                    self.measure_done.set()
                                else:
                                    self.moving = True
                                    threading.Thread(target=self.pick, args=(pose_t,)).start()
                        else:
                            self.stamp = time.time()
                        self.last_center = (center_x, center_y)
                        dist = depth_image[int(center_y),int(center_x)]
                        if dist < 100:
                            txt = "TOO CLOSE !!!"
                        else:
                            txt = "Dist: {}mm".format(dist)
                        cv2.circle(result_image, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                        cv2.circle(depth_color_map, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                        cv2.putText(depth_color_map, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 0), 10, cv2.LINE_AA)
                        cv2.putText(depth_color_map, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 2, cv2.LINE_AA)
                        self.last_pitch_yaw = p_y
                        vis_target = {
                            'box': self.tracker.last_box,
                            'c': [round(float(center_x), 1), round(float(center_y), 1)],
                            'dist_mm': int(dist),
                        }
                    else:
                        pass
                self.publish_vis(vis_target)
                if self.enable_disp:
                    result_image = np.concatenate([cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR), depth_color_map, ], axis=1)
                    cv2.imshow("depth", result_image)
                    key = cv2.waitKey(1)
                    if key != -1:
                        rospy.signal_shutdown('shutdown1')

            except Exception as e:
                rospy.logerr('callback error:', str(e))

if __name__ == "__main__":
    TrackAndGrapNode('track_and_grap')
