#!/usr/bin/env python3
# encoding: utf-8
"""
逼近环: 车导航到 standoff 点之后, 用纯平移把罐子送进抓取甜点区。

为什么全是平移:
  车是麦克纳姆轮, cmd_vel 的 linear.y 就是横移(见 cmd_vel_to_motor.py:44)。
  2026-07-23 实测过, 无里程计的 hector 一做大幅原地旋转必然 scan-match 漂移
  (纯旋转指令下估计位置滑了 0.42m, move_base 追鬼影不收敛, 还把定位搞乱)。
  平移基本不漂, 所以这里 angular.z 恒为 0 —— 一次都不转车。

坐标约定(臂基座系, x 前 y 左, 与车 base_footprint 同向):
  罐子 x 偏大 => 车要往前 => linear.x > 0
  罐子 y 偏大(偏左) => 车要往左 => linear.y > 0
这个同向假设没有被实测验证过, 所以第一步会被限得很小, 并且带符号自检:
动完之后如果误差反而朝反方向跑, 立刻停下报错, 不会一路跑飞。

用法:
  python3 ~/approach.py --probe             只测不动, 用来量检测范围/对照坐标
  python3 ~/approach.py                     跑逼近环
  python3 ~/approach.py --target 0.35 --key dist
"""
import sys
import json
import math
import time
import argparse

import rospy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from std_srvs.srv import Trigger

MEASURE_SRV = '/track_and_grab/measure'


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Approach:
    def __init__(self, args):
        self.args = args
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        rospy.wait_for_service(MEASURE_SRV, timeout=15)
        self.measure_srv = rospy.ServiceProxy(MEASURE_SRV, Trigger)
        rospy.sleep(0.5)
        self.moved = 0.0

    # ---------- 感知 ----------
    def measure(self, tries=2):
        '''调 track_and_grab 的 measure。返回 dict 或 None。'''
        for i in range(tries):
            try:
                res = self.measure_srv()
            except Exception as e:
                rospy.logwarn('measure service call failed: %s', e)
                return None
            if res.success:
                try:
                    return json.loads(res.message)
                except ValueError:
                    rospy.logwarn('measure returned non-json: %s', res.message)
                    return None
            rospy.loginfo('measure miss (%d/%d): %s', i + 1, tries, res.message)
            time.sleep(0.3)
        return None

    # ---------- 动作 ----------
    def drive(self, dx, dy):
        '''
        平移 (dx, dy) 米。开环发 cmd_vel 按时间积分 —— 没有里程计, 走多远不精确,
        但整个环是靠相机闭环的, 速度标定差个几成也能收敛, 只是多迭代一轮。
        cmd_vel_to_motor 有 0.5s watchdog, 所以必须持续发。
        '''
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return
        speed = self.args.speed
        dur = dist / speed
        vx, vy = dx / dist * speed, dy / dist * speed
        rospy.loginfo('  drive dx=%+.3f dy=%+.3f  (v=%.3f m/s, %.1fs)', dx, dy, speed, dur)
        rate = rospy.Rate(20)
        t0 = time.time()
        msg = Twist()
        msg.linear.x, msg.linear.y = vx, vy
        msg.angular.z = 0.0          # 绝不转车
        while time.time() - t0 < dur and not rospy.is_shutdown():
            self.cmd_pub.publish(msg)
            rate.sleep()
        self.stop()
        self.moved += dist

    def stop(self):
        msg = Twist()
        for _ in range(6):
            self.cmd_pub.publish(msg)
            time.sleep(0.04)

    # ---------- 采集 ----------
    def sweep_lateral(self):
        '''
        横向单调扫一轮, 找到就返回 measure 结果, 找不到就横向归位并返回 None。

        别用左右交替(+s,-s,+2s,-2s): 每次都横穿已搜区域, 总位移是 10*step;
        单调扫只要 3*search_max。2026-07-23 交替版把 0.6m 预算一次搜光。
        '''
        a = self.args
        offsets = []
        k = a.search_step
        while k <= a.search_max + 1e-6:
            offsets.append(round(k, 4))
            k += a.search_step
        k = -a.search_step
        while k >= -a.search_max - 1e-6:
            offsets.append(round(k, 4))
            k -= a.search_step
        cur = 0.0
        for off in offsets:
            rospy.logwarn('横向搜索: 挪到 %+.2fm', off)
            self.drive(0.0, off - cur)
            cur = off
            rospy.sleep(a.settle)
            m = self.measure(tries=1)
            if m is not None:
                rospy.loginfo('横移 %+.2fm 后采集到目标', cur)
                return m
        self.drive(0.0, -cur)          # 搜不到就横向归位
        rospy.sleep(a.settle)
        return None

    def acquire(self, hint=0):
        '''
        找到罐子。返回 (measure结果 or None, 净前后位移 米, 前为正)。

        实测(2026-07-23)横向采集窗口只有 -12.1cm ~ +10.5cm(就是 ROI 边界, px 224~460),
        角度上是左15.2°/右21.2°。车到点朝向不受控 ⇒ 采集不到是常态。

        hint: >=0 先往前找(默认); <0 先往后退着找。
        **"太近"和"太远"一样会看不见, 而且更凶** —— 2026-07-25 can3 的教训:
          - 深度相机最小量程 ~0.2m, 臂基座 x=0.22 时深度读数就在悬崖上, 出空洞;
          - 罐子越近轮廓中心在画面里越往下走, 越过 ROI 下边界(0.78)会被**整条丢弃**;
          - 而这个方向横移完全救不了(问题在纵向), 老版本 acquire 里又根本没有"后退"这个动作,
            于是就是用户看到的"左右横移也找不到"。
        后退没有撞罐风险, 所以额度可以给得比盲进大。

        阶段顺序(2026-07-23 演示踩出来的, 别改回去): 小步前进 → 横扫 → 小步后退 → 横扫 → ...
        原因: 前进一步只要 5cm, 横扫一轮要 3*search_max=0.36m; 一次本是距离问题的采集失败,
        先横扫白走了 0.53m 才靠盲进救回来。全程不转车。
        '''
        a = self.args
        m = self.measure()
        if m is not None:
            return m, 0.0

        if a.creep_max <= 0 and a.back_max <= 0:
            return self.sweep_lateral(), 0.0

        crept = 0.0      # 累计前进(正)
        backed = 0.0     # 累计后退(正)
        order = ['back', 'fwd'] if hint < 0 else ['fwd', 'back']

        while True:
            moved_this_round = False
            for d in order:
                if d == 'fwd':
                    if crept + a.creep_step > a.creep_max + 1e-6:
                        continue
                    rospy.logwarn('看不到目标, 小步前进 %.2fm (累计 %.2f / 上限 %.2f)',
                                  a.creep_step, crept + a.creep_step, a.creep_max)
                    self.drive(a.creep_step, 0.0)
                    crept += a.creep_step
                else:
                    if backed + a.creep_step > a.back_max + 1e-6:
                        continue
                    rospy.logwarn('看不到目标, 小步后退 %.2fm (累计 %.2f / 上限 %.2f)',
                                  a.creep_step, backed + a.creep_step, a.back_max)
                    self.drive(-a.creep_step, 0.0)
                    backed += a.creep_step
                moved_this_round = True
                rospy.sleep(a.settle)
                m = self.measure()
                if m is not None:
                    return m, crept - backed
                m = self.sweep_lateral()
                if m is not None:
                    return m, crept - backed

            if not moved_this_round:
                # 前后额度都用完了, 认输。**把净前进量退回去** ——
                # 否则车停在"比 standoff 更贴近一个它看不见的罐子"的位置,
                # 下一次尝试(或下一个罐子路过)就是直接顶翻它。
                net = crept - backed
                if net > 1e-3:
                    rospy.logwarn('采集失败, 退回净前进的 %.2fm', net)
                    self.drive(-net, 0.0)
                return None, 0.0

    # ---------- 主环 ----------
    def run(self):
        a = self.args
        key = a.key

        # move_base 可能还挂着 goal, 不取消会和我们抢 /cmd_vel
        # (2026-07-06 被残留 teleop 灌 0 抢过总线, 同一个坑)
        self.cancel_pub.publish(GoalID())
        rospy.sleep(0.5)

        # --- 采集: 测不到就盲进 ---
        # 实测检测在 x≈0.52m 就断了(色块太小), 而导航容差 ±10cm, 所以"停下来看不见罐子"
        # 是常态而不是异常。往前挪一点通常就看见了。
        # ⚠️ 局限: 看不见的原因可能是"太远", 也可能是"车头偏太多", 这两者靠单目分不出来。
        # 后者盲进是往罐子上撞, 所以 creep_max 必须保守。
        m, crept = self.acquire()
        # 采集阶段的位移不算进逼近阶段的预算 —— 两者是不同性质的运动,
        # budget 是用来拦"追一个测不准的目标一直爬"的, 不该被搜索吃掉。
        rospy.loginfo('采集阶段共走 %.2fm, 预算重置', self.moved)
        self.moved = 0.0
        if m is None:
            return 2, ('acquire failed: 横扫 +-%.2fm、前进 %.2fm、后退 %.2fm 之后仍看不到罐子。'
                       '车头偏得太多 / 罐子被撞到车侧面了 / 颜色标签背着镜头'
                       % (a.search_max, a.creep_max, a.back_max))
        rospy.loginfo('init  %s', json.dumps(m))

        prev = None
        prev_step = None
        lost = 0
        for it in range(1, a.max_iter + 1):
            ex = m[key] - a.target
            ey = m['y'] - a.target_y
            rospy.loginfo('[%d] %s=%.3f y=%+.3f  err=(%+.3f, %+.3f)', it, key, m[key], m['y'], ex, ey)

            # --- 符号自检: 上一步动完误差该变小, 变大说明轴向/符号搞反了 ---
            if prev_step is not None:
                obs_dx = m[key] - prev[key]
                obs_dy = m['y'] - prev['y']
                for name, cmd, obs in (('x', prev_step[0], obs_dx), ('y', prev_step[1], obs_dy)):
                    if abs(cmd) > 0.012 and abs(obs) > 0.012 and (cmd > 0) == (obs > 0):
                        self.stop()
                        return 3, ('sign check failed on %s: 指令 %+.3f 但测得误差朝同向变了 %+.3f。'
                                   '臂基座系和车 base_footprint 的 %s 轴大概不同向, 停车' % (name, cmd, obs, name))

            if abs(ex) <= a.tol_x and abs(ey) <= a.tol_y:
                self.stop()
                return 0, 'converged: %s=%.3f y=%+.3f (%d 步, 共走了 %.2fm)' % (key, m[key], m['y'], it - 1, self.moved)

            step_x = clamp(a.gain * ex, -a.max_step, a.max_step)
            step_y = clamp(a.gain * ey, -a.max_step, a.max_step)
            if it == 1:
                # 第一步限得很小, 用来验证符号和速度标定, 别一上来就冲
                step_x = clamp(step_x, -a.first_step, a.first_step)
                step_y = clamp(step_y, -a.first_step, a.first_step)

            # 步长低于死区时车可能压根不动。但只要误差还超容差, 就强制走一个死区步长 ——
            # 否则会卡在 tol 和 deadband/gain 之间的空档里空转(tol_y=0.015 时空档是 0.015~0.017)。
            if abs(step_x) < a.deadband:
                step_x = math.copysign(a.deadband, ex) if abs(ex) > a.tol_x else 0.0
            if abs(step_y) < a.deadband:
                step_y = math.copysign(a.deadband, ey) if abs(ey) > a.tol_y else 0.0

            # 宁可停远也别停近: 2026-07-23 实测 <0.31m 时 pre_grasp 伸臂会先把罐撞翻,
            # 而太远只是 IK 够不到, 重来一次即可。放在最后, 免得被上面的最小步长破坏。
            room = m[key] - a.min_dist
            lim = a.first_step if it == 1 else a.max_step
            if room < 0:
                # 已经比 min_dist 还近了(车开过头 / 罐子被顶近了) ⇒ **必须后退**。
                # 🔑 2026-07-25 can3 就死在这一行的老写法上: x=0.22 ⇒ room=-0.10,
                #    P 算出的 step_x=-0.091 也满足 `step_x > room`, 被 `max(0.0, room)`
                #    打成 0 —— 车一步都不敢退, 直接判 stalled 认输。
                #    代入 gain=0.7: 凡是 x < 0.25 都会中招, 一直藏到大场景才炸。
                # 至少退到 min_dist 外面 2cm; 第一步仍受 first_step 限, 保住符号自检。
                step_x = clamp(min(step_x, room - 0.02), -lim, lim)
                rospy.logwarn('  比 min_dist=%.2f 还近(%s=%.3f), 强制后退 %+.3f',
                              a.min_dist, key, m[key], step_x)
            elif step_x > room:
                step_x = max(0.0, room)
                rospy.logwarn('  前进步长被 min_dist=%.2f 限到 %+.3f', a.min_dist, step_x)

            if step_x == 0.0 and step_y == 0.0:
                self.stop()
                return 4, ('stalled: 还差 (%+.3f, %+.3f), 但再往前会突破 min_dist=%.2f, 收不动了'
                           % (ex, ey, a.min_dist))

            if self.moved + math.hypot(step_x, step_y) > a.budget:
                self.stop()
                return 5, 'travel budget %.2fm 用完了, 停车(可能是在追一个测不准的目标)' % a.budget

            self.drive(step_x, step_y)
            rospy.sleep(a.settle)

            prev, prev_step = m, (step_x, step_y)
            m = self.measure()
            if m is None:
                # 丢目标 ≠ 失败。2026-07-25 can3: 逼近到 x=0.22 太近 → 深度出空洞 → 丢目标,
                # 老版本在这里直接 return, 把一次本来后退 10cm 就能救回来的尝试判死了。
                # 上一次测到时如果已经比 min_dist 近, 就告诉 acquire 先往后找。
                lost += 1
                if lost > a.max_lost:
                    self.stop()
                    return 2, '目标丢了 %d 次, 重新采集也找不回来, 停车' % lost
                hint = -1 if prev[key] < a.min_dist else 0
                rospy.logwarn('目标丢了(第 %d/%d 次), 重新采集 (hint=%+d, 上次 %s=%.3f)',
                              lost, a.max_lost, hint, key, prev[key])
                mv = self.moved
                m, _ = self.acquire(hint)
                self.moved = mv          # 采集位移不算进逼近预算, 同上面初次采集
                if m is None:
                    self.stop()
                    return 2, '动完之后目标丢了, 重新采集也没找回来'
                prev_step = None         # 中间插了采集动作, 符号自检的基准作废

        self.stop()
        return 6, '迭代 %d 次仍未收敛' % a.max_iter


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--target', type=float, default=0.35, help='目标距离(m)')
    p.add_argument('--target-y', dest='target_y', type=float, default=0.0)
    p.add_argument('--key', default='dist', choices=['dist', 'x', 'cx'],
                   help='用哪个量收敛: dist=深度读数, x=raw 臂基座x, cx=pick 实际用的x')
    p.add_argument('--tol-x', dest='tol_x', type=float, default=0.015)
    # tol_y 必须 <=0.015: 07-23 实测 y=-0.025 收敛后 grab 空夹(419), 而 -0.013/-0.001/0.000 都成功。
    # 与记忆里"罐居中 y<0.02 稳抓"的判据一致。别放松。
    p.add_argument('--tol-y', dest='tol_y', type=float, default=0.015)
    p.add_argument('--min-dist', dest='min_dist', type=float, default=0.32,
                   help='绝不靠得比这更近(再近伸臂会撞翻罐)')
    p.add_argument('--gain', type=float, default=0.7)
    p.add_argument('--max-step', dest='max_step', type=float, default=0.12)
    p.add_argument('--first-step', dest='first_step', type=float, default=0.03)
    p.add_argument('--deadband', type=float, default=0.012)
    p.add_argument('--speed', type=float, default=0.06)
    p.add_argument('--settle', type=float, default=0.8)
    p.add_argument('--max-iter', dest='max_iter', type=int, default=8)
    p.add_argument('--budget', type=float, default=0.60, help='总位移上限(m), 安全阀')
    p.add_argument('--creep-step', dest='creep_step', type=float, default=0.05,
                   help='采集不到时每次盲进多少米')
    p.add_argument('--creep-max', dest='creep_max', type=float, default=0.10,
                   help='盲进总上限(m)。保守: 盲进有撞罐风险, 优先靠横向搜索')
    p.add_argument('--back-max', dest='back_max', type=float, default=0.15,
                   help='采集阶段最多往后退多少米。后退没有撞罐风险, 额度可以比盲进大; '
                        '"太近看不见"就靠它救(2026-07-25 can3)')
    p.add_argument('--max-lost', dest='max_lost', type=int, default=2,
                   help='逼近途中丢目标后最多重新采集几次(丢了不等于失败)')
    p.add_argument('--search-step', dest='search_step', type=float, default=0.06,
                   help='横向搜索每档挪多少米')
    p.add_argument('--search-max', dest='search_max', type=float, default=0.12,
                   help='横向搜索最多挪到多远(左右各)')
    p.add_argument('--then-grab', dest='then_grab', action='store_true',
                   help='收敛后在同一进程里直接调 grab, 不用等第二条命令')
    p.add_argument('--probe', action='store_true', help='只测量不动车')
    p.add_argument('--probe-n', dest='probe_n', type=int, default=5)
    p.add_argument('--nudge', nargs=2, type=float, metavar=('DX', 'DY'),
                   help='只平移这么多米然后退出(不做闭环), 手动调位用')
    p.add_argument('--sweep', action='store_true',
                   help='沿指定方向一步步挪着测, 找出还能看见罐子的边界')
    p.add_argument('--sweep-step', dest='sweep_step', type=float, default=0.05)
    p.add_argument('--sweep-max', dest='sweep_max', type=float, default=0.45,
                   help='最多挪多少米')
    p.add_argument('--sweep-dir', dest='sweep_dir', default='back',
                   choices=['back', 'fwd', 'left', 'right'],
                   help='扫描方向。back=后退测距离上限; left/right=横移测横向采集宽度')
    args = p.parse_args(rospy.myargv()[1:])

    rospy.init_node('approach', anonymous=True)
    ap = Approach(args)

    if args.probe:
        for i in range(args.probe_n):
            m = ap.measure(tries=1)
            print('%d: %s' % (i + 1, json.dumps(m) if m else 'MISS'))
            time.sleep(0.4)
        return 0

    if args.nudge:
        ap.drive(args.nudge[0], args.nudge[1])
        return 0

    if args.sweep:
        # 车往后退着测, 找检测的最远距离。相机朝前下方看, 罐子越远在画面里越靠上,
        # 迟早会掉出 ROI 上边界(画面高的 25%)。这个距离决定 standoff 点能录多远。
        # 车往哪挪, 罐子在臂基座系里就往反方向走
        step_vec = {'back': (-1.0, 0.0), 'fwd': (1.0, 0.0),
                    'left': (0.0, 1.0), 'right': (0.0, -1.0)}[args.sweep_dir]
        back = 0.0
        misses = 0
        print('# sweep dir=%s step=%.2f' % (args.sweep_dir, args.sweep_step))
        print('moved   x       y        px     py     dist')
        while back <= args.sweep_max + 1e-6 and not rospy.is_shutdown():
            m = ap.measure(tries=1)
            if m is None:
                print('%-7.2f MISS' % back)
                misses += 1
                if misses >= 2:
                    print('连续 2 次测不到, 停止扫描。最远可测距离在 back=%.2f 之前' % back)
                    break
            else:
                misses = 0
                print('%-7.2f %-7.3f %+-8.4f %-6.1f %-6.1f %-6.3f'
                      % (back, m['x'], m['y'], m['px'], m['py'], m['dist']))
            if back + args.sweep_step > args.sweep_max + 1e-6:
                break
            ap.drive(step_vec[0] * args.sweep_step, step_vec[1] * args.sweep_step)
            back += args.sweep_step
            rospy.sleep(0.6)
        print('扫描结束, 车共挪了 %.2fm。回原位: --nudge %.2f %.2f'
              % (back, -step_vec[0] * back, -step_vec[1] * back))
        return 0

    t0 = time.time()
    code, msg = ap.run()
    t_app = time.time() - t0
    print(('OK: ' if code == 0 else 'FAIL(%d): ' % code) + msg)
    print('  [逼近耗时 %.1fs]' % t_app)
    if code != 0 or not args.then_grab:
        return code

    # 收敛后直接在同一个进程里调 grab —— 演示时把 approach 和 grab 拆成两条 ssh 命令,
    # 中间要重新 source + 起 rospy + 查服务, 白等好几秒。整合进 car_run.py 时就是这么串的。
    t1 = time.time()
    grab = rospy.ServiceProxy('/track_and_grab/grab', Trigger)
    grab.wait_for_service(timeout=10)
    t_conn = time.time() - t1
    t2 = time.time()
    res = grab()
    print('GRAB: success=%s %s' % (res.success, res.message))
    print('  [连服务 %.2fs, 抓取本体 %.1fs, 逼近->抓完总计 %.1fs]'
          % (t_conn, time.time() - t2, time.time() - t0 - t_app))
    return 0 if res.success else 7


if __name__ == '__main__':
    sys.exit(main())
