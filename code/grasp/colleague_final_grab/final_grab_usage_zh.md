# JetRover 易拉罐识别抓取使用说明

这套脚本用于 JetRover 小车通过深度相机识别易拉罐，并控制机械臂抓取、放回。

## 1. 文件位置

最终抓取节点：

```bash
/home/uavg/JetRover-Jetson_nano_ros1/ros_ws/src/hiwonder_example/scripts/rgbd_function/final_track_and_grab.py
```

一键启动脚本：

```bash
/home/uavg/JetRover-Jetson_nano_ros1/start_final_grab.sh
```

日常控制脚本：

```bash
/home/uavg/JetRover-Jetson_nano_ros1/final_grabctl.sh
```

运行日志：

```bash
/tmp/final_track_and_grab.log
```

## 2. 启动系统

先 SSH 进入小车，然后执行：

```bash
/home/uavg/JetRover-Jetson_nano_ros1/final_grabctl.sh start green
```

抓红色易拉罐时，把 `green` 改成 `red`：

```bash
/home/uavg/JetRover-Jetson_nano_ros1/final_grabctl.sh start red
```

默认低位抓取 pitch 是 `10`，这是当前测试成功的稳定值。完整写法是：

```bash
/home/uavg/JetRover-Jetson_nano_ros1/final_grabctl.sh start green 10 30
```

## 3. 抓取

把易拉罐放在小车正前方，让目标颜色面能被深度相机看到。然后执行：

```bash
/home/uavg/JetRover-Jetson_nano_ros1/final_grabctl.sh grab
```

抓取成功时，日志里会出现：

```text
grab started
pick ik pre_grasp ret=[True, [...]]
pick ik horizontal_insert ret=[True, [...]]
servo cmd close gripper
pick done
```

## 4. 放回

```bash
/home/uavg/JetRover-Jetson_nano_ros1/final_grabctl.sh place
```

放回成功时，日志里会出现：

```text
place started
place ik pre_place ret=[True, [...]]
place ik lower ret=[True, [...]]
servo cmd open gripper
place done
```

## 5. 查看状态

```bash
/home/uavg/JetRover-Jetson_nano_ros1/final_grabctl.sh status
```

应能看到这些关键节点或服务：

```text
/board
/depth_cam
/kinematics
/hiwonder_servo_manager
/track_and_grab
/track_and_grab/grab
/track_and_grab/place
```

## 6. 查看日志

```bash
/home/uavg/JetRover-Jetson_nano_ros1/final_grabctl.sh log
```

退出日志查看按 `Ctrl-C`。

## 7. 停止抓取节点

```bash
/home/uavg/JetRover-Jetson_nano_ros1/final_grabctl.sh stop
```

这会停止当前抓取节点，并让机械臂回到初始张开姿态。

## 8. 使用注意

- 易拉罐尽量放在小车正前方，目标颜色面朝向相机。
- 当前稳定抓取 pitch 是 `10`。`0` 和 `5` 会让夹爪更上抬，但之前测试时在偏侧目标上 IK 无解。
- 抓取逻辑不是固定轨迹硬抓，而是：颜色识别、深度测距、坐标变换、官方 IK、预抓取、水平插入、夹紧、抬起。
- `/hiwonder_servo_manager` 必须启动，因为官方 IK 需要 `/servo_controllers/port_id_1/servo_states`。
- 颜色阈值文件在：

```bash
/home/uavg/JetRover-Jetson_nano_ros1/ros_ws/src/hiwonder_example/scripts/rgbd_function/lab_config_can.yaml
```

## 9. 常见问题

如果抓取服务不存在，先重新启动：

```bash
/home/uavg/JetRover-Jetson_nano_ros1/final_grabctl.sh start green
```

如果抓取失败，看日志：

```bash
tail -n 80 /tmp/final_track_and_grab.log
```

如果日志里出现 `ik failed`，通常是易拉罐太远、太偏，或者抓取姿态不可达。先把易拉罐放回小车正前方再试。
