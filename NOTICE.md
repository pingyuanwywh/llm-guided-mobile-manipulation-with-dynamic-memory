# 第三方代码与数据说明

本仓库的 `LICENSE`（MIT）**只覆盖本仓库自有的代码**。以下几处需要单独说明。

## 1. 幻尔科技（Hiwonder）JetRover 出厂例程

`code/grasp/final_track_and_grab_y0.py` **派生自**幻尔科技随 JetRover 出厂的例程：

```
hiwonder_example/scripts/rgbd_function/final_track_and_grab.py
```

派生版本改动很大（重写了颜色分割、深度取点、IK 调用、夹爪判据、失败重试、
`vis_info` 可视化输出等），但**骨架和部分逻辑来自厂商原版**。

**厂商原版不随本仓库分发。** 原版版权归幻尔科技所有，请从随机附带的 JetRover
系统镜像里取（路径同上）。`code/grasp/colleague_final_grab/` 目录下保留的是我们
自己写的启动封装 (`start_final_grab.sh`, `final_grabctl.sh`)、中文使用说明
(`final_grab_usage_zh.md`) 和现场标定的颜色阈值 (`lab_config_can.yaml`)，
不含厂商源码。

如果你是幻尔科技的权利人并认为这里的派生版本仍有不妥，请开 issue，我会配合处理。

## 2. ROS 生态

项目运行依赖 ROS Noetic 及以下开源包，均未修改、未随本仓库分发：
`hector_slam`、`teb_local_planner`、`move_base`、`amcl`、`map_server`、
`robot_localization`、`rplidar_ros`、`OrbbecSDK_ROS1`。

## 3. 运行数据里的隐私处理

`run_data/logs/*.log` 里出现过的校园内网 IP 已统一替换成 `<CAR_IP>`。
现场参数（车 IP / 本机 IP）走 `~/.jetrover_env`，该文件不进 git，
样例见 `jetrover_env.example`。
