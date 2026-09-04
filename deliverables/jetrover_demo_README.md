# JetRover Demo 素材目录

最后整理：2026-08-25。目录和文件名统一使用 ASCII，中文说明集中在本文档。

## 目录结构

```text
jetrover_demo/
├── raw/          每次录制的原始素材；一次任务一个目录
│   └── failed/   失败或只完成部分任务的录制
├── edited/       按对应任务目录归档的剪辑结果
│   └── <run>/
│       ├── final/         当前可交付成片
│       ├── intermediate/  检测框、动态 RVIZ 等合成中间件
│       ├── figures/       定性序列图和论文图
│       └── archive/       旧版本或已知有问题的版本
├── scripts/
│   ├── compose/   多视角合成脚本
│   ├── overlays/  无人机检测框脚本
│   ├── figures/   定性图生成脚本
│   └── archive/   旧脚本备份
├── external/     尚未并入 run 的外部机位素材
├── analysis/     临时对比图和排查结果
└── archive/      重复文件等保留归档
```

## edited 使用规则

- 找可交付视频：进入对应任务的 `final/`。
- 找合成输入：进入对应任务的 `intermediate/`。
- `archive/` 中的文件不作为当前推荐版本；其中可能是旧版或带已知问题的版本。
- 新导出文件必须先落到对应任务目录，不能直接写到 `edited/` 根目录。

当前三组六视角主成片：

| 任务 | 推荐版本 |
|---|---|
| `20260815_1858_gateB_2station-5can-SUCCESS-5of5` | `final/*_6view_v2_*` |
| `20260815_2120_drone5_2station-5can-SUCCESS-5of5` | `final/*_6view_v4_detection_corrected_*` |
| `20260816_1207_drone5b_2station-5can-SUCCESS-5of5` | `final/*_6view_v4_detection_corrected_*` |

07-29 单站五罐的中文五视角版位于：
`edited/20260729_2144_5can-1station-SUCCESS-5of5/final/`。
英文 `TITLE-OVERFLOW` 版本标题被裁切，已归入同一任务的 `archive/`。

## raw 命名规则

```text
raw/<YYYYMMDD>_<HHMM>_<任务简述>[-SUCCESS-NofM|-FAILED]/
```

原始录制文件保留 `record_cam.py`、RVIZ 录制和相机设备生成的原名。失败或部分完成的任务统一放在 `raw/failed/`，不使用中文目录名。

补充说明：

- `raw/20260801_1545_2station-dynamic-FAILED/` 只有 RVIZ 录像有效；车载 H.264 文件因硬断电没有写完 moov。
- `raw/20260802_final_crashed/` 是崩溃任务，相关 RVIZ 已并回该目录。
- `raw/20260802_rviz-tests/`、`raw/20260804_rviz-tests/` 和 `raw/20260815_2116_drone5_rviz-test/` 是只有 RVIZ 的测试录制。
- `archive/duplicate_top_level_rviz/` 中的 8 个文件与 `raw/` 内文件 SHA-256 完全一致，仅为保留副本。

## 脚本入口

```bash
# 多视角合成
bash scripts/compose/<script>.sh

# 无人机检测框
bash scripts/overlays/<script>.sh

# 0816 定性序列图
python3 scripts/figures/make_qualitative_sequence_0816.py

# 两组 0815 定性序列图（gateB + drone5）
python3 scripts/figures/make_qualitative_sequences_0815.py
```

脚本内的默认输出路径已经指向相应任务的 `final/`、`intermediate/` 或 `archive/`。项目外的三个动态 RVIZ 渲染脚本也已同步：

- `/home/ntuicg/render_gateB_rviz_dynamic.sh`
- `/home/ntuicg/render_0815_drone5_rviz_dynamic.sh`
- `/home/ntuicg/render_0816_drone5b_rviz_dynamic.sh`

## 同步与时间线

- 不要把小车任务日志的写入时间当作无人机首次检测时间。
- gateB 当前时间线在 `raw/20260815_1858_gateB_2station-5can-SUCCESS-5of5/drone_reveal_timeline_0815_gateB.yaml`。
- drone5 和 drone5b 的时间线分别保存在对应 run 目录。
- 修改时间线、检测框或 RVIZ reveal 逻辑前，先核对视频证据和各机位同步锚点。
