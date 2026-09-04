#!/bin/bash
# 车载相机录制的开关。只做透明串联: 每条命令原样打印再执行, 不加自检门。
#   bash ~/rec.sh start [tag]   开录(后台), 日志 ~/Log/rec.log
#   bash ~/rec.sh stop          停录(SIGTERM, 脚本自己收尾写完 avi)
#   bash ~/rec.sh status        在录没有 + 最近几个文件
#   bash ~/rec.sh list          列出所有已录文件
# 录制期间 run_collect5.sh 照原样跑, 互不影响。

run() { echo "+ $*"; "$@"; }

case "${1:-status}" in
  start)
    TAG="${2:-run}"
    mkdir -p ~/Log
    echo "+ setsid nohup python3 -u ~/record_cam.py --tag $TAG > ~/Log/rec.log 2>&1 &"
    ( source /opt/ros/noetic/setup.bash
      setsid nohup python3 -u ~/record_cam.py --tag "$TAG" "${@:3}" \
        > ~/Log/rec.log 2>&1 < /dev/null & )
    sleep 4
    cat ~/Log/rec.log
    ;;
  stop)
    run pkill -f "[r]ecord_cam.py"
    sleep 3
    tail -6 ~/Log/rec.log
    ;;
  status)
    pgrep -af "[r]ecord_cam.py" && echo "==> 正在录" || echo "==> 没在录"
    ls -lht ~/demo_videos/ 2>/dev/null | head -7
    ;;
  list)
    ls -lht ~/demo_videos/ 2>/dev/null
    ;;
  *)
    echo "用法: bash ~/rec.sh {start [tag] [--fps N] [--hud] | stop | status | list}"
    ;;
esac
