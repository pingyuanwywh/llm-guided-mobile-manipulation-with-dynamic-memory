#!/bin/bash
# 在用户 GNOME 桌面弹一个 gnome-terminal 窗口。
# 用法: pop.sh "标题" "要在终端里执行的命令"
GS=$(pgrep -u "$USER" gnome-session | head -1)
ENVFILE="/proc/$GS/environ"
DBUS=$(tr '\0' '\n' < "$ENVFILE" 2>/dev/null | grep -m1 '^DBUS_SESSION_BUS_ADDRESS=' | cut -d= -f2-)
DISP=$(tr '\0' '\n' < "$ENVFILE" 2>/dev/null | grep -m1 '^DISPLAY=' | cut -d= -f2-)
DISPLAY="${DISP:-:1}" DBUS_SESSION_BUS_ADDRESS="$DBUS" \
  gnome-terminal --title="$1" -- bash -c "$2; echo; echo '===[窗口保留,可在此手动操作/重跑]==='; exec bash" &
