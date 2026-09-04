#!/bin/bash
# Nav+grasp demo: pick canA(green) + canB(red), place each at collect.
# Runs on the car. Logs step markers so it can be polled.
set +e
P="[DEMO]"
GRASP_SRC="source /opt/ros/noetic/setup.bash; source /home/uavg/ros_car/devel/setup.bash; source /home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/setup.bash; export MACHINE_TYPE=JetRover_Mecanum"
NAV_SRC="source /opt/ros/noetic/setup.bash; source /home/uavg/ros1_ws/devel/setup.bash"
ts() { date +%H:%M:%S; }

nav() {
  echo "$P $(ts) NAV -> $1"
  local out
  out=$(bash -c "$NAV_SRC; python3 /home/uavg/llm_nav_commander.py goto --place $1 --wait" 2>&1)
  echo "$out"
  echo "$out" | grep -q '"state_text": "SUCCEEDED"'
}

set_color() {
  echo "$P $(ts) SET_COLOR $1"
  bash -c "$GRASP_SRC; rosservice call /track_and_grab/set_color \"data: '$1'\"" 2>&1 | grep -E "success|message"
}

grab() {
  echo "$P $(ts) APPROACH+GRAB"
  local out
  out=$(bash -c "$GRASP_SRC; python3 -u /home/uavg/approach.py --then-grab" 2>&1)
  echo "$out" | tail -22
  echo "$out" | grep -q "success=True"
}

place() {
  echo "$P $(ts) PLACE"
  bash -c "$GRASP_SRC; rosservice call /track_and_grab/place '{}'" 2>&1 | grep -E "success|message"
}

pkill -f "[t]eleop.py" 2>/dev/null
echo "$P $(ts) ===== DEMO START ====="

echo "$P $(ts) --- reposition to start ---"
nav start || { echo "$P NAV start FAILED, abort"; exit 1; }
sleep 2

for pair in "canA green" "canB red"; do
  set -- $pair; PLACE=$1; COLOR=$2
  echo "$P $(ts) ========== $PLACE ($COLOR) =========="
  nav $PLACE || { echo "$P NAV $PLACE FAILED, abort"; exit 1; }
  sleep 2
  set_color $COLOR
  sleep 1
  if grab; then
    echo "$P $(ts) GRABBED $PLACE -> collect"
    nav collect || { echo "$P NAV collect FAILED, abort"; exit 1; }
    sleep 2
    place
  else
    echo "$P $(ts) GRAB FAILED for $PLACE, skip place, continue"
  fi
done

echo "$P $(ts) ===== DEMO DONE ====="
