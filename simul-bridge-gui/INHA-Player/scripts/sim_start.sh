#!/bin/bash

cd `dirname $0`
cd ..

NS=robot0
for arg in "$@"; do
  case "$arg" in
    ns:=*) NS="${arg#ns:=}" ;;
  esac
done

source ./install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml

./scripts/sim_stop.sh $NS

# Extract player ID from namespace (e.g. robot1 -> 1, robot_1 -> 1)
PLAYER_ID=$(echo "$NS" | tr -dc '0-9')
if [ -z "$PLAYER_ID" ]; then
    PLAYER_ID=1
fi

# Assign role based on player ID
if [ "$PLAYER_ID" -eq 1 ]; then
    ROLE="goal_keeper"
else
    ROLE="striker"
fi

if [ "$PLAYER_ID" -eq 1 ]; then
    nohup ros2 launch game_controller launch.py > game_controller.log 2>&1 &
    nohup ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0 > joystick.log 2>&1 &
fi

nohup ros2 run vision vision_node ./src/vision/config/vision.yaml --ros-args -p use_sim_time:=true -r __ns:=/${NS} > vision.log 2>&1 &
nohup ros2 launch brain launch.py "$@" sim:=true player_id:=$PLAYER_ID role:=$ROLE > brain.log 2>&1 &
# ./scripts/sim_start.sh ns:=robot0