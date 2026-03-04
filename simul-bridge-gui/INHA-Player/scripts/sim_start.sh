#!/bin/bash

cd `dirname $0`
cd ..

NS=robot0
TREE=game.xml
for arg in "$@"; do
  case "$arg" in
    ns:=*) NS="${arg#ns:=}" ;;
    tree:=*) TREE="${arg#tree:=}" ;;
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

# Start GC Bridge and Joy node with namespace for all robots to avoid collisions in simulation
nohup ros2 launch game_controller launch.py --ros-args -r __ns:=/${NS} > game_controller_${NS}.log 2>&1 &
nohup ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0 -r __ns:=/${NS} > joystick_${NS}.log 2>&1 &

nohup ros2 run vision vision_node ./src/vision/config/vision.yaml --ros-args -p use_sim_time:=true -r __ns:=/${NS} > vision_${NS}.log 2>&1 &
nohup ros2 launch brain launch.py "$@" ns:=${NS} tree:=${TREE} sim:=true player_id:=$PLAYER_ID role:=$ROLE > brain_${NS}.log 2>&1 &
# Create a symlink or log copy for the GUI's expected launcher.log
ln -sf brain_${NS}.log launcher_${NS}.log

# ./scripts/sim_start.sh ns:=robot0