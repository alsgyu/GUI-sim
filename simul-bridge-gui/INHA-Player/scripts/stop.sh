#!/bin/bash
NS=$1
if [ -z "$NS" ]; then
    echo "Usage: ./scripts/stop.sh <namespace> (e.g. robot1)"
    echo "Killing ALL robots (fallback)"
    echo ["STOP VISION"]
    killall -9 vision_node
    echo ["STOP BRAIN"]
    killall -9 brain_node
    echo ["STOP SOUND"]
    killall -9 sound_play_node
    echo ["STOP GAMECONTROLLER"]
    # killall -9 game_controller
else
    echo ["STOP VISION $NS"]
    pkill -9 -f "vision_node.*__ns:=/$NS"
    echo ["STOP BRAIN $NS"]
    pkill -9 -f "brain_node.*__ns:=/$NS"
    echo ["STOP SOUND $NS"]
    pkill -9 -f "sound_play.*__ns:=/$NS"
    # echo ["STOP GAMECONTROLLER $NS"]
    # pkill -9 -f "game_controller.*__ns:=/$NS"
fi
