#!/bin/bash

NS=$1
if [ -z "$NS" ]; then
    killall -9 vision_node
    killall -9 brain_node
else
    pkill -9 -f "vision_node.*__ns:=/$NS"
    pkill -9 -f "brain_node.*__ns:=/$NS"
fi