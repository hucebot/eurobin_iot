#!/bin/bash

xhost +local:docker

docker run -it --net host -v $1:/ros2_ws -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ros2


    
    
