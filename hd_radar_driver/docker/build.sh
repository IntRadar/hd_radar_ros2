#!/bin/bash

green=`tput setaf 2`
violet=`tput setaf 5`
reset=`tput sgr0`

path="/home/docker_radar/colcon_ws/src"
tag="1.2.0"

dockerfile="Dockerfile.humble"
image_name="intradar/hd_radar_ros2"

echo ${green}"Building image from Dockerfile: ${violet}$dockerfile"${reset};
docker build . --no-cache\
    -f ${dockerfile} \
    --tag ${image_name}:${tag};
