#!/bin/bash

green=`tput setaf 2`
violet=`tput setaf 5`
reset=`tput sgr0`

path="/home/docker_radar/colcon_ws/src"
tag="1.0.0"
arch=$(uname -m)

dockerfile="Dockerfile.humble"
image_name="humble/hd_radar"


echo ${green}"Building image from Dockerfile: ${violet}$dockerfile"${reset};
docker build . --no-cache\
    -f ${dockerfile} \
    --tag ${image_name}:${tag};
