#!/bin/bash

arch=$(uname -m)
image_name="humble/hd_radar"
tag="1.0.0"
container_name="hd_radar"

docker run -it -d --rm \
                -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                -e DISPLAY=$DISPLAY \
                -e QT_X11_NO_MITSHM=1 \
                -e XAUTHORITY \
                -e ROS_DOMAIN_ID=80 \
                -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
                --net "host" \
                --ipc "host" \
                --pid "host" \
                --privileged \
                --name ${container_name} ${image_name}:${tag}
docker exec --user "default_user" -it ${container_name} /bin/bash \
            -c "source /opt/ros/humble/setup.bash;
                source install/setup.bash;
                ros2 launch hd_radar_driver hd_radar.launch.py "rqt:='true'" "rviz:='true'";
                /bin/bash;"
