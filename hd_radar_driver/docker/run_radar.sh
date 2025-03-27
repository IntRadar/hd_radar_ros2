#!/bin/bash

image_name="intradar/hd_radar_ros2"
tag="1.0.0"
container_name="hd_radar"

docker run -it -d --rm \
                --net "host" \
                --name ${container_name} ${image_name}:${tag}
docker exec --user "default_user" -it ${container_name} /bin/bash \
            -c "source /opt/ros/humble/setup.bash;
                source install/setup.bash;
                ros2 launch hd_radar_driver hd_radar.launch.py;
                /bin/bash;"
