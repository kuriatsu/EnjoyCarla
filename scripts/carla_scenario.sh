#!/bin/bash

if [ $1 = "Town05" ]; then
    scenarios=(
    "/home/kuriatsu/Source/carla_scenario/scenario/town5_pedestrian_cross0.xml"
    # "/home/kuriatsu/Source/carla_scenario/scenario/town5_pedestrian_cross1.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town5_pedestrian_stands0.xml"
    # "/home/kuriatsu/Source/carla_scenario/scenario/town5_pedestrian_stands1.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town5_static0.xml"
    # "/home/kuriatsu/Source/carla_scenario/scenario/town5_static1.xml"
    )
    python /home/kuriatsu/Source/carla_scenario/carla_ros_bridge.py -s "${scenarios[@]}"

elif [ $1 = "Town04" ]; then
    scenarios=(
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_base.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    )
    python /home/kuriatsu/Source/carla_scenario/carla_ros_bridge.py -s "${scenarios[@]}"

else
    echo "no town specified"
fi
# args=""
# for scenario in "${scenarios[@]}"; do
#     args+=""
#     args+="[ ${scenario} ]"
# done
