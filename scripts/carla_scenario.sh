#!/bin/bash

if [ $1 = "Town05" ]; then
    scenarios=(
    "/home/kuriatsu/Source/carla_scenario/scenario/scenario_town5_simple_base.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/scenario_town5_simple_1.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/scenario_town5_simple_2.xml"
    )
    python /home/kuriatsu/Source/carla_scenario/carla_ros_bridge.py -s "${scenarios[@]}"

elif [ $1 = "Town04" ]; then
    scenarios=(
    "/home/kuriatsu/Source/carla_scenario/scenario/scenario_town4.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/scenario_town4_intervene.xml"
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
