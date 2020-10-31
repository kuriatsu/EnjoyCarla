#!/bin/bash

scenarios=(
"/home/kuriatsu/Source/carla_scenario/scenario/scenario_town5_simple_base.xml"
"/home/kuriatsu/Source/carla_scenario/scenario/scenario_town5_simple_1.xml"
"/home/kuriatsu/Source/carla_scenario/scenario/scenario_town5_simple_2.xml"
)
# args=""
# for scenario in "${scenarios[@]}"; do
#     args+=""
#     args+="[ ${scenario} ]"
# done

python /home/kuriatsu/Source/carla_scenario/carla_ros_bridge.py -s "${scenarios[@]}"
