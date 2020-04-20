#!/usr/bin/env python
import glob
import os
import sys
try:
	sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass

import carla
import argparse
import pyperclip
import time
import math
import random

def main():

    print(sys.version)
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    debug = world.debug

    # print(start)
    # get ego car

# ##########################
# traffic light
############################
    # light_right = []
    # light_left = []
    #
    # for carla_actor in world.get_actors():
    #     # print(carla_actor)
    #     # if carla_actor.type_id.startswith("vehicle"):
    #     if carla_actor.attributes.get('role_name') == 'ego_vehicle':
    #         ego_vehicle = carla_actor
    #         ego_pose = ego_vehicle.get_transform()
    #
    #     if carla_actor.type_id == 'traffic.traffic_light':
    #         traffic_light = carla_actor
    #         light_pose = traffic_light.get_location()
    #         vector = light_pose - carla.Location(39.7840614319,-13.4922046661,0.137185171247)
    #         print(vector.x ** 2 + vector.y ** 2)
    #         if (vector.x ** 2 + vector.y ** 2 < 9.0):
    #             print("found_light!!!")
    #             traffic_light.set_state(carla.TrafficLightState.Green)
    #             traffic_light.set_green_time(30)

            #
            # if (abs(ego_pose.rotation.yaw - light_pose.rotation.yaw) < 30):
            # else:
            #     traffic_light.set_green_time(5)

    ###################
    ### speed test ####
    ###################
    # for carla_actor in world.get_actors():
    #     if carla_actor.attributes.get('role_name') == 'hero':
    #         ego_vehicle = carla_actor
    #         print("found")
    # batch = []
    # client.apply_batch(batch)
    # print("accel")
    # for i in range(0,5):
    #     batch.append(carla.command.ApplyVelocity(ego_vehicle, carla.Vector3D(100.0,0.0,0.0)))
    #     # print(ego_vehicle.get_velocity())
    #     # time.sleep(0.02)
    # # time.sleep(1.0)
    # # batch.append(carla.command.ApplyVelocity(ego_vehicle, carla.Vector3D(0.0,0.0,0.0)))
    # client.apply_batch(batch)
    # print("brake")
    # for i in range(0,5):
    #     print(ego_vehicle.get_velocity())
    #     time.sleep(0.02)

    for carla_actor in world.get_actors():
        if carla_actor.attributes.get('role_name') == 'hero':
            ego_vehicle = carla_actor
            print("found")

    ego_vehicle.set_location(carla.Location(40.0, -190.0,0.0))
    # while True:
        # ego_vehicle.set_velocity(carla.Vector3D(100.0, 0.0,0.0))
        # time.sleep(1.0)
        # while True:

    ##########################
    #### obrtain position ####
    ##########################
        # key = raw_input('ENTER to copy transorm to clip_boad')
        # print(key)
        # key = ''
        # if key == '':
            # ego_pose = ego_vehicle.get_transform()
            # str = "{},{},{},{},{},{}".format(ego_pose.location.x, ego_pose.location.y, ego_pose.location.z, ego_pose.rotation.pitch, ego_pose.rotation.yaw, ego_pose.rotation.roll)
            # print(ego_pose.location.x, ego_pose.location.y, ego_pose.location.z, ego_pose.rotation.pitch, ego_pose.rotation.roll, ego_pose.rotation.yaw)
            # pyperclip.copy(str)
            # time.sleep(0.1)
        # elif key == 'q':
        #     print('exit')
        #     break
        # else:
        #     print('wrong key')

        # control = carla.WalkerControl(carla.Vector3D(-1.0,0.0,0.0), speed=3.5)
        # ego_vehicle.apply_control(control)
        # print(ego_vehicle.get_velocity())

if __name__ == '__main__':

    main()
