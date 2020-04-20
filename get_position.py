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
import random

def main(args):

    print(sys.version)
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    # print(start)
    # get ego car
    for carla_actor in world.get_actors():
        print(carla_actor.attributes.get('role_name'))
        # if carla_actor.type_id.startswith("vehicle"):
        if carla_actor.attributes.get('role_name') == args[1]:
            ego_vehicle = carla_actor
    ####################
    #### speed test ####
    ####################
    # batch = []
    # batch.append(carla.command.ApplyVelocity(ego_vehicle, carla.Vector3D(100.0,0.0,0.0)))
    # client.apply_batch(batch)
    # print("accel")
    # for i in range(0,5):
    #     print(ego_vehicle.get_velocity())
    #     time.sleep(0.02)
    # time.sleep(1.0)
    # batch.append(carla.command.ApplyVelocity(ego_vehicle, carla.Vector3D(0.0,0.0,0.0)))
    # client.apply_batch(batch)
    # print("brake")
    # for i in range(0,5):
    #     print(ego_vehicle.get_velocity())
    #     time.sleep(0.02)


    ##########################
    #### obrtain position ####
    ##########################

    while True:
        key = raw_input('ENTER to copy transorm to clip_boad')
        # print(key)
        key = ''
        if key == '':
            ego_pose = ego_vehicle.get_transform()
            str = "{},{},{},{},{},{}".format(ego_pose.location.x, ego_pose.location.y, ego_pose.location.z, ego_pose.rotation.pitch, ego_pose.rotation.yaw, ego_pose.rotation.roll)
            # print(ego_pose.location.x, ego_pose.location.y, ego_pose.location.z, ego_pose.rotation.pitch, ego_pose.rotation.roll, ego_pose.rotation.yaw)
            pyperclip.copy(str)
            time.sleep(0.1)
        elif key == 'q':
            print('exit')
            break
        else:
            print('wrong key')

if __name__ == '__main__':

    main(sys.argv)
