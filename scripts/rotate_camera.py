#!/usr/bin/python
# -*- coding: utf-8 -*-

import glob
import os
import sys
import argparse
import math
try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32

class RotateCamera():

    def __init__(self):
        self.camera = None
        self.camera_location_list = {
            0    : carla.Transform(location=carla.Location(0.5, 0.0, 2.0), rotation=carla.Rotation(yaw=0.0)),
            -0    : carla.Transform(location=carla.Location(0.5, 0.0, 2.0), rotation=carla.Rotation(yaw=0.0)),
            45   : carla.Transform(location=carla.Location(0.5, 0.5, 2.0), rotation=carla.Rotation(yaw=45.0)),
            -45  : carla.Transform(location=carla.Location(0.5, -0.5, 2.0), rotation=carla.Rotation(yaw=-45.0)),
            90   : carla.Transform(location=carla.Location(0.0, 0.5, 2.0), rotation=carla.Rotation(yaw=90.0)),
            -90  : carla.Transform(location=carla.Location(0.0, -0.5, 2.0), rotation=carla.Rotation(yaw=-90.0)),
            135  : carla.Transform(location=carla.Location(-0.5, 0.5, 2.0), rotation=carla.Rotation(yaw=135.0)),
            -135 : carla.Transform(location=carla.Location(-0.5, -0.5, 2.0), rotation=carla.Rotation(yaw=-135.0)),
            180  : carla.Transform(location=carla.Location(-0.5, 0.0, 2.0), rotation=carla.Rotation(yaw=180.0)),
            -180 : carla.Transform(location=carla.Location(-0.5, 0.0, 2.0), rotation=carla.Rotation(yaw=180.0)),
        }


    def rotateCamera(self, in_msg):
        # euler = euler_from_quaternion([in_msg.x, in_msg.y, in_msg.z, in_msg.w]);
        yaw = math.degrees(in_msg.data)
        yaw = (yaw // 45) * -45 if yaw >= 0 else (yaw // -45) * (45)
        # print(euler[2], yaw)
        self.camera.set_transform(self.camera_location_list.get(yaw))


def main():
    """
    Initializes the client-side bounding box demo.
    """

    argparser = argparse.ArgumentParser(
        description='Carla image viewer for demo')
    argparser.add_argument(
        '--host',
        default='127.0.0.1',
        help='IP of the host server (127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-c', '--camera_name',
        metavar='NAME',
        default='front',
        help='camera role name (default: "wide_front")')

    args, unknown = argparser.parse_known_args()

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    rospy.init_node('carla_camera_angle_node')

    rotate_camera = RotateCamera()

    rospy.Subscriber('/carla_camera_angle', Float32, rotate_camera.rotateCamera)

    for actor in world.get_actors():
        if actor.attributes.get('role_name') == args.camera_name:
            rotate_camera.camera = actor
            print('find_camera')

    rospy.spin()


if __name__ == '__main__':
    main()
