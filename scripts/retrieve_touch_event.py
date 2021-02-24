#!/usr/bin/python
# -*- coding: utf-8 -*-

import evdev
import rospy
from std_msgs.msg import Bool


class RetrieveTouchEvent():
    def __init__(self):
        self.device = None
        self.pub = None

    def loop(self):
        for event in self.device.read_loop():
            if event.type == evdev.ecodes.EV_KEY:
                if event.value:
                    self.pub.publish(Bool(data=True))

    def __del__(self):
        del self.device

def main():
    rospy.init_node('retrieve_touch_event_node')
    retrieve_touch_event = RetrieveTouchEvent()
    retrieve_touch_event.device = evdev.InputDevice(rospy.get_param('/retrieve_touch_event_node/touch_device'))
    # retrieve_touch_event.device = evdev.InputDevice('/dev/input/event16')
    retrieve_touch_event.pub = rospy.Publisher('touch_event', Bool, queue_size=1)
    print('initialized')

    retrieve_touch_event.loop()

if __name__ == '__main__':
    main()
