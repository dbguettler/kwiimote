#!/usr/bin/python3

import asyncio
from kasa import Discover
import rospy
from std_msgs.msg import String, Bool

class kwii_main_node:
    def __init__(self, devices):
        rospy.init_node('kwii_main_node')
        self.command_pub = rospy.Publisher("commands", String, queue_size=10)
        self.state_pub = rospy.Publisher("state", Bool, queue_size=10)
        self.devices = devices
        self.device = None
        for key, value in self.devices.items():
            if value.sys_info["alias"] == "Drew Lights":
                self.device = value
        self.button_sub = rospy.Subscriber("button_events", String, self.button_callback)
            
    def button_callback(self, data):
        if data.data == "A":
            self.command_pub.publish('{"system":{"set_relay_state":{"state":1}}}')
            self.state_pub.publish(True)
        elif data.data == "B":
            self.command_pub.publish('{"system":{"set_relay_state":{"state":0}}}')
            self.state_pub.publish(False)
        elif data.data == "HOME":
            self.state_pub.publish(self.device.is_on)
        else:
            print("NOTA")

        

async def main():
    known_devices = await Discover.discover()
    device = None
    for key, value in known_devices.items():
        if value.sys_info["alias"] == "Drew Lights":
            device = value

    if device is None:
        print("No device found")
        exit(1)
    else:
        print("Device is", device)
        
    node = kwii_main_node(known_devices)
    rospy.spin()


        

if __name__ == "__main__":
    asyncio.run(main())