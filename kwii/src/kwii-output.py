#!/usr/bin/python3

import asyncio
from kasa import Discover
import rospy
from std_msgs.msg import String
        

def callback(data, publ):
    if data.data == "A":
        print("ON")
        publ.publish('{"system":{"set_relay_state":{"state":1}}}')
    elif data.data == "B":
        print("OFF")
        publ.publish('{"system":{"set_relay_state":{"state":0}}}')

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
        
    pub = rospy.Publisher("commands", String, queue_size=10)
    rospy.init_node('kwii_output_node')
    rospy.Subscriber('button_events', String, callback, (pub))
    rospy.spin()


        

if __name__ == "__main__":
    asyncio.run(main())