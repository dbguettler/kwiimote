#!/usr/bin/python3

import asyncio
from kasa import Discover
import rospy
from std_msgs.msg import String
        

def callback(data, device):
    my = asyncio.new_event_loop()
    asyncio.set_event_loop(my)
    if data.data == "A":
        print("ON", flush=True)
        asyncio.run(device.turn_on())
    elif data.data == "B":
        print("OFF", flush=True)
        asyncio.run(device.turn_off())

async def main():
    loop = asyncio.get_event_loop()
    known_devices = await Discover.discover()
    device = None
    for key, value in known_devices.items():
        if value.sys_info["alias"] == "Drew Lights":
            device = value

    if device is None:
        print("No device found")
        exit(1)
    else:
        print("Device is", device, flush=True)

    rospy.init_node('kwii_output_node')
    rospy.Subscriber('button_events', String, callback, (device))
    rospy.spin()


        

if __name__ == "__main__":
    asyncio.run(main())