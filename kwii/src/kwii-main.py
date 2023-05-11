#!/usr/bin/python3

import asyncio
from kasa import Discover
import rospy
from std_msgs.msg import String, Bool
from kwii.msg import DevCmd

class kwii_main_node:
    def __init__(self, devices):
        rospy.init_node('kwii_main_node')
        self.command_pub = rospy.Publisher("commands", DevCmd, queue_size=10)
        self.state_pub = rospy.Publisher("state", Bool, queue_size=10)
        self.devices = devices
        self.device = None
        for key, value in self.devices.items():
            if value.sys_info["alias"] == "Drew Lights":
                self.device = value
        self.button_sub = rospy.Subscriber("button_events", String, self.button_callback)
            
    def button_callback(self, data):
        if data.data == "A":
            cmd_msg = DevCmd()
            cmd_msg.cmd = '{"system":{"set_relay_state":{"state":1}}}'
            cmd_msg.ip = self.device.host
            self.command_pub.publish(cmd_msg)
            self.state_pub.publish(True)
        elif data.data == "B":
            cmd_msg = DevCmd()
            cmd_msg.cmd = '{"system":{"set_relay_state":{"state":0}}}'
            cmd_msg.ip = self.device.host
            self.command_pub.publish(cmd_msg)
            self.state_pub.publish(False)
        elif data.data == "HOME":
            self.state_pub.publish(self.device.is_on)
        else:
            print("NOTA")

        

async def main():
    known_devices = await Discover.discover()        
    node = kwii_main_node(known_devices)
    rospy.spin()


        

if __name__ == "__main__":
    asyncio.run(main())