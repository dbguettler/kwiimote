#!/usr/bin/python3

import asyncio
from kasa import Discover
import rospy
from std_msgs.msg import String, Bool
from kwii.msg import DevCmd


class kwii_main_node:
    def __init__(self, devices, alias):
        rospy.init_node('kwii_main_node')
        self.command_pub = rospy.Publisher("commands", DevCmd, queue_size=10)
        self.state_pub = rospy.Publisher("state", Bool, queue_size=10)
        self.devices = devices
        self.device = None
        for key, value in self.devices.items():
            if value.sys_info["alias"] == alias:
                self.device = value
        if self.device is None:
            print("Error: could not locate device ", alias)
            exit(1)
        else:
            print("Located device", alias, "at", self.device.host)
        self.button_sub = rospy.Subscriber("button_events", String, self.button_callback)
        self.state_initialized = False
            
    def button_callback(self, data):
        if not self.state_initialized:
            self.state_initialized = True
            self.state_pub.publish(self.device.is_on)

        if data.data == "A":
            cmd_msg = DevCmd()
            cmd_msg.cmd = self.get_command(self.device.sys_info["mic_type"], "ON")

            if not cmd_msg.cmd == "":
                cmd_msg.ip = self.device.host
                self.command_pub.publish(cmd_msg)
                self.state_pub.publish(True)
        elif data.data == "B":
            cmd_msg = DevCmd()
            cmd_msg.cmd = self.get_command(self.device.sys_info["mic_type"], "OFF")

            if not cmd_msg.cmd == "":
                cmd_msg.ip = self.device.host
                self.command_pub.publish(cmd_msg)
                self.state_pub.publish(False)
        else:
            print("NOTA")
    
    def get_command(self, mic_type, action):
        if mic_type == "IOT.SMARTPLUGSWITCH":
            if action == "ON":
                return '{"system":{"set_relay_state":{"state":1}}}'
            elif action == "OFF":
                return '{"system":{"set_relay_state":{"state":0}}}'
        elif mic_type == "IOT.SMARTBULB":
            if action == "ON":
                return '{"smartlife.iot.smartbulb.lightingservice":{"transition_light_state":{"on_off":1,"transition_period":0}}}'
            elif action == "OFF":
                return '{"smartlife.iot.smartbulb.lightingservice":{"transition_light_state":{"on_off":0,"transition_period":0}}}'
        return ""

        

async def main():
    known_devices = await Discover.discover()       
    node = kwii_main_node(known_devices, "Drew Lamp")
    rospy.spin()


        

if __name__ == "__main__":
    asyncio.run(main())