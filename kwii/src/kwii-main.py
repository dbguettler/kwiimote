#!/usr/bin/python3

import asyncio
from kasa import Discover
import rospy
from std_msgs.msg import String, Bool
from kwii.msg import DevCmd, DevState
from os import environ


class kwii_main_node:
    def __init__(self, devices, dev_aliases):
        # Initialize ROS node with name "kwii_main_node"
        rospy.init_node("kwii_main_node")

        # Set up publisher for Kasa device commands and device state
        self.command_pub = rospy.Publisher("commands", DevCmd, queue_size=10)
        self.state_pub = rospy.Publisher("state", DevState, queue_size=10)

        # Locate desired devices
        self.devices = [None, None]
        for key, value in devices.items():
            if value.sys_info["alias"] == dev_aliases[0]:
                self.devices[0] = value
            if value.sys_info["alias"] == dev_aliases[1]:
                self.devices[1] = value

        # Check if devices could be located
        if self.devices[0] is None:
            print("Error: could not locate device ", dev_aliases[0])
            exit(1)
        else:
            print("Located device", dev_aliases[0], "at", self.devices[0].host)
        if self.devices[1] is None:
            print("Error: could not locate device ", dev_aliases[1])
            exit(1)
        else:
            print("Located device", dev_aliases[1], "at", self.devices[1].host)

        # Set up subscriber for wiimote button presses
        self.button_sub = rospy.Subscriber(
            "button_events", String, self.button_callback
        )
        self.state_initialized = False
        self.current_device = 0
        self.statuses = [self.devices[0].is_on, self.devices[1].is_on]

    # Callback function for wiimote button events
    def button_callback(self, data):
        # If the initial state has not been sent, do that now
        if not self.state_initialized:
            self.state_initialized = True
            state_msg = DevState()
            state_msg.device = self.current_device
            state_msg.state = self.statuses[self.current_device]
            self.state_pub.publish(state_msg)

        if data.data == "A":
            # A button pressed

            # Get the command to send based on the device type
            cmd_msg = DevCmd()
            cmd_msg.cmd = self.get_command(
                self.devices[self.current_device].sys_info["mic_type"], "ON"
            )

            if not cmd_msg.cmd == "":
                # Getting the command shouldn't fail, but if it did this isn't executed

                # Set the destination ip and publish the command
                cmd_msg.ip = self.devices[self.current_device].host
                self.command_pub.publish(cmd_msg)

                # Construct a DevState message to send to the wiimote control node
                state_msg = DevState()
                state_msg.device = self.current_device
                state_msg.state = True

                # Publish state and update internal state
                self.state_pub.publish(state_msg)
                self.statuses[self.current_device] = True
        elif data.data == "B":
            # B button pressed

            # Get the command to send based on the device type
            cmd_msg = DevCmd()
            cmd_msg.cmd = self.get_command(
                self.devices[self.current_device].sys_info["mic_type"], "OFF"
            )

            if not cmd_msg.cmd == "":
                # Getting the command shouldn't fail, but if it did this isn't executed

                # Set the destination ip and publish the command
                cmd_msg.ip = self.devices[self.current_device].host
                self.command_pub.publish(cmd_msg)

                # Construct a DevState message to send to the wiimote control node
                state_msg = DevState()
                state_msg.device = self.current_device
                state_msg.state = False

                # Publish and update internal state
                self.state_pub.publish(state_msg)
                self.statuses[self.current_device] = False
        elif data.data == "1":
            # Swap device and send state to wiimote control node
            self.current_device = 0
            state_msg = DevState()
            state_msg.device = self.current_device
            state_msg.state = self.statuses[self.current_device]
            self.state_pub.publish(state_msg)

        elif data.data == "2":
            # Swap device and send state to wiimote control node
            self.current_device = 1
            state_msg = DevState()
            state_msg.device = self.current_device
            state_msg.state = self.statuses[self.current_device]
            self.state_pub.publish(state_msg)
        else:
            # Button event was sent but no action has been specified
            print("No actions for button press.")

    # Gets the proper command to send based on the device type and desired action
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
    # Discover devices and construct node
    known_devices = await Discover.discover()
    dev_one = environ.get("DEV_1")
    dev_two = environ.get("DEV_2")
    if dev_two == "":
        dev_two = dev_one
    node = kwii_main_node(known_devices, (dev_one, dev_two))
    # Spin
    rospy.spin()


if __name__ == "__main__":
    asyncio.run(main())
