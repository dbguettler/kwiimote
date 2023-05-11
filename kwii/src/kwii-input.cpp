#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <wiiuse.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "kwii/DevState.h"

#define MAX_WIIMOTES 1

short any_wiimote_connected(wiimote **wm, int wiimotes);
void state_callback(const kwii::DevState::ConstPtr &msg);

wiimote *wm;

int main(int argc, char **argv)
{
    // Create a ROS node called "kwii_input_node"
    ros::init(argc, argv, "kwii_input_node");

    // Pair a wii remote
    wiimote **wiimotes;
    int found, connected;

    // Init wiimote array
    wiimotes = wiiuse_init(MAX_WIIMOTES);
    wm = wiimotes[0];

    std::cout << "\n==================================================================\n"
              << "==================================================================\n"
              << "Press SYNC or (1) and (2) simultaneously to pair the Wii remote...\n"
              << "==================================================================\n"
              << "==================================================================\n"
              << std::endl;

    // Find wiimotes to use
    found = wiiuse_find(&wm, MAX_WIIMOTES, 5);
    if (!found)
    {
        std::cout << " No wiimotes found.\n"
                  << std::endl;
        wiiuse_cleanup(wiimotes, MAX_WIIMOTES);
        return 0;
    }

    // Connect to the wiimotes
    connected = wiiuse_connect(&wm, MAX_WIIMOTES);
    if (connected)
    {
        std::cout << "Connected to " << connected << " wiimotes (of " << found << " found)." << std::endl;
    }
    else
    {
        std::cout << "Failed to connect to any wiimote.\n"
                  << std::endl;
        wiiuse_cleanup(wiimotes, MAX_WIIMOTES);
        return 0;
    }

    // Turn on P1 LED and rumble briefly to indicate connection success.
    // Additionally, turn on P3 LED to indicate the device
    wiiuse_set_leds(wm, WIIMOTE_LED_1 | WIIMOTE_LED_3);
    wiiuse_rumble(wm, 1);
    usleep(200000);
    wiiuse_rumble(wm, 0);

    // Advertise publishing the button events to "/button_events"
    // and subscribe to "/state"
    ros::NodeHandle handle;

    ros::Publisher pub = handle.advertise<std_msgs::String>("button_events", 10);
    ros::Subscriber sub = handle.subscribe("state", 100, state_callback);

    // Print useful information for the user
    std::cout << "Wiimote battery: " << wm->battery_level * 100 << " %" << std::endl;
    std::cout << "Power off the Wii remote to exit." << std::endl;

    // Event loop
    while (any_wiimote_connected(&wm, MAX_WIIMOTES) && ros::ok())
    {
        if (wiiuse_poll(&wm, MAX_WIIMOTES) && wm->event == WIIUSE_EVENT)
        {
            // An event has occurred
            std_msgs::String str_msg;
            str_msg.data = "";
            // I don't like the huge else if chain but it's the best I've got right now.
            // Send the appropriate message if button is any of {A, B, HOME, 1, 2}
            if (IS_PRESSED(wm, WIIMOTE_BUTTON_A))
            {
                str_msg.data = "A";
            }
            else if IS_PRESSED (wm, WIIMOTE_BUTTON_B)
            {
                str_msg.data = "B";
            }
            else if IS_PRESSED (wm, WIIMOTE_BUTTON_HOME)
            {
                str_msg.data = "HOME";
            }
            else if IS_PRESSED (wm, WIIMOTE_BUTTON_ONE)
            {
                str_msg.data = "1";
            }
            else if IS_PRESSED (wm, WIIMOTE_BUTTON_TWO)
            {
                str_msg.data = "2";
            }

            if (str_msg.data.compare("") != 0)
            {
                pub.publish(str_msg);
            }
        }

        // Handle any incoming state messages
        ros::spinOnce();
    }

    // Either ROS is exiting or the wiimote disconnected.
    // Cleanup the wii remotes.
    wiiuse_cleanup(wiimotes, MAX_WIIMOTES);

    // Print final message and exit
    std::cout << "Done!" << std::endl;
    return EXIT_SUCCESS;
}

// Determines if any wii remote is connected
short any_wiimote_connected(wiimote **wm, int wiimotes)
{
    int i;
    if (!wm)
    {
        return 0;
    }

    for (i = 0; i < wiimotes; i++)
    {
        if (wm[i] && WIIMOTE_IS_CONNECTED(wm[i]))
        {
            return 1;
        }
    }

    return 0;
}

/**
 * Callback for updating the displayed device state.
 *
 * @param msg DevState ROS message containing the device number and message
 */
void state_callback(const kwii::DevState::ConstPtr &msg)
{
    // Decompose the message
    bool state = msg->state;
    int16_t device = msg->device;
    // Print a debug message
    std::cout << "Device " << device << " state: " << state << std::endl;
    // Set the LEDs to indicate the device state and which device
    wiiuse_set_leds(wm, WIIMOTE_LED_1 | (state ? WIIMOTE_LED_2 : WIIMOTE_LED_NONE) | (device == 0 ? WIIMOTE_LED_3 : WIIMOTE_LED_4));
}