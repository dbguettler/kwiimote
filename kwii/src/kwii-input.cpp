#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <wiiuse.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#define MAX_WIIMOTES 1

short any_wiimote_connected(wiimote **wm, int wiimotes);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kwii_input_node");
    wiimote **wiimotes;
    wiimote *wm;
    int found, connected;

    // Init wiimote array
    wiimotes = wiiuse_init(MAX_WIIMOTES);
    wm = wiimotes[0];

    // std::cout << "Searching for wiimotes..." << std::endl;
    // std::cout << std::flush;

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
        // printf("Connected to %i wiimotes (of %i found).\n", connected, found);
        std::cout << "Connected to " << connected << " wiimotes (of " << found << " found)." << std::endl;
    }
    else
    {
        // printf("Failed to connect to any wiimote.\n");
        std::cout << "Failed to connect to any wiimote.\n"
                  << std::endl;
        wiiuse_cleanup(wiimotes, MAX_WIIMOTES);
        return 0;
    }

    // Turn on P1/P4 LED and rumble briefly to indicate connection success
    wiiuse_set_leds(wm, WIIMOTE_LED_1 | WIIMOTE_LED_4);
    wiiuse_rumble(wm, 1);
    usleep(200000);
    wiiuse_rumble(wm, 0);

    ros::NodeHandle handle;

    ros::Publisher pub = handle.advertise<std_msgs::String>("button_events", 10);

    // printf("\n\nPress 'A' on the Wii remote to exit...\n");
    std::cout << "Power off the Wii remote to exit." << std::endl;
    // std::cout << std::flush;

    while (any_wiimote_connected(&wm, MAX_WIIMOTES) && ros::ok())
    {
        if (wiiuse_poll(&wm, MAX_WIIMOTES) && wm->event == WIIUSE_EVENT)
        {
            if (IS_PRESSED(wm, WIIMOTE_BUTTON_A))
            {
                std_msgs::String str_msg;
                str_msg.data = "A";
                pub.publish(str_msg);
            }
            else if IS_PRESSED (wm, WIIMOTE_BUTTON_B)
            {
                std_msgs::String str_msg;
                str_msg.data = "B";
                pub.publish(str_msg);
            }
        }
    }

    wiiuse_cleanup(wiimotes, MAX_WIIMOTES);

    // printf("Done!\n");
    std::cout << "Done!" << std::endl;
    return EXIT_SUCCESS;
}

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