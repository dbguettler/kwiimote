#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiiuse.h>

#define MAX_WIIMOTES 1

short any_wiimote_connected(wiimote** wm, int wiimotes);

int main()
{
    wiimote **wiimotes;
    wiimote *wm;
    int found, connected;

    // Init wiimote array
    wiimotes = wiiuse_init(MAX_WIIMOTES);
    wm = wiimotes[0];

    // Find wiimotes to use
    found = wiiuse_find(&wm, MAX_WIIMOTES, 5);
    if (!found)
    {
        printf("No wiimotes found.\n");
        wiiuse_cleanup(wiimotes, MAX_WIIMOTES);
        return 0;
    }

    // Connect to the wiimotes
    connected = wiiuse_connect(&wm, MAX_WIIMOTES);
    if (connected)
    {
        printf("Connected to %i wiimotes (of %i found).\n", connected, found);
    }
    else
    {
        printf("Failed to connect to any wiimote.\n");
        wiiuse_cleanup(wiimotes, MAX_WIIMOTES);
        return 0;
    }

    // Turn on P1/P4 LED and rumble briefly to indicate connection success
    wiiuse_set_leds(wm, WIIMOTE_LED_1 | WIIMOTE_LED_4);
    wiiuse_rumble(wm, 1);
    usleep(200000);
    wiiuse_rumble(wm, 0);

    printf("\n\nPress 'A' on the Wii remote to exit...\n");
    fflush(stdout);

    while (any_wiimote_connected(&wm, MAX_WIIMOTES))
    {
        if (wiiuse_poll(&wm, MAX_WIIMOTES) && wm->event == WIIUSE_EVENT && IS_PRESSED(wm, WIIMOTE_BUTTON_A))
        {
            break;
        }
    }

    wiiuse_cleanup(wiimotes, MAX_WIIMOTES);

    printf("Done!\n");
    return EXIT_SUCCESS;
}

short any_wiimote_connected(wiimote** wm, int wiimotes) {
	int i;
	if (!wm) {
		return 0;
	}

	for (i = 0; i < wiimotes; i++) {
		if (wm[i] && WIIMOTE_IS_CONNECTED(wm[i])) {
			return 1;
		}
	}

	return 0;
}