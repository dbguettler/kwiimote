# KWiimote README
This project allows the user to control TP-Link Kasa smart plugs and bulbs using a Wii remote. 

Supported devices:
- KL130
- HS103
- HS105

It is possible that other devices are supported, but I have not had the opportunity to test any other devices.

## Acknowledgments
The code used in this project is largely reliant on various other works:

- [WiiUse](https://github.com/wiiuse/wiiuse): Used for interfacing with the Wii remote.
- [python-kasa](https://github.com/python-kasa/python-kasa): Used for device discovery.
- https://github.com/piki/kasa: Used to send commands to the smart devices.
- [Docker](https://www.docker.com): For easy building and running
- [ROS](https://www.ros.org): ROS Noetic is used to facilitate communication between the C++ and Python components

Without each of these, I would not be able to create this.

## Compiling
To my knowledge, this project will only work on Linux. I have not tried it on MacOS, but I know for sure that it does not work on Windows. The platform I test and run on
is a Raspberry Pi 4B running Raspberry Pi OS with no desktop environment (Debian Buster based).

You will need to have Docker/Docker Compose installed to compile and run. While the instructions are for the Raspberry Pi, I had success using 
[these instructions](https://pimylifeup.com/raspberry-pi-docker/).

The TL;DR for installing Docker is:

```bash
# Install Docker
curl -sSL https://get.docker.com | sh
sudo usermod -aG docker <your_username>
# Reboot your system
sudo reboot
# Check that you are in the docker group
groups
```

Then, to build the image, run:
```bash
docker compose build
```

## Running
Before running, you will need to specify the devices to control. In the project root, there is a file called `.env.example`. Make a copy of this file with the name `.env`,
then replace the names in quotation marks with the devices you want to control. If you only want to control one device, remove the line containing `DEV_2` or change the 
value to an empty string.

Then, run the program with:

```bash
docker compose up
```

When the following message is printed, either press the sync button on the Wii remote, or simultaneously press 1 and 2 to pair:

```
==================================================================
==================================================================
Press SYNC or (1) and (2) simultaneously to pair the Wii remote...
==================================================================
==================================================================
```

The Wii remote will vibrate briefly when it is connected. The selected device will initially be `DEV_1`.

## Controls
| Button  | Action |
| ------------- | ------------- |
| A | Turn on the selected device |
| B | Turn off the selected device |
| 1 | Select the first device (`DEV_1`) |
| 2 | Select the second device (`DEV_2`) |

| LED  | Meaning |
| ------------- | ------------- |
| 1 | Indicates connection to the system |
| 2 | Indicates the on/off status of the selected device |
| 3 | Indicates that the first device is selected |
| 4 | Indicates that the second device is selected |

## Future Improvements
- [ ] Add controls for smart bulb brightness
- [ ] Improve controls when using only a single device
- [ ] Possibly add support for controlling more than 2 devices
