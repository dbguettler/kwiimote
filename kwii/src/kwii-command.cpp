/* TP-Link (Kasa) command sender/receiver
 * Each invocation sends a single command and receives a single packet in response.
 *
 * Adapted from code by Patrick Reynolds <dukepiki@gmail.com> at (https://github.com/piki/kasa)
 * with just a few modification
 */

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <errno.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "kwii/DevCmd.h"

#define KASA_PORT 9999

extern int errno;

uint8_t *kasa_crypto(const uint8_t *p, int len, int enc)
{
	uint8_t *ret = (uint8_t *)malloc(len + 1);
	ret[len] = '\0';
	uint8_t k = 0xab;
	int i;
	for (i = 0; i < len; i++)
	{
		ret[i] = (uint8_t)(k ^ p[i]);
		k = enc ? ret[i] : p[i];
	}
	return ret;
}

uint8_t *kasa_encrypt(const uint8_t *p, int len)
{
	return kasa_crypto(p, len, 1);
}

uint8_t *kasa_decrypt(const uint8_t *p, int len)
{
	return kasa_crypto(p, len, 0);
}

void command_callback(const kwii::DevCmd::ConstPtr &msg);

int main(int argc, char **argv)
{
	// Initialize ROS node with name "kwii_command_node"
	ros::init(argc, argv, "kwii_command_node");
	ros::NodeHandle handle;

	// Subscribe to the "/commands topic"
	ros::Subscriber sub = handle.subscribe("commands", 100, command_callback);

	ros::spin();

	return EXIT_SUCCESS;
}

/**
 * Executed whenever a command is received from the "/commands" topic
 *
 * @param msg DevCmd ROS message containing target IP and command string
 */
void command_callback(const kwii::DevCmd::ConstPtr &msg)
{
	struct in_addr addr;
	if (!inet_aton(msg->ip.c_str(), &addr))
	{
		fprintf(stderr, "Could not parse \"%s\" as an IP\n", msg->ip.c_str());
		exit(1);
	}

	uint8_t *enc = kasa_encrypt((const uint8_t *)msg->cmd.c_str(), strlen(msg->cmd.c_str()));

	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
	if (sock == -1)
	{
		perror("socket");
		exit(1);
	}

	// Set receive timeout of 1 sec
	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

	struct sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_port = htons(0);
	sin.sin_addr.s_addr = htonl(INADDR_ANY);
	int res = bind(sock, (struct sockaddr *)&sin, sizeof(sin));
	if (res == -1)
	{
		perror("bind");
		exit(1);
	}

	sin.sin_family = AF_INET;
	sin.sin_port = htons(KASA_PORT);
	sin.sin_addr = addr;
	res = sendto(sock, enc, strlen(msg->cmd.c_str()), 0, (struct sockaddr *)&sin, sizeof(sin));
	if (res == -1)
	{
		perror("sendto");
		exit(1);
	}
	free(enc);

	char buf[4096];
	socklen_t slen = sizeof(sin);
	res = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&sin, &slen);
	if (res == -1)
	{
		// Receive failed, likely due to a timeout
		perror("recvfrom");
		if (errno == EAGAIN)
		{
			// If error is "try again", reset the error and move on to the next message.
			errno = 0;
			return;
		}
		else
		{
			exit(1);
		}
	}

	uint8_t *dec = kasa_decrypt((const uint8_t *)buf, res);
	fwrite((const char *)dec, 1, res, stdout);
	putchar('\n');
	free(dec);
}
