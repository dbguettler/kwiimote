/* Raw TP-Link (Kasa) command sender/receiver
 * Each invocation sends a single command and receives a single packet in response.
 *
 * This program is a minimal substitute for the tplink-lightbulb Node package.
 * Because it's C, it's about 40x faster than the Node package.  On a Raspberry
 * Pi, that matters.
 *
 * Building:
 *   make
 *
 * Usage:
 *   ./kasa <ip-address> <json-blob>
 *
 * There's a good list of JSON blobs to try here:
 *   https://github.com/softScheck/tplink-smartplug/blob/master/tplink-smarthome-commands.txt
 * Especially:
 *   - get bulb info: ./kasa <ip> '{"system":{"get_sysinfo":null}}'
 *   - turn bulb on:  ./kasa <ip> '{"system":{"set_relay_state":{"state":1}}}'
 *   - turn bulb off: ./kasa <ip> '{"system":{"set_relay_state":{"state":0}}}'
 *
 * Written by Patrick Reynolds <dukepiki@gmail.com>
 * Released into the public domain, or Creative Commons CC0, your choice.
 */

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#define KASA_PORT 9999
#define IP_ADDR "192.168.68.91"

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

void command_callback(const std_msgs::String::ConstPtr &msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kwii_command_node");
	ros::NodeHandle handle;

	ros::Subscriber sub = handle.subscribe("commands", 100, command_callback);

	ros::spin();

	return EXIT_SUCCESS;
}

void command_callback(const std_msgs::String::ConstPtr &msg)
{
	struct in_addr addr;
	if (!inet_aton(IP_ADDR, &addr))
	{
		fprintf(stderr, "Could not parse \"%s\" as an IP\n", IP_ADDR);
		exit(1);
	}

	uint8_t *enc = kasa_encrypt((const uint8_t *)msg->data.c_str(), strlen(msg->data.c_str()));

	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
	if (sock == -1)
	{
		perror("socket");
		exit(1);
	}

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
	res = sendto(sock, enc, strlen(msg->data.c_str()), 0, (struct sockaddr *)&sin, sizeof(sin));
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
		perror("recvfrom");
		exit(1);
	}

	uint8_t *dec = kasa_decrypt((const uint8_t *)buf, res);
	fwrite((const char *)dec, 1, res, stdout);
	putchar('\n');
	free(dec);
	// std::cout << std::flush;
}
