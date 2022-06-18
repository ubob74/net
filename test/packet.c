#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <net/ethernet.h> 
#include <netinet/in.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>

static int get_sock(const char *ifname)
{
	int psock = -1;
	struct sockaddr_ll s_ll;
	struct ifreq ifr;

	psock = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
	if (psock < 0) {
		perror("socket");
		return -1;
	}

	memset((void *)&s_ll, 0, sizeof(struct sockaddr_ll));

	sprintf(ifr.ifr_name, "%s", ifname);

	if (ioctl(psock, SIOCGIFINDEX, &ifr) < 0) {
		perror("ioctl SIOCGIFINDEX");
		goto err;
	}

	s_ll.sll_family = PF_PACKET;
	s_ll.sll_protocol = htons(ETH_P_ALL);
	s_ll.sll_ifindex = ifr.ifr_ifindex;

	if (bind(psock, (struct sockaddr *)&s_ll, sizeof(struct sockaddr_ll)) < 0) {
		perror("bind");
		goto err;
	}

	return psock;
err:
	close(psock);
	return -1;
}

int main(void)
{
	int i;
	int psock;
	const char *ifname = "enp4s0";
	unsigned char buf[32];

	memset(buf, 0, sizeof(buf));
	for (i = 0; i < ETH_ALEN; i++)
		buf[i] = 0xFF;
	for (; i < sizeof(buf); i++)
		buf[i] = 0xAC;

	psock = get_sock(ifname);
	if (psock < 0)
		goto out;

	for (;;sleep(2), sendto(psock, buf, sizeof(buf), 0, NULL, 0));

	close(psock);

out:
	return 0;
}
