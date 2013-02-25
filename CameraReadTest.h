/*
 * CameraReadTest.h
 *
 *  Created on: Feb 6, 2013
 *      Author: Adam
 */

#ifndef CAMERAREADTEST_H_
#define CAMERAREADTEST_H_

#include <stdlib.h>
//#include <stdio.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
#include <arpa/inet.h>
//#include <fcntl.h>
//#include <netdb.h>
//#include <string.h>
#include <sys/neutrino.h>
//#include <pthread.h>
#include <sys/ioctl.h>
#include <pcap.h>
//#include <netinet/if_ether.h>
//#include <packet.h>
//#include <net/if.h>
//#include <net/ethernet.h>
#include <net/route.h>
#include <math.h>

#define FILTER "udp and dst host 192.168.1.101 and (len <= 73 || len >= 75)"

#define ETHER_ADDR_LEN 6

#define NUM_SAMPLES 10000

#define SIZE_ETHERNET 14 //Ethernet packet headers are always 14 bytes
#define SIZE_UDP	  8  //UDP packet headers are always 8 bytes

//Ethernet header
struct sniff_ethernet {
	u_char ether_dhost[ETHER_ADDR_LEN]; //Destination host address
	u_char ether_shost[ETHER_ADDR_LEN]; //Source host address
	u_short ether_type; //IP? ARP? RARP? etc
};

//IP Header
struct sniff_ip {
	u_char ip_vhl;		//Version << 4 | header length >> 2
	u_char ip_tos;		//Type of service
	u_short ip_len; 	//Total length
	u_short ip_id;		//Identification
	u_short ip_off; 	//Fragment offset field
#define IP_RF 0x8000	//Reserved fragment flag
#define IP_DF 0x4000	//Don't fragment flag
#define IP_MF 0x2000	//More fragments flag
#define IP_OFFMASK 0x1fff //Mask for fragmenting bits
	u_char ip_ttl;		//Time to live
	u_char ip_p;		//protocol
	u_short ip_sum;		//Checksum
	struct in_addr ip_src,ip_dst; //Source and dest address
};

#define IP_HL(ip)		(((ip)->ip_vhl) & 0x0f)
#define IP_V(ip)		(((ip)->ip_vhl) >> 4)

//UDP header
struct sniff_udp {
	u_short udp_sport;		//Source port
	u_short udp_dport;		//Destination port
	u_short udp_len;		//Packet length (header + data)
	u_short udp_sum;		//Checksum
};

#endif /* CAMERAREADTEST_H_ */
