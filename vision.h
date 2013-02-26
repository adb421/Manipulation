/*
 * vision.h
 *
 *  Created on: Feb 6, 2013
 *      Author: Adam
 */

#ifndef VISION_H_
#define VISION_H_

#include "includes.h"

#define FILTER "udp and dst host 192.168.1.101 and (len <= 73 || len >= 75)"

#define ETHER_ADDR_LEN 6

#define NUM_SAMPLES 10001

#define SIZE_ETHERNET 14 //Ethernet packet headers are always 14 bytes
#define SIZE_UDP	  8  //UDP packet headers are always 8 bytes

#define desObj 3
/* #define X_OFFSET (-0.804427) */
/* #define Y_OFFSET (-1.07845) */
#define X_OFFSET 0.0
#define Y_OFFSET 0.0

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

#define H_mat_vals {0.002195474896537, -0.00006412979676103758, -0.669754875953130, -0.000006920680662281351, 0.002230533548092, -0.045558655093215, -0.00005754571383216432, 0.00001007534286376455};

//Global variables
float xGlobal[desObj];
float yGlobal[desObj];
int areaGlobal[desObj];
int newCameraData;
int visionCount;



//Thread function call
void * vision_loop_thread(void *arg);

//Callback function for pcap_loop
void got_Packet(u_char *args, const struct pcap_pkthdr *header, const u_char *packet);

//Function converts x and y to world frames using H transform above
void convertToWorldFrame(float *x, float *y);

//Function sorts the objects in descending order by size of marker
void sortByArea(float *x, float *y, int* area, int nObjects);
float cameraTiming[NUM_SAMPLES];

//Calculates joint angles for 1 and 2 based on camera marker locations
double calculateJointOneCamera();
double calculateJointTwoCamera();

#endif /* VISION_H_ */
