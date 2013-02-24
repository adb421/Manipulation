#include "CameraReadTest.h"

//Callback function for pcap_loop
void got_packet(u_char *args, const struct pcap_pkthdr *header, const u_char *packet);

float UDPToFloat(u_char *data);
void undist(double *xPt, double *yPt);

_uint64 preLoop;

FILE *filep;

int count = 0;
long long areaSum = 0;

int main(int argc, char *argv[]) {
	printf("Welcome to the QNX Momentics IDE\n");

	//Set tick size faster
	struct _clockperiod new;
	new.nsec = 20*1000; //20us in ns
	new.fract = 0;
	if(ClockPeriod(CLOCK_REALTIME, &new, NULL, 0) == -1)
		printf("Clock period change fail\n");

//	filep = fopen("/pc104CameraRead.txt","w");
//	filep = fopen("/pc104Calibration2.txt","a");
	filep = fopen("/pc104Calibration4.txt","a");

	if(filep == NULL) {
		printf("Couldn't open file\n");
		return EXIT_FAILURE;
	}

	//Make priority higher
	struct sched_param params;
	params.sched_priority = 50;
	pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);

	//Variable declarations
	struct in_addr addr; //Address struct, used to get things in human readable mode
	char *dev;			 //Contains device name
	char errbuf[PCAP_ERRBUF_SIZE]; //Error buffer for pcap functions
	char *net, *mask;    //Mask and net names
	pcap_t *handle;      //Session handle for pcap
	struct bpf_program fp; //Filter string for BPFs
	char filter_exp[] = FILTER;
	bpf_u_int32 maskp;   //BPF Netmask of sniffing device
	bpf_u_int32 netp;    //BPF IP of sniffing device
	unsigned int immediate_on = 1; //Used for setting the session into immediate mode

	//Let pcap get our device
	dev = pcap_lookupdev(errbuf);
	if(dev == NULL) {
		//Ruh roh, didn't get device
		printf("Couldn't find default device: %s\n", errbuf);
		return EXIT_FAILURE;
	}
	//Hooray! Got a device! Print it so we can see.
	printf("Dev: %s\n", dev);

	//Get netmask
	if(pcap_lookupnet(dev, &netp, &maskp, errbuf) == -1) {
		printf("Can't get netmask for device %s\n", dev);
		net = 0;
		mask = 0;
	}

	//Useaddr struct to get network address in human readable form
	addr.s_addr = netp;
	net = inet_ntoa(addr);
	if(net == NULL) {
		printf("inet_ntoa error\n");
		return EXIT_FAILURE;
	}
	printf("NET: %s\n", net);
	//Same for mask
	addr.s_addr = maskp;
	mask = inet_ntoa(addr);
	if(mask == NULL) {
		printf("inet_ntoa error on mask\n");
		return EXIT_FAILURE;
	}
	printf("MASK: %s\n", mask);

	//Open a sniffing session
	//Structure: pcap_open_live(char *device, int snaplen, int promisc, int to_ms, char *ebuf);
	handle = pcap_open_live(dev, BUFSIZ, 1, 0, errbuf);
	if(handle == NULL) {
		printf("Couldn't open device %s: %s\n", dev, errbuf);
		return EXIT_FAILURE;
	}

	//Compile and set the filter. BPF takes the string and converts it into something usable.
	if(pcap_compile(handle, &fp, filter_exp, 0, netp) == -1) {
		printf("Couldn't parse filter %s: %s\n", filter_exp, errbuf);
		return EXIT_FAILURE;
	}
	if(pcap_setfilter(handle, &fp) == -1) {
		printf("Couldn't install filter %s: %s\n", filter_exp, errbuf);
		return EXIT_FAILURE;
	}

	//Set the device into immediate mode
	if(ioctl(pcap_fileno(handle), BIOCIMMEDIATE, &immediate_on) < 0) {
		printf("Cannot enable immediate mode! Things will be slow\n");
	}

	//Start the loop!
	//Structure: pcap_loop(pcap_t *p, int cnt, pcap_handler callback, u_char *user);
	printf("Start loop\n");
//	fprintf(filep,"X Y Roundness Timing\n");
	ClockTime(CLOCK_REALTIME, NULL, &preLoop);
	//pcap_loop(handle, NUM_SAMPLES + 1, got_packet, (u_char *) filep);
	pcap_loop(handle, 0, got_packet, (u_char *) handle);
	fclose(filep);
	printf("Done!\n");
	return EXIT_SUCCESS;
}

void got_packet(u_char *args, const struct pcap_pkthdr *header, const u_char *packet) {

	const struct sniff_ethernet *ethernet;	//Ethernet header
	const struct sniff_ip *ip;				//The IP header
	const struct sniff_udp *udp;				//The UDP header
	const char *payload;						//Packet payload (actual data)
	char *currentObject;
	u_int size_ip;
	u_int size_payload;
	_uint64 postLoop;

	//FILE *filep = (FILE *) args;

	ClockTime(CLOCK_REALTIME, NULL, &postLoop);

	//Camera specific variables
	u_char *xChar, *yChar, *roundChar, *areaChar;
	float x,y,round;
	double xSum, ySum;
	int areaInt;

	//Got a packet!
	if(count == 0) {
		//printf("Got first packet!\n");
		ClockTime(CLOCK_REALTIME, NULL, &preLoop);
		count++;
		return;
	}

	//Cool typecasting!
	//Ethernet header takes up the first section of the packet
	ethernet = (struct sniff_ethernet*)(packet);
	//Next comes IP header
	ip = (struct sniff_ip*)(packet + SIZE_ETHERNET);
	size_ip = IP_HL(ip)*4;
	if(size_ip < 20) {
		printf("    * Invalid IP header length: %u bytes\n", size_ip);
		return;
	}
	//Now UDP header
	udp = (struct sniff_udp*)(packet + SIZE_ETHERNET + size_ip);
	payload = (u_char *)(packet + SIZE_ETHERNET + size_ip + SIZE_UDP);

	//Now, we know that the udp packet header is 8 bytes, and the length includes both.
	//Therefore, number of bytes ifs length - 8. Note that we also need to make sure we have the right byte order!
	size_payload = (int)(ntohs(udp->udp_len)) - SIZE_UDP;
	if((size_payload - 24)%20 != 0) {
		printf("Payload is weird size: %d\n", size_payload);
	}

	//Check how many bytes we got!
	int nObjects = (size_payload - 24)/20;
	printf("%d\n",nObjects);
#define desObj 30
	if(nObjects == desObj) {
		printf("See %d\n", desObj);
		int i;
		for(i = 0; i < nObjects; i++) {
			currentObject = payload + 24 + 20*i;
			xChar = currentObject;
			yChar = currentObject + 4;
			x = *((float *)xChar);
			y = *((float *)yChar);
			//x = UDPToFloat(xChar);
			//y = UDPToFloat(yChar);
			fprintf(filep,"%f %f\n",x,y);
		}
		printf("Wrote %d\n",desObj);
		pcap_breakloop((pcap_t *) args);
	}
//	if(nObjects > 0) {
//		xChar = payload + 24;
////		yChar = payload + 24 + 4;
//		yChar = xChar + 4;
////		roundChar = payload + 24 + 4 + 4;
//		roundChar = yChar + 4;
////		areaChar = payload + 24 + 4 + 4 + 4;
//		areaChar = roundChar + 4;
//		x = UDPToFloat(xChar);
//		y = UDPToFloat(yChar);
//		round = UDPToFloat(roundChar);
//		areaInt = *(int *)(areaChar);
//		areaSum += areaInt;
//		xSum += (double)x;
//		ySum += (double)y;
//
//		//printf("X: %f, Y: %f, round: %f\n", x,y,round);
////		printf("round: %x\n", *((unsigned int*)&round));
////		printf("round: %x\n", *((unsigned int*)roundChar));
//		fprintf(filep,"%f %f %f %f\n", x, y, round, (postLoop - preLoop)/1000000.0);
//		count++;
//		if(count == 250) {
//			printf("Average int: %d, average adjusted float: %f\n",(int)(areaSum/250), areaSum/250.0/254.0);
//			xSum = xSum/250.0;
//			ySum = ySum/250.0;
//			undist(&xSum, &ySum);
////			printf("X: %f, Y: %f\n", xSum/250.0, ySum/250.0);
//			printf("X: %f, Y: %f\n", xSum, ySum);
//			xSum = 0;
//			ySum = 0;
//			areaSum = 0;
//			count = 0;
//		}
//	} else {
//		printf("No objects!\n");
//	}
	ClockTime(CLOCK_REALTIME, NULL, &preLoop);

}

float UDPToFloat(u_char *data) {
//	float *returnVal;
//	char tempChar[4];
//	int i;
//	for(i = 0; i < 4; i++) {
//		tempChar[i] = data[3-i];
//	}
//	returnVal = (float *)tempChar;
//	return *returnVal;

	//Try this!
//	uint32_t *temp = (uint32_t *) data;
//	uint32_t temp2 = ntohl(*temp);
//	return *((float *)(&temp2));
	float *returnVal;
	returnVal = (float *)data;
	return *returnVal;
}

void undist(double *xPt, double* yPt) {
	double tempXundis, tempYundis, tempX, tempY;
	tempX = *xPt;
	tempY = *yPt;
	printf("tempX: %f, tempY: %f\n",tempX,tempY);
//	double r_sqr = (tempX - 416)*(tempX - 416) + (tempY - 416)*(tempY - 416);
//	tempXundis = (tempX - 416)*(1 + 0.120669*r_sqr - 0.150538*r_sqr*r_sqr + 0.0049567*r_sqr*r_sqr*r_sqr);
//	tempYundis = (tempY - 416)*(1 + 0.120669*r_sqr - 0.150538*r_sqr*r_sqr + 0.0049567*r_sqr*r_sqr*r_sqr);
	tempXundis = (tempX - 416)/776;
	tempYundis = (tempY - 416)/776;
	*xPt = tempXundis;
	*yPt = tempYundis;
	printf("Xundis: %f, Yundis: %f\n",tempXundis, tempYundis);
}






