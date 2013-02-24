#include "includes.h"

_uint64 preLoop;

FILE *filep;

void * vision_loop_thread(void *arg) {
  //Variable declarations
  struct in_addr addr; //Address struct, used to get things in human readable mode
  char *dev;			 //Contains device name10001
  char errbuf[PCAP_ERRBUF_SIZE]; //Error buffer for pcap functions
  char *net, *mask;    //Mask and net names
  pcap_t *handle;      //Session handle for pcap
  struct bpf_program fp; //Filter string for BPFs
  char filter_exp[] = FILTER;
  bpf_u_int32 maskp;   //BPF Netmask of sniffing device
  bpf_u_int32 netp;    //BPF IP of sniffing device
  unsigned int immediate_on = 1; //Used for setting the session into immediate mode

  newCameraData = 0;
  visionCount = 0;

  //Let pcap get our device
  dev = pcap_lookupdev(errbuf);
  if(dev == NULL) {
    //Ruh roh, didn't get device
    printf("Couldn't find default device: %s\n", errbuf);
    return EXIT_FAILURE;
  }
  //Hooray! Got a device! Print it so we can see.
  //printf("Dev: %s\n", dev);

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
  //printf("NET: %s\n", net);
  //Same for mask
  addr.s_addr = maskp;
  mask = inet_ntoa(addr);
  if(mask == NULL) {
    printf("inet_ntoa error on mask\n");
    return EXIT_FAILURE;
  }
  //printf("MASK: %s\n", mask);
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
  printf("Start receive loop\n");
  //	fprintf(filep,"X Y Roundness Timing\n");
  ClockTime(CLOCK_REALTIME, NULL, &preLoop);
  //pcap_loop(handle, NUM_SAMPLES + 1, got_packet, (u_char *) filep);
  pcap_loop(handle, 0, got_Packet, (u_char *) handle);
  fclose(filep);
  printf("Done!\n");
  return EXIT_SUCCESS;
}

void convertToWorldFrame(float *x, float *y) {
  const float H_mat[8] = H_mat_vals;
  float xLambda = (*x)*H_mat[0] + (*y)*H_mat[1] + H_mat[2];
  float yLambda = (*x)*H_mat[3] + (*y)*H_mat[4] + H_mat[5];
  float lambda  = (*x)*H_mat[6] + (*y)*H_mat[7] + 1.0;
  *x = xLambda/lambda + X_OFFSET;
  *y = yLambda/lambda + Y_OFFSET;
	
}

void got_Packet(u_char *args, const struct pcap_pkthdr *header, const u_char *packet) {
  const struct sniff_ethernet *ethernet;	//Ethernet header
  const struct sniff_ip *ip;				//The IP header
  const struct sniff_udp *udp;				//The UDP header
  const char *payload;					//Packet payload (actual data)
  char *currentObject;
  u_int size_ip;
  u_int size_payload;
  _uint64 postLoop;

  //FILE *filep = (FILE *) args;

  ClockTime(CLOCK_REALTIME, NULL, &postLoop);
  /* if(visionCount < NUM_SAMPLES) { */
  /* 	//put it in ms, do it for 10k why not */
  /* 	cameraTiming[visionCount] = ((float)(postLoop-preLoop))/1000000.0; */
  /* 	visionCount++; */
  /* } */

  //Camera specific variables
  u_char *xChar, *yChar, *roundChar, *areaChar;
  float x,y,round;
  double xSum, ySum;
  int areaInt;

  //Got a packet!
  /* if(visionCount == 0) { */
  /* 	//printf("Got first packet!\n"); */
  /* 	ClockTime(CLOCK_REALTIME, NULL, &preLoop); */
  /* 	visionCount++; */
  /* 	return; */
  /* } */

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
  // printf("%d\n",nObjects);
  if(nObjects == desObj) {
    //printf("See %d\n", desObj);
    int i;
    for(i = 0; i < nObjects; i++) {
      currentObject = payload + 24 + 20*i;
      xChar = currentObject;
      yChar = currentObject + 4;
      roundChar = yChar + 4;
      areaChar = roundChar + 4;
      x = *((float *)xChar);
      y = *((float *)yChar);
      round = *((float *)roundChar);
      areaInt = *((int *)areaChar);
      convertToWorldFrame(&x, &y);
      xGlobal[i] = x;
      yGlobal[i] = y;
      areaGlobal[i] = areaInt;
    }
    sortByArea(xGlobal, yGlobal, areaGlobal, nObjects);
    newCameraData = 1;
    //printf("Wrote %d\n",desObj);
    //pcap_breakloop((pcap_t *) args);
  }
  else {
    printf("%d Objects\n",nObjects);
  }
  ClockTime(CLOCK_REALTIME, NULL, &preLoop);
}

void sortByArea(float *x, float *y, int* area, int nObjects) {
  //Implement via bubblesort
  int swapped = 1;
  int i;
  int areaTemp;
  float xTemp, yTemp;
  while(swapped) {
    swapped = 0;
    for(i = 0; i < nObjects - 1; i++) {
      if(area[i] < area[i+1]) {
	areaTemp = area[i];
	xTemp = x[i];
	yTemp = y[i];
	//Now swap
	area[i] = area[i+1];
	x[i] = x[i+1];
	y[i] = y[i+1];
	area[i+1] = areaTemp;
	x[i+1] = xTemp;
	y[i+1] = yTemp;
	swapped = 1;
      }
    }
    nObjects -= 1;
  }
}
