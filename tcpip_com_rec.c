/*
 * tcpip_com_rec.c
 *
 *  Created on: Oct 29, 2012
 *      Author: Adam
 */

#include "tcpip_com_rec.h"
static void sig_handler_rec(int signo){};

void * tcpip_com_rec_thread(void *arg) {
	mInitialized_rec = 1;
	waiting_for_cmd = 1;
//	sem_init(&rec_sem, 0,
	//Create the socket
	mSocket_rec = socket(AF_INET, SOCK_STREAM, 0);
	if(mSocket_rec <0) {
		printf("Error opening rec socket\n");
		return (void*)-1;
	}

	int opt = 1;
	if(setsockopt(mSocket_rec, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(int)) < 0) {
		printf("Error setting rec socket option\n");
		return (void*)-1;
	}

	mServerAddr_rec.sin_family = AF_INET;
	mServerAddr_rec.sin_addr.s_addr = INADDR_ANY;
	mServerAddr_rec.sin_port = htons(REC_PORT);
	memset(&(mServerAddr_rec.sin_zero),'\0', 8);
	//Bind the socket
	if(bind(mSocket_rec, (struct sockaddr *)&mServerAddr_rec, sizeof(struct sockaddr)) <0) {
		printf("Error binding rec socket\n");
		return (void*)-1;
	}

	//Listen
	if(listen(mSocket_rec,2) == -1) {
		printf("Listen failed rec\n");
		return (void*)-1;
	}

	//Don't allow exiting upon client disconnect
	struct sigaction act_rec;

	act_rec.sa_handler = &sig_handler_rec;
	act_rec.sa_flags = 0;
	sigaction(SIGPIPE, &act_rec, NULL);

	//Done initializing
	//int bytes;
	double storage;

	while(1) {
		validSession_rec = 0;
		//ACcept request
		mSessionSocket_rec = accept(mSocket_rec, 0, 0);
		//If we haven't accepted, redo the loop
		if(mSessionSocket_rec == -1) {
			continue;
		}

		printf("Receive thread accepted connection\n");
		validSession_rec = 1;
		while(1){
			//Use recv() with MSG_PEEK, so that we don't count them as read. If we get one, go to the process command thing
			recv(mSessionSocket_rec, &storage, sizeof(double), MSG_PEEK);
			if(waiting_for_cmd) {
				waiting_for_cmd = 0;
				process_cmd();
			} else {
				continue;
			}

		}
	}
}

void readInt(int* val) {
	if(validSession_rec) {
		recv(mSessionSocket_rec, val, sizeof(int), 0);
		//printf("Received: %d\n", *val);
	}
	else {
		*val = -1;
	}
}

void readDouble(double * val) {
	if(validSession_rec) {
		recv(mSessionSocket_rec, val, sizeof(double), 0);
	} else {
		*val = -1;
	}
}
