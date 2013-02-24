/*
 * tcpip_com_rec.h
 *
 *  Created on: Oct 29, 2012
 *      Author: Adam
 */

#ifndef TCPIP_COM_REC_H_
#define TCPIP_COM_REC_H_

#include "includes.h"
#include <semaphore.h>

#define REC_PORT 3490 //TCP/IP Port

void * tcpip_com_rec_thread(void *arg);

void readDouble(double * val);
void readInt(int * val);

int mBuf_rec[4];
int mInitialized_rec;
int mSocket_rec;
int mSessionSocket_rec;
int validSession_rec;
struct sockaddr_in mServerAddr_rec;
int waiting_for_cmd;

sem_t rec_sem;

#endif /* TCPIP_COM_REC_H_ */
