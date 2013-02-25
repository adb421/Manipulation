/*
 * tcpip_com_send.h
 *
 *  Created on: Oct 25, 2012
 *      Author: Adam
 */

#ifndef TCPIP_COM_SEND_H_
#define TCPIP_COM_SEND_H_

#include "includes.h"
#include <semaphore.h>

#define NUM_CHANNELS_SEND	4
#define SEND_PORT			3100
#define TASK_PULSE_CODE		_PULSE_CODE_MINAVAIL

void * tcpip_com_send_thread(void *arg);
int TriggerWait_send();
int Trigger_send(int value);
int FifoQGet(void *obj);
int FifoQPut(void *obj);
void FifoQReset();
int FifoQRead(void *obj, int index);
void FifoQInit(int objSize, int len);
void AddSignal_send(int ch, double *val);
void Process_send();
void sendString(const char *stringToSend);

int mSocket_send;
int mSessionSocket_send;
int mInitialized_send;
struct sockaddr_in mServerAddr_send;
int mChannelId_send;
int mConnectId_send;
int bytes;
int divisorCount_send;
unsigned char validSession_send;
double mpValBuf_send[NUM_CHANNELS_SEND];
double *mpValPtrArr_send[NUM_CHANNELS_SEND];
double mSampleRate_send;
unsigned char mBuf_send[NUM_CHANNELS_SEND*8];

struct FifoQStruct{
	sem_t mSemaphore;
	int mHead;
	int mTail;
	int mSize;
	int mObjSize;
	int mLength;
	int mIndex;
	char *mFifoPtr;
} FifoQ;
#endif /* TCPIP_COM_H_ */
