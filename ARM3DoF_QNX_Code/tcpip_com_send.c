#include "tcpip_com_send.h"
static void sig_handler_send(int signo){};

void * tcpip_com_send_thread(void *arg) {

//	FifoQInit(sizeof(double) * NUM_CHANNELS_SEND,3);
	mSocket_send = socket(AF_INET, SOCK_STREAM, 0);
	if(mSocket_send < 0) {
		printf("Error opening socket\n");
		return (void *) -1;
	}
	int opt = 1;
	if(setsockopt(mSocket_send, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(int)) < 0) {
		printf("error setting socket option for SO_REUSEADDR using SOL_SOCKET\n");
		return (void *) -1;
	}

	mServerAddr_send.sin_family = AF_INET;
	mServerAddr_send.sin_addr.s_addr = INADDR_ANY;
	mServerAddr_send.sin_port = htons(SEND_PORT);
	memset(&(mServerAddr_send.sin_zero),'\0',8);

	if(bind(mSocket_send, (struct sockaddr *)&mServerAddr_send, sizeof(struct sockaddr)) < 0) {
		printf("Error binding socket\n");
		return (void *) -1;
	}

	if(listen(mSocket_send, 2) == -1) {
		printf("listen failed\n");
		return (void *) -1;
	}
	//Register an empty signal handler for SIGPIPE
	// Prevents exiting upon client disconnect
	struct sigaction act_send;

	act_send.sa_handler = &sig_handler_send;
	act_send.sa_flags = 0;
	sigaction(SIGPIPE, &act_send, NULL);
	//AperiodicTask::Init("MatlabNet Task", priority);

	//Create a pulse channel
	if((mChannelId_send = ChannelCreate(NULL)) == -1) {
		printf("ChannelCreate Failed");
		exit(1);
	}
	//Connect to it
	if((mConnectId_send = ConnectAttach(0, 0, mChannelId_send, _NTO_SIDE_CHANNEL,0)) == -1) {
		printf("ConnectAttach failed\n");
		exit(1);
	}
	//Using "task()"
	//printf("Starting tcpip loop 1\n");
	while(1){
		validSession_send = 0;
		mSessionSocket_send = accept(mSocket_send,0,0);
		if(mSessionSocket_send == -1){
			printf("Error Accepting Socket\n");
			continue;
		}
		else {
			opt = 1;
			if(setsockopt(mSessionSocket_send, IPPROTO_TCP, TCP_NODELAY, (char *)&opt, sizeof(int)) < 0) {
				printf("Error setting socket option for TCP_NODELAY using IPPROTO\n");
				close(mSessionSocket_send);
				continue;
			}
		}

		validSession_send = 1;
//		FifoQReset();
		//printf("Starting tcpip loop 2\n");
		while(1) {
//			// wait for trigger to send signals
//			if(TriggerWait_send() == -1){
//				continue;
//			}
			//Pull signals from buffer
//			if(FifoQGet(&mBuf_send[0])!=-1) {
//				bytes = 8*NUM_CHANNELS_SEND;
//				//Sending:
//				int ii,jj;
//				double *ptrSend;
//				char	charSends[8];
//				ii = 0;
//				while(ii < 8*NUM_CHANNELS_SEND) {
//					for(jj = ii; jj < ii + 8; jj++) {
//						charSends[jj-ii] = mBuf_send[jj];
//					}
//					ptrSend = (double *)charSends;
//					ii = jj;
//					//printf("Sending: %f\n", *ptrSend);
//					//printf("ii: %d\n", ii);
//				}
//				if(send(mSessionSocket_send, mBuf_send, bytes, 0) == -1) {
//					printf("Unable to send\n");
//					close(mSessionSocket_send);
//					break;
//				}
//			}
//			sched_yield();
			sleep(1);
		}
	}
	//Don't need to create a thread since we are in the thread we weant to be in!
}

//int TriggerWait_send(){
//	struct _pulse pulseMsg_send;
//
//	if(MsgReceivePulse(mChannelId_send, &pulseMsg_send, sizeof(pulseMsg_send), NULL) == -1) {
//		printf("MsgReceivePulse failed.\n");
//		exit(1);
//	}
//
//	if(pulseMsg_send.code == TASK_PULSE_CODE){
//		printf("TriggerWait got pulse\n");
//		return pulseMsg_send.value.sival_int;
//	}
//	else {
//		return -1;
//	}
//}

//int Trigger_send(int value) {
//	if(MsgSendPulse(mConnectId_send, 60, TASK_PULSE_CODE, value) == -1) {
//		printf("MsgSendPulse failed.\n");
//		exit(1);
//	}
//	return 1;
//}

//void Process_send(){
//	//printf("Process called\n");
//	if(validSession_send){
//		//divisorCount = 0;
//		//Copy values into buffer
//		int i;
//		for(i = 0; i< NUM_CHANNELS_SEND; i++) {
//			mpValBuf_send[i] = *mpValPtrArr_send[i];
//		}
//		FifoQPut(mpValBuf_send);
//		Trigger_send(0);
//		//printf("Triggered!\n");
//	}
//}

//void AddSignal_send(int ch, double *val){
//	if((ch <= NUM_CHANNELS_SEND -1)) {
//		//Record pointer in array
//		mpValPtrArr_send[ch] = val;
//	}
//}

//void FifoQInit(int objSize, int len){
//	FifoQ.mObjSize = objSize;
//	FifoQ.mLength = len;
//	FifoQ.mHead = FifoQ.mTail = FifoQ.mSize = 0;
//	FifoQ.mFifoPtr = (char *)malloc(len * objSize);
//	sem_init(&FifoQ.mSemaphore,1,1);
//}

//int FifoQPut(void *obj){
//	int ret = -1;
//
//	sem_wait(&FifoQ.mSemaphore);
//
//	if(FifoQ.mSize < FifoQ.mLength) {
//		//printf("Copying: %f", *((double *)obj));
//		//(void* st, const void* src, size_t length)
//		memcpy(FifoQ.mFifoPtr + FifoQ.mHead*FifoQ.mObjSize, obj, FifoQ.mObjSize);
//		//printf("Copied: %f\n", *((double *)(FifoQ.mFifoPtr + FifoQ.mHead*FifoQ.mObjSize)));
//		++FifoQ.mHead;
//		FifoQ.mHead %= FifoQ.mLength;
//		FifoQ.mSize++;
//		ret = 1;
//	}
//	else {
//		memcpy(FifoQ.mFifoPtr + FifoQ.mHead*FifoQ.mObjSize, obj, FifoQ.mObjSize);
//		++FifoQ.mHead;
//		FifoQ.mHead %= FifoQ.mLength;
//		FifoQ.mSize = FifoQ.mLength;
//		ret = 1;
//	}
//
//	sem_post(&FifoQ.mSemaphore);
//	return ret;
//}

//int FifoQGet(void *obj) {
//	int ret = -1;
//	sem_wait(&FifoQ.mSemaphore);
//	if(FifoQ.mSize > 0) {
//		memcpy(obj, FifoQ.mFifoPtr + FifoQ.mTail*FifoQ.mObjSize, FifoQ.mObjSize);
//		++FifoQ.mTail;
//		FifoQ.mTail %= FifoQ.mLength;
//		FifoQ.mSize--;
//		ret = 1;
//	} else {
//		ret = -1;
//	}
//	sem_post(&FifoQ.mSemaphore);
//
//	return ret;
//}

//int FifoQRead(void *obj, int index){
//	if(FifoQ.mSize >0) {
//		if(index > FifoQ.mSize){
//			return -1;
//		} else {
//			FifoQ.mIndex = FifoQ.mTail + index;
//			FifoQ.mIndex %= FifoQ.mLength;
//			obj = &FifoQ.mFifoPtr[FifoQ.mIndex];
//			return 1;
//		}
//	} else
//		return -1;
//	return 1;
//}

//int FifoQLength() {
//	return FifoQ.mSize;
//}
//
//void FifoQReset() {
//	FifoQ.mHead = 0;
//	FifoQ.mTail = 0;
//	FifoQ.mSize = 0;
//}

void sendString(const char *stringToSend) {
	if(validSession_send) {
		//printf("Sending string %s of length %d\n", stringToSend, strlen(stringToSend));
		send(mSessionSocket_send, (const void *)stringToSend, strlen(stringToSend), 0);
	}
}

void sendDouble(const double *val) {
	if(validSession_send) {
		send(mSessionSocket_send, (const void *)val, sizeof(double), 0);
	}
}

void sendInt(const int *val) {
	if(validSession_send) {
		send(mSessionSocket_send, (const void *)val, sizeof(int), 0);
	}
}

void sendDoublePacket(const double *buf, int num_doubles) {
    int i;
    const int max_doubles_per_packet = 175;
    if(validSession_send) {
	//for(i = 0; i=i+max_doubles_per_packet; i < num_doubles - 1) {
    for(i = 0; i < (num_doubles - 1); i+=max_doubles_per_packet) {
	    if(i+max_doubles_per_packet < num_doubles)
		send(mSessionSocket_send, buf+i, max_doubles_per_packet*sizeof(double), 0);
	    else
		send(mSessionSocket_send, buf+i, (num_doubles - i)*sizeof(double), 0);
	    
	}
    }
}
