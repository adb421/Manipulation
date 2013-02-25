///*
// * serial_com.c
// *
// *  Created on: Dec 20, 2012
// *      Author: Adam
// */
//
//#include "serial_com.h"
//
//void readInt(int* val) {
//	if(validSession) {
//		read(port, (void *) val, sizeof(int));
//	}
//	else {
//		*val = -1;
//	}
//}
//
//void readDouble(double* val) {
//	if(validSession) {
//		read(port, (void *) val, sizeof(double));
//	} else {
//		*val = -1;
//	}
//}
//
//void sendString(const char *stringToSend) {
//	int bytes = -1;
//	if(validSession) {
//		while(bytes < 0) {
//			bytes = write(port, stringToSend, sizeof(char)*strlen(stringToSend));
//		fflush(&port);
//		}
//		printf("Sent the word: %s in %d bytes, word is of length %d\n", stringToSend, bytes, strlen(stringToSend));
//	}
//}
//
//void sendInt(const int* val) {
//	int bytes = -1;
//	if(validSession) {
//		while(bytes < 0) {
//			bytes = write(port, val, sizeof(int));
//			fflush(&port);
//		}
//	}
//}
//void sendDouble(const double* val){
//	int bytes = -1;
//	if(validSession) {
//		while(bytes < 0) {
//			bytes = write(port, val, sizeof(double));
//			fflush(&port);
//		}
//	}
//}
//
//void * serial_com_thread(void *arg) {
//	//Use the serial port
//	SERIAL_PORT_NAME = "/dev/ser1";
//	//Open with read-write, don't use it as a command line terminal
//	port = open(SERIAL_PORT_NAME, O_RDWR | O_NOCTTY | O_NDELAY);
//	if(port == -1) {
//		printf("FAIL: Port not opened\n");
//	} else {
//		//Don't delay on reads
//		fcntl(port, F_SETFL);
//
//	}
//	//Set the options
//	tcgetattr(port, &options);
//	//Set baud rate to 9600 for IO
//	cfsetispeed(&options, B9600);
//	cfsetospeed(&options, B9600);
//	//Set hardware flow control
//	options.c_cflag |= IHFLOW;
//	options.c_cflag |= OHFLOW;
//	//Set options
//	tcsetattr(port, TCSANOW, &options);
//	validSession = 1;
//	//id is the interrupt id
//	int id;
//	int cmd = -1; //Pointer for the command to send
//	//This sigevent sends the interrupt to later in the thread.
//	struct sigevent serialEvent;
//	SIGEV_INTR_INIT(&serialEvent);
//	//Attach to the event
//	id = InterruptAttachEvent(IRQ4, &serialEvent, 0);
//	if(id < 0)
//		printf("Interrupt not attached.\n");
//	//Now, run your loop for the rest of time
//	while(1) {
//		InterruptWait(0, NULL);
//		//Got an interrupt.
//		//Check if we are waiting for a command:
//		if(waiting_for_cmd) {
//			//Read the command from the serial port
//			//read(port, (void *) cmd, sizeof(int));
//			readInt(&cmd);
//			//Process the command!
//			process_cmd(cmd);
//		} else {
//			printf("Got a serial int without waiting for command\n");
//		}
//		//else: do nothing, shouldn't happen as the interrupt shouldn't be unmasked if we aren't waiting.
//		//Unmask interrupt so it kicks in next time.
//		InterruptUnmask(IRQ4, id);
//	}
//
//}
