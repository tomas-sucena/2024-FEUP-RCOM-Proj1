// Link layer header.
// NOTE: This file must not be changed.

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_

#include <sys/time.h>

#include "serial_port.h"

/**
 * @brief A struct that represents the data-link layer.
 */
typedef struct {
    SerialPort *sp;         /** the serial port */
    _Bool isSender;         /** indicates whether the program is the sender or receiver */
    int maxRetransmissions; /** the maximum number of times an individual frame can be retransmitted */
    int timeout;            /** the number of seconds before a timeout occurs */
    int iFrames;            /** the number of I-frames sent/received */
    FILE *logbook;          /** the file to which logs will be output */

    // for statistical purposes
    struct timeval startTime;     /** the time when the program starts */
    int numFramesTransmitted;     /** the number of frames transmitted */
    int numFramesRetransmitted;   /** the number of frames retransmitted */
    int numFramesReceived;        /** the number of frames received */
    int numFramesRejected;        /** the number of frames rejected */
    int numTimeouts;              /** the number of timeouts that occurred */
    long numDataBytesTransferred; /** the total number of data bytes sent/received */
} LinkLayer;

// SIZE of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer
#define MAX_PAYLOAD_SIZE 1000

/* API */
LinkLayer *llInit(const char *serialPort, const char *role, int baudRate, int maxRetransmissions, int timeout);
int llFree(LinkLayer *ll);

int llOpen(LinkLayer *ll);
int llWrite(LinkLayer *ll, const unsigned char *packet, int packetSize);
int llRead(LinkLayer *ll, unsigned char *packet);
int llClose(LinkLayer *ll, _Bool showStatistics);

#endif // _LINK_LAYER_H_
