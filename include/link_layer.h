// Link layer header.
// NOTE: This file must not be changed.

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_

#include <sys/time.h>

#include "serial_port.h"

/**
 * @brief A struct that represents the Data-Link Layer.
 */
typedef struct {
    SerialPort *sp;
    _Bool isSender;         /** indicates whether the program is the sender or receiver */
    int maxRetransmissions; /** the maximum number of times an individual frame can be retransmitted */
    int timeout;            /** the number of seconds of timeout */
    _Bool sequenceNum;

    // for statistical purposes
    struct timeval startTime; /** the time when the program starts */
    int framesTransmitted;
    int framesRetransmitted;
    int framesReceived;
    int framesRejected;
    int timeouts;
    long dataBytesTransferred;
} LinkLayer;

// SIZE of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer
#define MAX_PAYLOAD_SIZE 1000

/* API */
LinkLayer *llInit(const char *serialPort, _Bool isSender, int baudRate, int maxRetransmissions, int timeout);
int llFree(LinkLayer *ll);

int llOpen(LinkLayer *ll);
int llWrite(LinkLayer *ll, const unsigned char *packet, int packetSize);
int llRead(LinkLayer *ll, unsigned char *packet);
int llClose(LinkLayer *ll, int showStatistics);
void llPrintStatistics(LinkLayer *ll);

#endif // _LINK_LAYER_H_
