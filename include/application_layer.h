// Application layer protocol header.
// NOTE: This file must not be changed.

#ifndef _APPLICATION_LAYER_H_
#define _APPLICATION_LAYER_H_

#include "link_layer.h"

typedef struct {
    LinkLayer *ll;
    FILE *file;
    char *filename;
    long fileSize;
    int dataSize;
} ApplicationLayer;

// Application layer main function.
// Arguments:
//   serialPort: Serial port name (e.g., /dev/ttyS0).
//   role: Application role {"tx", "rx"}.
//   baudrate: Baudrate of the serial port.
//   nTries: Maximum number of frame retries.
//   timeout: Frame timeout.
//   filename: Name of the file to send / receive.
ApplicationLayer *appInit(const char *serialPort, _Bool isSender, int baudRate, int nTries, int timeout,
    const char *filepath, int dataSize);
int appFree(ApplicationLayer *app);

// Application layer main function
int appRun(ApplicationLayer *app);

#endif // _APPLICATION_LAYER_H_
