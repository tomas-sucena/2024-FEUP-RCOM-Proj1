// Application layer protocol header.
// NOTE: This file must not be changed.

#ifndef _APPLICATION_LAYER_H_
#define _APPLICATION_LAYER_H_

typedef struct {
    FILE *file;
    const char *filename;
    const char *filepath;
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
ApplicationLayer *applicationLayer(_Bool isSender, const char *filepath, int dataSize);

// Application layer main function
//int fileTransferProtocol(ApplicationLayer *app, LinkLayer *ll);

#endif // _APPLICATION_LAYER_H_
