// Application layer protocol header.
// NOTE: This file must not be changed.

#ifndef _APPLICATION_LAYER_H_
#define _APPLICATION_LAYER_H_

#include "link_layer.h"

/**
 * @brief A struct that represents the application layer.
 */
typedef struct {
    LinkLayer *ll;  /** the data-link layer */
    FILE *file;     /** pointer to the file to be sent/received */
    char *filename; /** the name of the file to be sent/received */
    long fileSize;  /** the size of the file to be sent/received */
    int dataSize;   /** the maximum number of data bytes of a data packet */
} ApplicationLayer;

/* API */
ApplicationLayer *appInit(const char *serialPort, const char *role, int baudRate, int maxRetransmissions, int timeout,
                          const char *filepath, int dataSize);
int appFree(ApplicationLayer *app);

// Application layer main function
int appRun(ApplicationLayer *app);

#endif // _APPLICATION_LAYER_H_
