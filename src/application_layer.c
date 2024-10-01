// Application layer protocol implementation

#include <string.h>

#include "application_layer.h"
#include "link_layer.h"

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // configure the connection
    LinkLayer connectionParameters = {
        .role = (role[0] == 't') ? LlTx : LlRx,
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout
    };
    memcpy(connectionParameters.serialPort, serialPort, 50);

    llopen(connectionParameters);
}
