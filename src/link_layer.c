// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

int llreceiveFrame(unsigned char *frame) {
    State state = STATE_START;
    
    while (state != STATE_STOP)  {
        unsigned char byte;
        readByteSerialPort(&byte);        

        switch (state) {
            case STATE_START:
                if (byte == FLAG) {
                    state = STATE_FLAG_RCV;
                }

                break;
        }
    }    
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1;
    }

    // TODO
    unsigned char frame[5];
    
    if (connectionParameters.role == LlTx) {
        frame[0] = frame[4] = FLAG;
        frame[1] = ADDRESS_TX_SEND;
        frame[2] = CONTROL_SET;
        frame[3] = frame[1] ^ frame[2];

        writeBytesSerialPort(frame, 5);
    }
    else {
        

    }

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}
