// Link layer protocol implementation

#include <stdio.h>

#include "link_layer.h"
#include "serial_port.h"

#define FLAG 0x7E

#define ADDRESS_TX_SEND 0x03
#define ADDRESS_RX_SEND 0x01

#define CONTROL_SET  0x03
#define CONTROL_UA   0x07
#define CONTROL_RR0  0xAA
#define CONTROL_RR1  0xAB
#define CONTROL_REJ0 0x54
#define CONTROL_REJ1 0x55
#define CONTROL_DISC 0x0B

typedef enum {
    STATE_START,
    STATE_FLAG_RCV,
    STATE_ADDRESS_RCV,
    STATE_CONTROL_RCV,
    STATE_BCC_OK,
    STATE_STOP
} State;

static int receiveFrame(unsigned char address, unsigned char control, unsigned char *frame) {
    State state = STATE_START;
    const unsigned char BCC = address ^ control; // the expected BCC

    while (state != STATE_STOP)  {
        unsigned char byte;
        
        // ensure there were no errors reading the byte
        if (readByteSerialPort(&byte) <= 0) {
            continue;
        }

        switch (state) {
            case STATE_FLAG_RCV:
                // determine if the byte received is the expected address
                if (byte == address) {
                    state = STATE_ADDRESS_RCV;
                    continue;
                }

                break;

            case STATE_ADDRESS_RCV:
                // determine if the byte received is the expected control byte
                if (byte == control) {
                    state = STATE_CONTROL_RCV;
                    continue;
                }

                break;

            case STATE_CONTROL_RCV:
                // verify if the byte equals the BCC
                if (byte == BCC) {
                    state = STATE_BCC_OK;
                    continue;
                }

                break;
            
            case STATE_BCC_OK:
                // verify if the byte received is the FLAG, that is,
                // if we have reached the end of the frame
                state = (byte == FLAG)
                    ? STATE_STOP
                    : STATE_START;

                continue;
            
            default:
                break;
        }

        state = (byte == FLAG)
            ? STATE_FLAG_RCV
            : STATE_START;
    }

    return 0;
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

    // establish communication with the other PC
    unsigned char frame[5];
    
    if (connectionParameters.role == LlTx) {
        // send the SET frame
        frame[0] = frame[4] = FLAG;
        frame[1] = ADDRESS_TX_SEND;
        frame[2] = CONTROL_SET;
        frame[3] = frame[1] ^ frame[2];
        
        writeBytesSerialPort(frame, 5);

        // receive the UA frame
        receiveFrame(ADDRESS_TX_SEND, CONTROL_UA, NULL);
    }
    else {
        // receive the SET frame
        receiveFrame(ADDRESS_TX_SEND, CONTROL_SET, frame);

        // send the UA frame
        frame[0] = frame[4] = FLAG;
        frame[1] = ADDRESS_TX_SEND;
        frame[2] = CONTROL_UA;
        frame[3] = frame[1] ^ frame[2];
        
        writeBytesSerialPort(frame, 5);
    }
    
    printf("\e[0;32mConnection established!\e[0;37m\n");
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
