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

static int llreceiveFrame(unsigned char *frame) {
    State state = STATE_START;
    unsigned char BCC;
    
    while (state != STATE_STOP)  {
        unsigned char byte;
        
        // ensure there were no errors reading the byte
        if (readByteSerialPort(&byte) <= 0) {
            continue;
        }

        switch (state) {
            case STATE_START:
                if (byte == FLAG) {
                    state = STATE_FLAG_RCV;
                }

                break;

            case STATE_FLAG_RCV:
                // determine if the byte received is a valid address
                switch (byte) {
                    case ADDRESS_TX_SEND:
                    case ADDRESS_RX_SEND:
                        BCC = frame[0] = byte;
                        state = STATE_ADDRESS_RCV;

                        break;
                    
                    case FLAG:
                        break;

                    default:
                        state = STATE_START;
                }

                break;

            case STATE_ADDRESS_RCV:
                // determine if the byte received is a valid control byte
                switch (byte) {
                    case CONTROL_SET:
                    case CONTROL_UA:
                        BCC ^= frame[1] = byte;
                        state = STATE_CONTROL_RCV;

                        break;

                    case FLAG:
                        state = STATE_FLAG_RCV;
                        break;

                    default:
                        state = STATE_START;
                }

                break;

            case STATE_CONTROL_RCV:
                // verify if the byte equals the BCC
                if (byte == BCC) {
                    state = STATE_BCC_OK;
                }
                else if (byte == FLAG) {
                    state = STATE_FLAG_RCV;                
                }
                else {
                    state = STATE_START;                
                }

                break;
            
            case STATE_BCC_OK:
                state = (byte == FLAG)
                    ? STATE_STOP
                    : STATE_START;
            
            // unused but prevents compilation warnings
            default:
                break;
        }
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
        llreceiveFrame(frame);
    }
    else {
       // receive the SET frame
       llreceiveFrame(frame);

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
