// Link layer protocol implementation

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
    STATE_BCC_RCV,
    STATE_STOP
} State;

static int llreceiveFrame(unsigned char *frame) {
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

            case STATE_FLAG_RCV:
                // determine if the byte received is a valid address
                switch (byte) {
                    case ADDRESS_TX_SEND:
                    case ADDRESS_RX_SEND:
                        frame[0] = byte;
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
                        frame[1] = byte;
                        state = STATE_CONTROL_RCV;

                        break;

                    case FLAG:
                        state = STATE_FLAG_RCV;
                        break;

                    default:
                        state = STATE_START;
                }

                break;

            //case STATE_CONTROL_RCV:
                
        }
    }

    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
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
