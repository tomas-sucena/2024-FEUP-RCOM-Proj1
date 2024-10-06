// Link layer protocol implementation

#include <signal.h>
#include <stdio.h>
#include <unistd.h>

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

// global variables
_Bool isSender;
int timeout;
_Bool alarmIsEnabled;
int nRetransmissions;

static void handleAlarm() {
    alarmIsEnabled = FALSE;
}

static void setAlarm(int time) {
    (void) signal(SIGALRM, handleAlarm);
    alarm(time);
    alarmIsEnabled = TRUE;
}

static int sendFrame(unsigned char address, unsigned char control) {
    // initialize and configure the frame
    unsigned char frame[5];
    
    frame[0] = frame[4] = FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = address ^ control;
    
    // send the frame
    return writeBytesSerialPort(frame, 5);
}

static int receiveFrame(unsigned char address, unsigned char control, unsigned char *frame) {
    State state = STATE_START;
    const unsigned char BCC = address ^ control; // the expected BCC

    // activate the alarm
    setAlarm(timeout);

    while (alarmIsEnabled && state != STATE_STOP)  {
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

    // verify if a timeout occurred
    return (state == STATE_STOP) ? 1 : -1;
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

    // set the global variables
    isSender = (connectionParameters.role == LlTx);
    timeout = connectionParameters.timeout;
    nRetransmissions = connectionParameters.nRetransmissions;

    // establish communication with the other PC
    const char *otherPC = isSender ? "receiver" : "sender";
    int attempt;

    printf("\n> Establishing communication with the %s...\n", otherPC);

    for (attempt = 0; attempt < nRetransmissions; ++attempt) {
        if (attempt > 0) {
            printf("\nTrying again...\n");
        }

        if (isSender) {
            // send the SET frame
            if (sendFrame(ADDRESS_TX_SEND, CONTROL_SET) < 0) {
                printf("Failed to send the SET frame!\n");
                continue;
            }

            // receive the UA frame
            if (receiveFrame(ADDRESS_TX_SEND, CONTROL_UA, NULL) < 0) {
                printf("Failed to receive the UA frame!\n");
                continue;
            }
        }
        else {
            // receive the SET frame
            if (receiveFrame(ADDRESS_TX_SEND, CONTROL_SET, NULL) < 0) {
                printf("Failed to receive the SET frame!\n");
                continue;
            }

            // send the UA frame
            if (sendFrame(ADDRESS_TX_SEND, CONTROL_UA) < 0) {
                printf("Failed to send the UA frame!\n");
                continue;
            }
        }

        break;
    }

    // ensure communication was established
    if (attempt == nRetransmissions) {
        printf("Failed to establish communication with the %s!\n", otherPC);
        return -1;
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
