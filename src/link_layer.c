// Link layer protocol implementation

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>

#include "../include/link_layer.h"
#include "../include/serial_port.h"
#include "../include/utils.h"

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

#define ESCAPE   0x5E
#define XOR_BYTE 0x20

typedef enum {
    STATE_START,
    STATE_FLAG_RCV,
    STATE_ADDRESS_RCV,
    STATE_CONTROL_RCV,
    STATE_BCC_OK
} State;

/* global variables */
// link layer
static _Bool isSender;
static int timeout;

// alarm
static _Bool alarmIsEnabled;
static int nRetransmissions;

static void handleAlarm() {
    alarmIsEnabled = FALSE;
}

static void setAlarm(int time) {
    (void) signal(SIGALRM, handleAlarm);
    alarm(time);
    alarmIsEnabled = TRUE;
}

/* SENDER */
static int sendFrame(unsigned char address, unsigned char control) {
    // initialize and configure the frame
    unsigned char frame[5];
    
    frame[0] = frame[4] = FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = address ^ control;
    
    // send the frame
    return (writeBytesSerialPort(frame, 5) == 5)
        ? STATUS_SUCCESS
        : STATUS_ERROR;
}

static int sendDataFrame(unsigned char control, const unsigned char *data, int dataSize) {
    // initialize and configure the frame
    int frameSize = 7 + dataSize;
    unsigned char *frame = malloc(frameSize * sizeof(unsigned char));

    frame[0] = frame[frameSize - 1] = FLAG;
    frame[1] = ADDRESS_TX_SEND; // NOTE: Only the sender can send data, so hardcoding
                                // the address is justifiable.
    frame[2] = control;
    frame[3] = ADDRESS_TX_SEND ^ control;

    // append the data to the frame
    int index = 4;

    for (int i = 0; i < dataSize; ++i) {
        unsigned char byte = data[i];

        switch (byte) {
            case FLAG:
            case ESCAPE:
                // escape the byte
                frame[index++] = ESCAPE;
                frame[index++] = byte ^ XOR_BYTE;

                break;

            default:
                frame[index++] = byte;
        }
    }

    // send the frame
    if (writeBytesSerialPort(frame, frameSize) < frameSize) {
        return STATUS_ERROR;
    }

    free(frame);
    return STATUS_SUCCESS;
}

/* RECEIVER */
static int receiveFrame(unsigned char address, unsigned char control, unsigned char *data) {
    State state = STATE_START;
    const unsigned char BCC = address ^ control; // the expected BCC

    // activate the alarm
    setAlarm(timeout);

    while (alarmIsEnabled)  {
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
                // verify if we have reached the end of the frame
                if (byte == FLAG) {
                    return STATUS_SUCCESS;
                }

                // if the current byte is NOT the flag, that means
                // the frame contains data, so receive it
                int bytesRead = receiveData(data);

                if (bytesRead > 0) {
                    return bytesRead;
                }
                
                // NOTE: If this code is reached, that means a timeout occurred when
                // receiving data or the data itself contained errors. 
                state = STATE_FLAG_RCV;
                continue;
        }

        state = (byte == FLAG)
            ? STATE_FLAG_RCV
            : STATE_START;
    }

    return STATUS_ERROR; // a timeout occurred
}

static int receiveData(unsigned char *data) {
    unsigned char BCC = 0;
    _Bool escape = FALSE; // indicates if the next character should be escaped

    int index = 0;

    while (alarmIsEnabled) {
        unsigned char byte;

        // ensure there were no errors reading the byte
        if (readByteSerialPort(&byte) <= 0) {
            continue;
        }

        switch (byte) {
            case FLAG:
                // ensure the BCC is zero, as that means the data is correct
                // NOTE: x ^ x = 0
                return (BCC == 0)
                    ? index
                    : STATUS_ERROR;

            case ESCAPE:
                escape = TRUE;
                break;

            default:
                // verify if the byte must be escaped
                if (escape) {
                    byte ^= XOR_BYTE;
                    escape = FALSE;
                }

                // append the byte to the data
                data[index++] = byte;
                BCC ^= byte;
        }
    }

    return STATUS_ERROR; // a timeout occurred
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) {
        return STATUS_ERROR;
    }

    // set the global variables
    isSender = (connectionParameters.role == LlTx);
    timeout = connectionParameters.timeout;
    nRetransmissions = connectionParameters.nRetransmissions;

    // establish communication with the other PC
    int attempt;
    const char *otherPC = isSender ? "receiver" : "sender";

    printf("\n> Establishing connection with the %s...\n", otherPC);

    for (attempt = 0; attempt < nRetransmissions; ++attempt) {
        if (attempt > 0) {
            printf(" Trying again...\n");
        }

        if (isSender) {
            // send the SET frame
            if (sendFrame(ADDRESS_TX_SEND, CONTROL_SET) < 0) {
                printf(FAINT "Failed to send the SET frame.");
                continue;
            }

            // receive the UA frame
            if (receiveFrame(ADDRESS_TX_SEND, CONTROL_UA, NULL) < 0) {
                printf(FAINT "Failed to receive the UA frame.");
                continue;
            }
        }
        else {
            // receive the SET frame
            if (receiveFrame(ADDRESS_TX_SEND, CONTROL_SET, NULL) < 0) {
                printf(FAINT "Failed to receive the SET frame.");
                continue;
            }

            // send the UA frame
            if (sendFrame(ADDRESS_TX_SEND, CONTROL_UA) < 0) {
                printf(FAINT "Failed to send the UA frame.");
                continue;
            }
        }

        break;
    }

    // ensure communication was established
    if (attempt == nRetransmissions) {
        printf(RED "\nError! Failed to connect to the %s.\n" RESET, otherPC);
        return STATUS_ERROR;
    }

    printf(GREEN "Success!\n" RESET);
    return STATUS_SUCCESS;
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
int llclose(int showStatistics){
    // end communication with the other PC
    int attempt;
    printf("\n> Disconnecting...\n");

    for (attempt = 0; attempt < nRetransmissions; ++attempt) {
        if (attempt > 0) {
            printf(FAINT " Trying again...\n" RESET);
        }

        if (isSender) {
            // send the DISC frame
            if (sendFrame(ADDRESS_TX_SEND, CONTROL_DISC) < 0) {
                printf(FAINT "Failed to send the DISC frame!" RESET);
                continue;
            }

            // receive the receiver's DISC frame
            if (receiveFrame(ADDRESS_RX_SEND, CONTROL_DISC, NULL) < 0) {
                printf(FAINT "Failed to receive the receiver's DISC frame!" RESET);
                continue;
            }

            // send the UA frame
            if (sendFrame(ADDRESS_RX_SEND, CONTROL_UA) < 0) {
                printf(FAINT "Failed to send the UA frame!" RESET);
                continue;
            }
        }
        else {
            // receive the sender's DISC frame
            if (receiveFrame(ADDRESS_TX_SEND, CONTROL_DISC, NULL) < 0) {
                printf(FAINT "Failed to receive the sender's DISC frame!" RESET);
                continue;
            }

            // send the DISC frame
            if (sendFrame(ADDRESS_RX_SEND, CONTROL_DISC) < 0) {
                printf(FAINT "Failed to send the DISC frame!" RESET);
                continue;
            }

            // receive the UA frame
            if (receiveFrame(ADDRESS_RX_SEND, CONTROL_UA, NULL) < 0) {
                printf(FAINT "Failed to receive the UA frame!\n" RESET);
                continue;
            }
        }

        break;
    }

    // ensure the connection was terminated
    if (attempt == nRetransmissions) {
        printf(RED "\nError! Failed to terminate the connection with the %s.\n" RESET,
            isSender ? "receiver" : "sender");
        return STATUS_ERROR;
    }

    printf(GREEN "Success!\n" RESET);
    return closeSerialPort();
}
