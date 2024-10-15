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
#define CONTROL_I0   0x00
#define CONTROL_I1   0x80
#define CONTROL_RR   0xAA
#define CONTROL_REJ  0x54
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
static _Bool sequenceNum;

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

static int receiveFrame(unsigned char *address, unsigned char *control, unsigned char *data) {
    State state = STATE_START;

    // activate the alarm
    setAlarm(timeout);

    while (alarmIsEnabled)  {
        unsigned char byte;
        
        // ensure there were no errors reading the byte
        if (readByteSerialPort(&byte) <= 0) {
            continue;
        }

        switch (state) {
            case STATE_START:
                // verify if the byte is the flag
                if (byte == FLAG) {
                    state = STATE_FLAG_RCV;
                }

                break;

            case STATE_FLAG_RCV:
                *address = byte; // assign the address
                state = STATE_ADDRESS_RCV;

                break;

            case STATE_ADDRESS_RCV:
                *control = byte; // assign the control byte
                state = STATE_CONTROL_RCV;

                break;

            case STATE_CONTROL_RCV:
                // verify if the current byte is the expected BCC,
                // that is, if it is the XOR of the previous two bytes
                if (byte == *address ^ *control) {
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
                // verify if the current byte is the flag, that is,
                // if we have reached the end of the frame
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
                break;
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
    sequenceNum = 0;
    nRetransmissions = connectionParameters.nRetransmissions;

    // establish communication with the other PC
    int attempt;

    for (attempt = 0; attempt < nRetransmissions; ++attempt) {
        if (attempt > 0) {
            printf(" Trying again...\n");
        }

        unsigned char address, control;

        if (isSender) {
            // send the SET frame
            if (sendFrame(ADDRESS_TX_SEND, CONTROL_SET) < 0) {
                printf(FAINT "Failed to send the SET frame.");
                continue;
            }

            // receive the UA frame
            if (receiveFrame(&address, &control, NULL) < 0) {
                printf(FAINT "Failed to receive the UA frame.");
                continue;
            }

            // ensure the UA frame is correct
            if (address != ADDRESS_TX_SEND || control != CONTROL_UA) {
                printf(FAINT "Received a UA frame with errors.");
                continue;
            }
        }
        else {
            // receive the SET frame
            if (receiveFrame(&address, &control, NULL) < 0) {
                printf(FAINT "Failed to receive the SET frame.");
                continue;
            }

            // ensure the SET frame is correct
            if (address != ADDRESS_TX_SEND || control != CONTROL_SET) {
                printf(FAINT "Received a UA frame with errors.");
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
    return (attempt < nRetransmissions)
        ? STATUS_SUCCESS
        : STATUS_ERROR;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *packet, int packetSize)
{
    int attempt;

    for (attempt = 0; attempt < nRetransmissions; ++attempt) {
        if (attempt > 0) {
            printf(" Trying again...\n");
        }

        unsigned char address, control;

        // send the packet        
        if (sendDataFrame(CONTROL_REJ | sequenceNum, packet, packetSize) < 0) {
            printf(FAINT "Failed to send the I-frame.");
            continue;
        }

        // receive the UA frame
        if (receiveFrame(ADDRESS_TX_SEND, CONTROL_REJ ^ sequenceNum, NULL) < 0) {
            printf(FAINT "Failed to receive the UA frame.");
            continue;
        }

        // ensure the UA frame is correct
        if (address != ADDRESS_TX_SEND) {
            printf(FAINT "Received a UA frame with errors.");
            continue;
        }

        break;
    }

    // verify if a timeout occurred
    if (attempt == nRetransmissions) {
        putchar('\n');
        return STATUS_ERROR;
    }

    sequenceNum ^= 1; // toggle the sequence number
    return packetSize;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    int bytesRead, attempt;

    for (attempt = 0; attempt < nRetransmissions; ++attempt) {
        unsigned char address, control;

        // receive the packet
        bytesRead = receiveFrame(&address, &control, packet);

        if (bytesRead < 0) {
            printf(FAINT "Failed to receive the I-frame.");
            continue;
        }

        // ensure the address of the I-frame is correct
        if (address != ADDRESS_TX_SEND) {
            printf(FAINT "Received an I-frame with errors.");
            continue;
        }

        // parse the control byte
        _Bool newData = FALSE;

        switch (control) {
            // the sender wants to disconnect
            case CONTROL_DISC:
                return 0;

            // received data
            case CONTROL_I0:
            case CONTROL_I1:
                // verify if the data received is new
                if (control >> 7 == sequenceNum) {
                    newData = TRUE;
                }
                else {
                    printf(FAINT "Received duplicate I-frame %d.\n", sequenceNum ^ 1);
                }

                control = CONTROL_RR | (sequenceNum ^ 1);
                break;

            default:
                printf(FAINT "Received an I-frame with errors.");
                control = CONTROL_REJ | sequenceNum;

                break;
        }

        // send the UA frame
        if (sendFrame(ADDRESS_TX_SEND, control) < 0) {
            printf(FAINT "Failed to send the UA frame.");
            continue;
        }

        // exit the loop only if the data received was new
        if (newData) {
            break;
        }
    }

    // verify if a timeout occurred
    if (attempt == nRetransmissions) {
        return STATUS_ERROR;
    }

    sequenceNum ^= 1; // toggle the sequence number
    return bytesRead;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics){
    // end communication with the other PC
    int attempt;

    for (attempt = 0; attempt < nRetransmissions; ++attempt) {
        if (attempt > 0) {
            printf(FAINT " Trying again...\n" RESET);
        }

        unsigned char address, control;

        if (isSender) {
            // send the DISC frame
            if (sendFrame(ADDRESS_TX_SEND, CONTROL_DISC) < 0) {
                printf(FAINT "Failed to send the DISC frame." RESET);
                continue;
            }

            // receive the receiver's DISC frame
            if (receiveFrame(&address, &control, NULL) < 0) {
                printf(FAINT "Failed to receive the receiver's DISC frame." RESET);
                continue;
            }

            // ensure the receiver's DISC frame is correct
            if (address != ADDRESS_RX_SEND || control != CONTROL_DISC) {
                printf(FAINT "Received a DISC frame with errors.");
                continue;
            }

            // send the UA frame
            if (sendFrame(ADDRESS_RX_SEND, CONTROL_UA) < 0) {
                printf(FAINT "Failed to send the UA frame." RESET);
                continue;
            }
        }
        else {
            // send the DISC frame
            if (sendFrame(ADDRESS_RX_SEND, CONTROL_DISC) < 0) {
                printf(FAINT "Failed to send the DISC frame!" RESET);
                continue;
            }

            // receive the UA frame
            if (receiveFrame(&address, &control, NULL) < 0) {
                printf(FAINT "Failed to receive the UA frame!\n" RESET);
                continue;
            }

            // ensure the UA frame is correct
            if (address != ADDRESS_RX_SEND || control != CONTROL_UA) {
                printf(FAINT "Received a UA frame with errors.");
                continue;
            }
        }

        break;
    }

    // verify if a timeout occurred
    if (attempt == nRetransmissions) {
        return STATUS_ERROR;
    }

    return closeSerialPort();
}
