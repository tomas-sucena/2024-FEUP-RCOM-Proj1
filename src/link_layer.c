// Link layer protocol implementation

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include "../include/link_layer.h"
#include "../include/utils.h"

#define FLAG 0x7E

#define ADDRESS_TX_SEND 0x03
#define ADDRESS_RX_SEND 0x01

#define CONTROL_SET   0x03
#define CONTROL_UA    0x07
#define CONTROL_I0    0x00
#define CONTROL_I1    0x80
#define CONTROL_RR0   0xAA
#define CONTROL_RR1   0xAB  
#define CONTROL_REJ0  0x54
#define CONTROL_REJ1  0x55  
#define CONTROL_DISC  0x0B

#define ESCAPE   0x7D
#define XOR_BYTE 0x20

typedef enum {
    STATE_START,
    STATE_FLAG_RCV,
    STATE_ADDRESS_RCV,
    STATE_CONTROL_RCV,
    STATE_BCC_OK
} State;

////////////////////////////////////////////////
// ALARM
////////////////////////////////////////////////
static _Bool alarmIsEnabled; /** indicates if the alarm is activated */

static void handleAlarm() {
    alarmIsEnabled = FALSE;
}

static void setAlarm(int time) {
    (void) signal(SIGALRM, handleAlarm);
    alarm(time);
    alarmIsEnabled = TRUE;
}

////////////////////////////////////////////////
// SENDER
////////////////////////////////////////////////
static int sendFrame(LinkLayer *ll, unsigned char address, unsigned char control) {
    // initialize and configure the frame
    unsigned char frame[5] = {FLAG, address, control, address ^ control, FLAG};
    
    // send the frame
    return (spWrite(ll->port, frame, 5) == 5)
        ? STATUS_SUCCESS
        : STATUS_ERROR;
}

static int sendDataFrame(LinkLayer *ll, unsigned char control, const unsigned char *data, int dataSize) {
    // initialize the frame
    unsigned char *frame = (unsigned char *) malloc((7 + dataSize * 2) * sizeof(unsigned char));

    // configure the header of the frame
    frame[0] = FLAG;
    frame[1] = ADDRESS_TX_SEND; // NOTE: Only the sender can send data, so hardcoding
                                // the address is justifiable.
    frame[2] = control;
    frame[3] = ADDRESS_TX_SEND ^ control;

    // append the data to the frame
    int index = 4;
    unsigned char BCC = 0;

    for (int i = 0; i < dataSize; ++i) {
        unsigned char byte = data[i];
        BCC ^= byte;

        switch (byte) {
            case FLAG:
            case ESCAPE:
                // escape the byte
                frame[index++] = ESCAPE;
                byte ^= XOR_BYTE;

            default:
                frame[index++] = byte;
                break;
        }
    }

    // configure the trailer of the frame
    switch (BCC) {
        case FLAG:
        case ESCAPE:
            // escape the byte
            frame[index++] = ESCAPE;
            BCC ^= XOR_BYTE;

        default:
            frame[index++] = BCC;
            break;
    }

    frame[index++] = FLAG;

    // send the frame
    if (spWrite(ll->port, frame, index) < 0) {
        return STATUS_ERROR;
    }

    free(frame);
    return STATUS_SUCCESS;
}

////////////////////////////////////////////////
// RECEIVER
////////////////////////////////////////////////
static int receiveData(LinkLayer *ll, unsigned char *data, unsigned char byte) {
    unsigned char BCC = 0;
    _Bool escape = FALSE; // indicates if the next character should be escaped

    int index = 0;

    // parse the first data byte
    if (byte == ESCAPE) {
        escape = TRUE;
    }
    else {
        data[index++] = byte;
        BCC = byte; 
    }

    // receive the remaining data bytes
    while (alarmIsEnabled) {
        // ensure there were no errors reading the byte
        if (spRead(ll->port, &byte) < 0) {
            continue;
        }

        switch (byte) {
            case FLAG:
                // ensure the BCC is zero, as that means the data is correct
                // NOTE: x ^ x = 0
                return (BCC == 0)
                    ? (index - 1)
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

                break;
        }
    }

    return STATUS_ERROR; // a timeout occurred
}

static int receiveFrame(LinkLayer *ll, unsigned char *address, unsigned char *control, unsigned char *data) {
    State state = STATE_START;
    unsigned char BCC;

    // activate the alarm
    setAlarm(ll->timeout);

    while (alarmIsEnabled)  {
        unsigned char byte;
        
        // ensure there were no errors reading the byte
        if (spRead(ll->port, &byte) <= 0) {
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
                BCC = (*address = byte); // assign the address and update the BCC
                state = STATE_ADDRESS_RCV;

                break;

            case STATE_ADDRESS_RCV:
                BCC ^= (*control = byte); // assign the control byte and update the BCC
                state = STATE_CONTROL_RCV;

                break;

            case STATE_CONTROL_RCV:
                // verify if the current byte is the expected BCC,
                // that is, if it is the XOR of the previous two bytes
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
                // verify if the current byte is the flag, that is,
                // if we have reached the end of the frame
                if (byte == FLAG) {
                    return STATUS_SUCCESS;
                }

                // if the current byte is NOT the flag, that means
                // the frame contains data, so receive it
                int bytesRead = receiveData(ll, data, byte);

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
// API
////////////////////////////////////////////////
LinkLayer *llInit(const char *serialPort, _Bool isSender, int baudRate, int nRetransmissions, int timeout) {
    // open the serial port
    SerialPort *port = spInit(serialPort, baudRate);

    if (port == NULL) {
        return NULL;
    }

    // initialize the link layer
    LinkLayer *ll = malloc(sizeof(LinkLayer));

    ll->port = port;
    ll->isSender = isSender;
    ll->nRetransmissions = nRetransmissions;
    ll->timeout = timeout;
    ll->sequenceNum = 0;

    return ll;
}

int llFree(LinkLayer *ll) {
    // free the serial port
    int statusCode = spFree(ll->port);
    
    free(ll);
    return statusCode;
}

int llOpen(LinkLayer *ll) {
    // establish communication with the other PC
    _Bool done = FALSE;

    for (int attempt = 0; !done && attempt < ll->nRetransmissions; ++attempt) {
        if (attempt > 0) {
            printf(" Trying again..." RESET "\n");
        }

        unsigned char address, control;

        if (ll->isSender) {
            // send the SET frame
            if (sendFrame(ll, ADDRESS_TX_SEND, CONTROL_SET) < 0) {
                printf(FAINT "Failed to send the SET frame.");
                continue;
            }

            // receive the UA frame
            if (receiveFrame(ll, &address, &control, NULL) < 0) {
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
            if (receiveFrame(ll, &address, &control, NULL) < 0) {
                printf(FAINT "Failed to receive the SET frame.");
                continue;
            }

            // ensure the SET frame is correct
            if (address != ADDRESS_TX_SEND || control != CONTROL_SET) {
                printf(FAINT "Received a UA frame with errors.");
                continue;
            }

            // send the UA frame
            if (sendFrame(ll, ADDRESS_TX_SEND, CONTROL_UA) < 0) {
                printf(FAINT "Failed to send the UA frame.");
                continue;
            }
        }

        done = TRUE;
    }

    // verify if a timeout occurred
    if (!done) {
        putchar('\n');
        return STATUS_ERROR;
    }

    return STATUS_SUCCESS;
}

int llWrite(LinkLayer *ll, const unsigned char *packet, int packetSize) {
    _Bool done = FALSE;

    for (int attempt = 0; !done && attempt < ll->nRetransmissions; ++attempt) {
        if (attempt > 0) {
            printf(" Trying again..." RESET "\n");
        }

        unsigned char address, control;

        // send the packet
        if (sendDataFrame(ll, ll->sequenceNum << 7, packet, packetSize) < 0) {
            printf(FAINT "Failed to send the I-frame.");
            continue;
        }

        // receive the UA frame
        if (receiveFrame(ll, &address, &control, NULL) < 0) {
            printf(FAINT "Failed to receive the UA frame.");
            continue;
        }

        // ensure the address of the UA frame is correct
        if (address != ADDRESS_TX_SEND) {
            printf(FAINT "Received a UA frame with errors.");
            continue;
        }

        // parse the control byte of the UA frame
        switch (control) {
            case CONTROL_RR0:
            case CONTROL_RR1:
                // ensure the receiver is requesting the correct frame
                if (control == (CONTROL_RR0 | (ll->sequenceNum ^ 1))) {
                    done = TRUE;
                }
                else {
                    printf(FAINT "Received a UA frame with errors.");
                }

                break;

            case CONTROL_REJ0:
            case CONTROL_REJ1:
                // ensure the receiver is rejecting the correct frame
                if (control == (CONTROL_REJ0 | ll->sequenceNum)) {
                    printf(FAINT "Sent duplicate I-frame #%d.", ll->sequenceNum);
                }

            default:
                printf(FAINT "Received a UA frame with errors.");
                break;
        }
    }

    // verify if a timeout occurred
    if (!done) {
        putchar('\n');
        return STATUS_ERROR;
    }

    ll->sequenceNum ^= 1; // toggle the sequence number
    return packetSize;
}

int llRead(LinkLayer *ll, unsigned char *packet) {
    int bytesRead;
    _Bool done = FALSE;

    for (int attempt = 0; !done && attempt < ll->nRetransmissions; ++attempt) {
        if (attempt > 0) {
            printf(" Trying again..." RESET "\n");
        }

        unsigned char address, control;

        // receive the packet
        bytesRead = receiveFrame(ll, &address, &control, packet);

        if (bytesRead < 0) {
            printf(FAINT "Failed to receive an I-frame.");
            continue;
        }

        // ensure the address of the I-frame is correct
        if (address != ADDRESS_TX_SEND) {
            printf(FAINT "Received an I-frame with errors.");
            continue;
        }

        // parse the control byte of the I-frame
        switch (control) {
            // the sender wants to disconnect
            case CONTROL_DISC:
                return 0;

            // received data
            case CONTROL_I0:
            case CONTROL_I1:
                // verify if the data received is new
                if ((control >> 7) == ll->sequenceNum) {
                    done = TRUE;
                }
                else {
                    printf(FAINT "Received duplicate I-frame #%d.", ll->sequenceNum ^ 1);
                }

                control = CONTROL_RR0 | (ll->sequenceNum ^ 1);
                break;

            default:
                printf(FAINT "Received an I-frame with errors.");
                control = CONTROL_REJ0 | ll->sequenceNum;

                break;
        }

        // send the acknowledgement frame
        if (sendFrame(ll, ADDRESS_TX_SEND, control) < 0) {
            printf(FAINT "Failed to send the UA frame.");
            done = FALSE;
        }
    }

    // verify if a timeout occurred
    if (!done) {
        putchar('\n');
        return STATUS_ERROR;
    }

    ll->sequenceNum ^= 1; // toggle the sequence number
    return bytesRead;
}

int llClose(LinkLayer *ll, int showStatistics){
    // end communication with the other PC
    _Bool done = FALSE;

    for (int attempt = 0; !done && attempt < ll->nRetransmissions; ++attempt) {
        if (attempt > 0) {
            printf(" Trying again..." RESET "\n");
        }

        unsigned char address, control;

        if (ll->isSender) {
            // send the DISC frame
            if (sendFrame(ll, ADDRESS_TX_SEND, CONTROL_DISC) < 0) {
                printf(FAINT "Failed to send the DISC frame." RESET);
                continue;
            }

            // receive the receiver's DISC frame
            if (receiveFrame(ll, &address, &control, NULL) < 0) {
                printf(FAINT "Failed to receive the receiver's DISC frame." RESET);
                continue;
            }

            // ensure the receiver's DISC frame is correct
            if (address != ADDRESS_RX_SEND || control != CONTROL_DISC) {
                printf(FAINT "Received a DISC frame with errors.");
                continue;
            }

            // send the UA frame
            if (sendFrame(ll, ADDRESS_RX_SEND, CONTROL_UA) < 0) {
                printf(FAINT "Failed to send the UA frame." RESET);
                continue;
            }
        }
        else {
            // send the DISC frame
            if (sendFrame(ll, ADDRESS_RX_SEND, CONTROL_DISC) < 0) {
                printf(FAINT "Failed to send the DISC frame!" RESET);
                continue;
            }

            // receive the UA frame
            if (receiveFrame(ll, &address, &control, NULL) < 0) {
                printf(FAINT "Failed to receive the UA frame!" RESET);
                continue;
            }

            // ensure the UA frame is correct
            if (address != ADDRESS_RX_SEND || control != CONTROL_UA) {
                printf(FAINT "Received a UA frame with errors.");
                continue;
            }
        }

        done = TRUE;
    }

    // verify if a timeout occurred
    if (!done) {
        putchar('\n');
        return STATUS_ERROR;
    }

    return STATUS_SUCCESS;
}
