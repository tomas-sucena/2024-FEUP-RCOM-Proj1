// Link layer protocol implementation

#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "../include/link_layer.h"
#include "../include/utils.h"

#define FLAG 0x7E

#define ADDRESS_TX_SEND 0x03
#define ADDRESS_RX_SEND 0x01

#define CONTROL_SET     0x03
#define CONTROL_UA      0x07
#define CONTROL_I0      0x00
#define CONTROL_I1      0x80
#define CONTROL_I(n)    ((n & 1) << 7)  
#define CONTROL_RR0     0xAA
#define CONTROL_RR1     0xAB
#define CONTROL_RR(n)   (CONTROL_RR0 | ((n) & 1))  
#define CONTROL_REJ0    0x54
#define CONTROL_REJ1    0x55
#define CONTROL_REJ(n)  (CONTROL_REJ0 | ((n) & 1))  
#define CONTROL_DISC    0x0B

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
// LOGBOOK
////////////////////////////////////////////////
static void writeLogsHeader(LinkLayer *ll, const char *header) {
    // ensure the logbook file has been properly opened
    if (ll->logbook == NULL) {
        return;
    }

    fprintf(ll->logbook, "\n> %s\n", header);
}

static void logEvent(LinkLayer *ll, _Bool error, const char *log, ...) {
    // ensure the logbook file has been properly opened
    if (ll->logbook == NULL) {
        return;
    }

    // if the current event is an error,
    // color the output red
    if (error) {
        fprintf(ll->logbook, RED);
    }

    // write the current time
    time_t currTime = time(NULL);
    fprintf(ll->logbook, "[%.*s] ", 24, asctime(gmtime(&currTime)));

    // fetch the variadic arguments
    va_list args;
    va_start(args, log);

    // write the log message
    vfprintf(ll->logbook, log, args);
    va_end(args);

    if (error) {
        fprintf(ll->logbook, RESET);
    }

    // output a new line
    fputc('\n', ll->logbook);
}

////////////////////////////////////////////////
// SENDER
////////////////////////////////////////////////
static int sendFrame(LinkLayer *ll, unsigned char address, unsigned char control) {
    // initialize and configure the frame
    unsigned char frame[5] = {FLAG, address, control, address ^ control, FLAG};
    
    // send the frame
    if (spWrite(ll->sp, frame, 5) < 5) {
        return STATUS_ERROR;
    }

    ++ll->framesTransmitted;
    return STATUS_SUCCESS;
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
    if (spWrite(ll->sp, frame, index) < 0) {
        return STATUS_ERROR;
    }

    free(frame);
    ++ll->framesTransmitted;

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
        if (spRead(ll->sp, &byte) < 0) {
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
        if (spRead(ll->sp, &byte) <= 0) {
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
                    ++ll->framesReceived;
                    return STATUS_SUCCESS;
                }

                // if the current byte is NOT the flag, that means
                // the frame contains data, so receive it
                int bytesRead = receiveData(ll, data, byte);

                if (bytesRead > 0) {
                    ++ll->framesReceived;
                    return bytesRead;
                }
                
                // NOTE: If this code is reached, that means a timeout occurred when
                // receiving data or the data itself contained errors. 
                state = STATE_FLAG_RCV;
                break;
        }
    }

    ++ll->timeouts;
    return STATUS_ERROR; // a timeout occurred
}

////////////////////////////////////////////////
// API
////////////////////////////////////////////////
/**
 * @brief Initializes and allocates memory for the data-link layer.
 * @param serialPort the filename of the serial port that will be used by the data-link layer
 * @param role string that denotes the role of the application (sender or receiver)
 * @param baudRate the number of symbols (bits) per second that can be transmitted through the serial port
 * @param maxRetransmissions the maximum number of retransmissions for a single frame
 * @param timeout the maximum number of seconds before a timeout occurs
 * @return a pointer to the data-link layer on success, NULL otherwise
 */
LinkLayer *llInit(const char *serialPort, const char *role, int baudRate, int maxRetransmissions, int timeout) {
    // open the serial port
    SerialPort *port = spInit(serialPort, baudRate);

    if (port == NULL) {
        return NULL;
    }

    // validate the role
    if (strcmp(role, "tx") != 0 && strcmp(role, "rx") != 0) {
        printf(RED "Error! Role must be '" BOLD "tx" R_BOLD "' or '" BOLD "rx" R_BOLD "'.\n" RESET);
        return NULL;
    }

    // initialize the link layer
    LinkLayer *ll = (LinkLayer *) malloc(sizeof(LinkLayer));

    ll->sp = port;
    ll->isSender = (role[0] == 't');
    ll->maxRetransmissions = maxRetransmissions;
    ll->timeout = timeout;
    ll->iFrames = 0;
    ll->logbook = fopen(ll->isSender ? "logs_tx.txt" : "logs_rx.txt", "wb");
    
    gettimeofday(&ll->startTime, NULL);
    ll->framesTransmitted = 0;
    ll->framesReceived = 0;
    ll->timeouts = 0;
    ll->dataBytesTransferred = 0;

    return ll;
}

/**
 * @brief Deallocates the memory occupied by the data-link layer.
 * @param ll the data-link layer
 * @return 1 on success, -1 otherwise
 */
int llFree(LinkLayer *ll) {
    // free the serial port
    int statusCode = spFree(ll->sp);
    
    free(ll);
    return statusCode;
}

/**
 * @brief Establishes communication with the other PC.
 * @param ll the link layer
 * @return 1 on success, -1 on error
 */
int llOpen(LinkLayer *ll) {
    int attempt;

    // establish communication with the other PC
    writeLogsHeader(ll, "Connecting...");
    
    for (attempt = 0; attempt < ll->maxRetransmissions; ++attempt) {
        _Bool retransmission = (attempt > 0);
        unsigned char address, control;

        if (ll->isSender) {
            // send the SET frame
            if (sendFrame(ll, ADDRESS_TX_SEND, CONTROL_SET) < 0) {
                logEvent(ll, TRUE, "Failed to send the SET frame.");
                continue;
            }

            logEvent(ll, FALSE, "Sent the SET frame.");
            ll->framesRetransmitted += retransmission;

            // receive the UA frame
            if (receiveFrame(ll, &address, &control, NULL) < 0) {
                logEvent(ll, TRUE, "Failed to receive the UA frame.");
                continue;
            }

            // ensure the UA frame is correct
            if (address != ADDRESS_TX_SEND || control != CONTROL_UA) {
                logEvent(ll, TRUE, "Received a UA frame with errors.");
                ++ll->framesRejected;

                continue;
            }

            logEvent(ll, FALSE, "Received the UA frame.");
        }
        else {
            // receive the SET frame
            if (receiveFrame(ll, &address, &control, NULL) < 0) {
                logEvent(ll, TRUE, "Failed to receive the SET frame.");
                continue;
            }

            // ensure the SET frame is correct
            if (address != ADDRESS_TX_SEND || control != CONTROL_SET) {
                logEvent(ll, TRUE, "Received a UA frame with errors.");
                ++ll->framesRejected;

                continue;
            }

            logEvent(ll, FALSE, "Received the SET frame.");

            // send the UA frame
            if (sendFrame(ll, ADDRESS_TX_SEND, CONTROL_UA) < 0) {
                logEvent(ll, TRUE, "Failed to send the UA frame.");
                continue;
            }

            logEvent(ll, FALSE, "Sent the UA frame.");
            ll->framesRetransmitted += retransmission;
        }

        break; // exit the loop
    }

    // verify if a timeout occurred
    if (attempt == ll->maxRetransmissions) {
        return STATUS_ERROR;
    }

    writeLogsHeader(ll, "Transferring file...");
    return STATUS_SUCCESS;
}

/**
 * @brief Sends an application packet via the serial port.
 * @param ll the data-link layer
 * @param packet the packet to be sent
 * @param packetSize the number of bytes of the packet
 * @return the number of bytes written on success, -1 otherwise
 */
int llWrite(LinkLayer *ll, const unsigned char *packet, int packetSize) {
    _Bool done = FALSE;

    for (int attempt = 0; !done && attempt < ll->maxRetransmissions; ++attempt) {
        _Bool retransmission = (attempt > 0);
        unsigned char address, control;

        // send the packet
        if (sendDataFrame(ll, CONTROL_I(ll->iFrames), packet, packetSize) < 0) {
            logEvent(ll, TRUE, "Failed to send I-frame #%d.", ll->iFrames);
            continue;
        }

        logEvent(ll, FALSE, "Sent I-frame #%d.", ll->iFrames);
        ll->framesRetransmitted += retransmission;

        // receive the UA frame
        if (receiveFrame(ll, &address, &control, NULL) < 0) {
            logEvent(ll, TRUE, "Failed to receive the UA frame.");
            continue;
        }

        // ensure the address of the UA frame is correct
        if (address != ADDRESS_TX_SEND) {
            logEvent(ll, TRUE, "Received a UA frame with errors.");
            ++ll->framesRejected;

            continue;
        }

        // parse the control byte of the UA frame
        switch (control) {
            case CONTROL_RR0:
            case CONTROL_RR1:
                // ensure the receiver is requesting the correct frame
                if (control == CONTROL_RR(ll->iFrames + 1)) {
                    logEvent(ll, FALSE, "Received the UA frame.");
                    done = TRUE;
                }
                else {
                    logEvent(ll, TRUE, "Received a UA frame with errors.");
                    ++ll->framesRejected;
                }

                break;

            case CONTROL_REJ0:
            case CONTROL_REJ1:
                // ensure the receiver is rejecting the correct frame
                if (control == CONTROL_REJ(ll->iFrames)) {
                    logEvent(ll, TRUE, "Sent duplicate I-frame #%d.", ll->iFrames);
                }

            default:
                logEvent(ll, TRUE, "Received a UA frame with errors.");
                ++ll->framesRejected;

                break;
        }
    }

    // verify if a timeout occurred
    if (!done) {
        return STATUS_ERROR;
    }

    ++ll->iFrames; // update the number of I-frames transmitted
    ll->dataBytesTransferred += packetSize; // update the number of data bytes written

    return packetSize;
}

/**
 * @brief Reads an application packet from the serial port.
 * @param ll the data-link layer
 * @param packet buffer which will store the packet read
 * @return the number of bytes read on success, -1 otherwise
 */
int llRead(LinkLayer *ll, unsigned char *packet) {
    int bytesRead;
    _Bool done = FALSE;

    for (int attempt = 0; !done && attempt < ll->maxRetransmissions; ++attempt) {
        _Bool retransmission = (attempt > 0);
        unsigned char address, control;

        // receive the packet
        bytesRead = receiveFrame(ll, &address, &control, packet);

        if (bytesRead < 0) {
            logEvent(ll, TRUE, "Failed to receive an I-frame.");
            continue;
        }

        // ensure the address of the I-frame is correct
        if (address != ADDRESS_TX_SEND) {
            logEvent(ll, TRUE, "Received an I-frame with errors.");
            ++ll->framesRejected;

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
                if (control == CONTROL_I(ll->iFrames)) {
                    done = TRUE;
                }
                else {
                    logEvent(ll, TRUE, "Received duplicate I-frame #%d.", ll->iFrames - 1);
                }

                control = CONTROL_RR(ll->iFrames + 1);
                break;

            default:
                logEvent(ll, TRUE, "Received an I-frame with errors.");
                control = CONTROL_REJ(ll->iFrames);

                ++ll->framesRejected;
                break;
        }

        logEvent(ll, FALSE, "Received I-frame #%d.", ll->iFrames);

        // send the acknowledgement frame
        if (sendFrame(ll, ADDRESS_TX_SEND, control) < 0) {
            logEvent(ll, TRUE, "Failed to send the UA frame.");
            done = FALSE;
        }

        logEvent(ll, FALSE, "Sent the UA frame.");
        ll->framesRetransmitted += retransmission;
    }

    // verify if a timeout occurred
    if (!done) {
        return STATUS_ERROR;
    }

    ++ll->iFrames; // update the number of I-frames received
    ll->dataBytesTransferred += bytesRead; // update the number of data bytes read

    return bytesRead;
}

/**
 * @brief Terminates the connection.
 * @param ll the data-link layer
 * @param showStatistics indicates whether statistics should be shown on closing
 * @return 1 on success, -1 otherwise
 */
int llClose(LinkLayer *ll, int showStatistics){
    int attempt;

    // end communication with the other PC
    writeLogsHeader(ll, "Disconnecting...");

    for (attempt = 0; attempt < ll->maxRetransmissions; ++attempt) {
        unsigned char address, control;
        _Bool retransmission = (attempt > 0);

        if (ll->isSender) {
            // send the DISC frame
            if (sendFrame(ll, ADDRESS_TX_SEND, CONTROL_DISC) < 0) {
                logEvent(ll, TRUE, "Failed to send the DISC frame.");
                continue;
            }

            logEvent(ll, FALSE, "Sent the DISC frame.");
            ll->framesRetransmitted += retransmission;

            // receive the receiver's DISC frame
            if (receiveFrame(ll, &address, &control, NULL) < 0) {
                logEvent(ll, TRUE, "Failed to receive the receiver's DISC frame.");
                continue;
            }

            // ensure the receiver's DISC frame is correct
            if (address != ADDRESS_RX_SEND || control != CONTROL_DISC) {
                logEvent(ll, TRUE, "Received a DISC frame with errors.");
                ++ll->framesRejected;

                continue;
            }

            logEvent(ll, FALSE, "Received the sender's DISC frame.");

            // send the UA frame
            if (sendFrame(ll, ADDRESS_RX_SEND, CONTROL_UA) < 0) {
                logEvent(ll, TRUE, "Failed to send the UA frame." RESET);
                continue;
            }

            logEvent(ll, FALSE, "Sent the UA frame.");
            ll->framesRetransmitted += retransmission;
        }
        else {
            // NOTE: The receiver only needs to receive the sender's DISC frame
            // if 'showStatistics' is set to true. This is because 'showStatistics is
            // only set to false when the sender abruptly terminates the file transferring
            // process by sending a DISC frame before the entire file has been transferred.
            if (showStatistics) {
                // receive the sender's DISC frame
                if (receiveFrame(ll, &address, &control, NULL) < 0) {
                    logEvent(ll, TRUE, "Failed to receive the sender's DISC frame!" RESET);
                    continue;
                }

                // ensure the sender's DISC frame is correct
                if (address != ADDRESS_TX_SEND || control != CONTROL_DISC) {
                    logEvent(ll, TRUE, "Received a DISC frame with errors.");
                    ++ll->framesRejected;

                    continue;
                }

                logEvent(ll, FALSE, "Received the sender's DISC frame.");

                // send the DISC frame
                if (sendFrame(ll, ADDRESS_RX_SEND, CONTROL_DISC) < 0) {
                    logEvent(ll, TRUE, "Failed to send the DISC frame!" RESET);
                    continue;
                }
                
                logEvent(ll, FALSE, "Sent the DISC frame.");
                ll->framesRetransmitted += retransmission;
            }

            // receive the UA frame
            if (receiveFrame(ll, &address, &control, NULL) < 0) {
                logEvent(ll, TRUE, "Failed to receive the UA frame!" RESET);
                continue;
            }

            // ensure the UA frame is correct
            if (address != ADDRESS_RX_SEND || control != CONTROL_UA) {
                logEvent(ll, TRUE, "Received a UA frame with errors.");
                ++ll->framesRejected;

                continue;
            }

            logEvent(ll, FALSE, "Received the sender's UA frame.");
        }

        break; // exit the loop
    }

    // verify if a timeout occurred
    return (attempt < ll->maxRetransmissions)
        ? STATUS_SUCCESS
        : STATUS_ERROR;
}

void llPrintStatistics(LinkLayer *ll) {
    // compute and print the time elapsed
    struct timeval endTime;
    gettimeofday(&endTime, NULL);

    struct timeval timeElapsed;
    timersub(&endTime, &ll->startTime, &timeElapsed);

    time_t minutes = timeElapsed.tv_sec / 60;
    time_t seconds = timeElapsed.tv_sec % 60;
    time_t milliseconds = timeElapsed.tv_usec / 1000;

    printf("  " BOLD "- Time:" RESET);

    if (minutes > 0) {
        printf(" %ldmin", minutes);
    }
    
    printf(" %lds %ldms\n", seconds, milliseconds);

    // print the remaining statistics
    double FER = (double) (ll->framesRetransmitted + ll->framesRejected)
                 / (double) (ll->framesTransmitted + ll->framesReceived);

    double capacity = (double) (ll->dataBytesTransferred * 8)              // the number of data bits transferred
                      / ((double) seconds + (double) milliseconds / 1000); // the number of seconds it took to transfer the data bits
    double efficiency = capacity / (double) ll->sp->baudRate;

    printf("  " BOLD "- Frames transmitted:" RESET " %d\n", ll->framesTransmitted);
    printf("  " BOLD "- Frames retransmitted:" RESET " %d\n", ll->framesRetransmitted);
    printf("  " BOLD "- Frames received:" RESET " %d\n", ll->framesReceived);
    printf("  " BOLD "- Frames rejected:" RESET " %d\n", ll->framesRejected);
    printf("  " BOLD "- Frame Error Ratio (FER):" RESET " %.2f%%\n", 100 * FER);
    printf("  " BOLD "- Timeouts:" RESET " %d\n", ll->timeouts);
    printf("  " BOLD "- Capacity:" RESET " %.1f bit/s\n", capacity);
    printf("  " BOLD "- Efficiency:" RESET " %.2f%%\n", 100 * efficiency);
}
