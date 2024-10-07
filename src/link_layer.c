// Link layer protocol implementation

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>

#include "../include/console.h"
#include "../include/link_layer.h"
#include "../include/serial_port.h"

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

/* global variables */
// link layer
_Bool isSender;
int timeout;

// alarm
_Bool alarmIsEnabled;
int nRetransmissions;

// loading screen
volatile _Bool loading;

static void handleAlarm() {
    alarmIsEnabled = FALSE;
}

static void setAlarm(int time) {
    (void) signal(SIGALRM, handleAlarm);
    alarm(time);
    alarmIsEnabled = TRUE;
}

static void stopLoading() {
    loading = FALSE;
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

    // create a new process to display a loading screen
    int pid = fork(); // the process ID of the child process
    
    if (pid == 0) {
        // set up a signal handler to stop the loading screen
        signal(SIGUSR1, stopLoading);

        // initialize the loading screen
        loading = TRUE;
        loadingScreen("\n> Establishing connection");

        exit(EXIT_SUCCESS);
    }
    // if the child process could not be created, simply output the message
    else if (pid < 0) {
        printf("\n> Establishing connection...\n");
    }

    // establish communication with the other PC
    int attempt;

    for (attempt = 0; attempt < nRetransmissions; ++attempt) {
        if (attempt > 0) {
            //printf(" Trying again...\n");
        }

        if (isSender) {
            // send the SET frame
            if (sendFrame(ADDRESS_TX_SEND, CONTROL_SET) < 0) {
                //printf(FAINT "Failed to send the SET frame!");
                continue;
            }

            // receive the UA frame
            if (receiveFrame(ADDRESS_TX_SEND, CONTROL_UA, NULL) < 0) {
                //printf(FAINT "  Failed to receive the UA frame!");
                continue;
            }
        }
        else {
            // receive the SET frame
            if (receiveFrame(ADDRESS_TX_SEND, CONTROL_SET, NULL) < 0) {
                //printf(FAINT "  Failed to receive the SET frame!");
                continue;
            }

            // send the UA frame
            if (sendFrame(ADDRESS_TX_SEND, CONTROL_UA) < 0) {
                //printf(FAINT "  Failed to send the UA frame!");
                continue;
            }
        }

        break;
    }

    // terminate the loading screen
    if (pid > 0) {
        kill(pid, SIGUSR1);
        wait(NULL);
    }

    // ensure communication was established
    const char *otherPC = isSender ? "receiver" : "sender";

    if (attempt == nRetransmissions) {
        printf(RED "  Failed to establish connection with the %s!\n" RESET, otherPC);
        return -1;
    }

    printf(GREEN "  Connection established!\n" RESET);
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
int llclose(int showStatistics){
    // end communication with the other PC
    const char *otherPC = isSender ? "receiver" : "sender";
    int attempt;

    printf("\n> Terminating the connection with the %s...\n", otherPC);

    for (attempt = 0; attempt < nRetransmissions; ++attempt) {
        if (attempt > 0) {
            printf(FAINT " Trying again...\n" RESET);
        }

        if (isSender) {
            // send the DISC frame
            if (sendFrame(ADDRESS_TX_SEND, CONTROL_DISC) < 0) {
                printf(FAINT "  Failed to send the DISC frame!" RESET);
                continue;
            }

            // receive the receiver's DISC frame
            if (receiveFrame(ADDRESS_RX_SEND, CONTROL_DISC, NULL) < 0) {
                printf(FAINT "  Failed to receive the receiver's DISC frame!" RESET);
                continue;
            }

            // send the UA frame
            if (sendFrame(ADDRESS_RX_SEND, CONTROL_UA) < 0) {
                printf(FAINT "  Failed to send the UA frame!" RESET);
                continue;
            }
        }
        else {
            // receive the sender's DISC frame
            if (receiveFrame(ADDRESS_TX_SEND, CONTROL_DISC, NULL) < 0) {
                printf(FAINT "  Failed to receive the sender's DISC frame!" RESET);
                continue;
            }

            // send the DISC frame
            if (sendFrame(ADDRESS_RX_SEND, CONTROL_DISC) < 0) {
                printf(FAINT "  Failed to send the DISC frame!" RESET);
                continue;
            }

            // receive the UA frame
            if (receiveFrame(ADDRESS_RX_SEND, CONTROL_UA, NULL) < 0) {
                printf(FAINT "  Failed to receive the UA frame!\n" RESET);
                continue;
            }
        }

        break;
    }

    // ensure the connection was terminated
    if (attempt == nRetransmissions) {
        printf(RED "\n  Error! Failed to terminate the connection with the %s!\n" RESET, otherPC);
        return -1;
    }

    printf(GREEN "Connection terminated!\n" RESET);
    return closeSerialPort();
}
