// Application layer protocol implementation

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../include/application_layer.h"
#include "../include/link_layer.h"
#include "../include/utils.h"

#define CONTROL_START 1
#define CONTROL_DATA  2
#define CONTROL_END   3

#define TYPE_FILE_SIZE 0
#define TYPE_FILENAME  1
#define TYPE_DATA_SIZE 2

/**
 * @brief Returns the minimum number of bytes needed to represent a value.
 * @param value an integer value
 * @return the minimum number of bytes needed to represent the value
 */
static unsigned char countBytes(long value) {
    unsigned char bits = 0;

    for (; value; value >>= 1) {
        ++bits;
    }

    return bits / 8; // NOTE: 1B = 8b
}

/**
 * @brief Determines the size of a file
 * 
 * @param file pointer to the file
 * @return the size of the file
 */
static long getFileSize(FILE *file) {
    // move the file pointer to the end of the file
    fseek(file, 0L, SEEK_END);
    
    // compute the size of the file
    long fileSize = ftell(file);

    // return the file pointer to the beginning of the file
    rewind(file);
    
    return fileSize;
}

////////////////////////////////////////////////
// SENDER
////////////////////////////////////////////////
static int sendControlPacket(ApplicationLayer *app, unsigned char control) {
    // compute the size of the packet
    unsigned char L_fileSize = countBytes(app->fileSize);
    unsigned char L_filename = strlen(app->filename);
    unsigned char L_dataSize = countBytes(app->dataSize);

    int packetSize = 7 + L_fileSize + L_filename + L_dataSize;

    // initialize the packet
    unsigned char packet[MAX_PAYLOAD_SIZE], *ptr = packet;
    *ptr++ = control;

    // write the TLV (Type-Length-Value) corresponding to the file size
    *ptr++ = TYPE_FILE_SIZE;
    *ptr++ = L_fileSize;

    memcpy(ptr, &app->fileSize, L_fileSize * sizeof(unsigned char));
    ptr += L_fileSize;

    // write the TLV corresponding to the file name
    *ptr++ = TYPE_FILENAME;
    *ptr++ = L_filename;

    memcpy(ptr, app->filename, L_filename * sizeof(unsigned char));
    ptr += L_filename;

    // write the TLV corresponding to the size of each data packet
    *ptr++ = TYPE_DATA_SIZE;
    *ptr++ = L_dataSize;

    memcpy(ptr, &app->dataSize, L_dataSize * sizeof(unsigned char));

    // send the packet
    return (llWrite(app->ll, packet, packetSize) == packetSize)
        ? STATUS_SUCCESS
        : STATUS_ERROR;
}

static int sendFile(ApplicationLayer *app) {
    // send the initial control packet
    if (sendControlPacket(app, CONTROL_START) < 0) {
        return STATUS_ERROR;
    }

    return STATUS_SUCCESS;
}

////////////////////////////////////////////////
// RECEIVER
////////////////////////////////////////////////
static int receiveControlPacket(ApplicationLayer *app) {
    // receive data from the serial port
    unsigned char packet[MAX_PAYLOAD_SIZE];
    int packetSize = llRead(app->ll, packet);
    
    if (packetSize <= 0) {
        printf(RED "Error! Failed to receive data from the serial port.\n" RESET);
        return STATUS_ERROR;
    }

    // ensure the packet received is the initial control packet
    if (packet[0] != CONTROL_START) {
        printf(RED "Error! Received unexpected packet.\n" RESET);
        return STATUS_ERROR;
    }

    // parse the control packet    
    int index = 1;

    while (index < packetSize) {
        // parse the Type and Length fields
        unsigned T = packet[index++];
        unsigned L = packet[index++];

        // ensure there is no overflow
        if (index + L > packetSize) {
            printf(RED "Error! Received badly formatted packet.\n" RESET);
            return STATUS_ERROR;
        }

        void *ptr = NULL;

        switch (T) {
            // parse the file size
            case TYPE_FILE_SIZE:
                ptr = &app->fileSize;
                break;

            // parse the filename
            case TYPE_FILENAME:
                // ptr = filename;
                break;

            // parse the size of a data packet
            case TYPE_DATA_SIZE:
                ptr = &app->dataSize;
                break;

            default:
                printf(RED "Error! Received badly formatted packet.\n" RESET);
                return STATUS_ERROR;
        }

        memcpy(ptr, packet + index, L * sizeof(unsigned char));
        index += L;
    }

    return STATUS_SUCCESS;
}

static int receiveFile(ApplicationLayer *app) {
    // receive the initial control packet
    if (receiveControlPacket(app) < 0) {
        return STATUS_ERROR;
    }

    // create the file
    FILE *file = fopen(app->filename, "wb");

    if (file == NULL) {
        printf(RED "Error! Failed to open '%s'.\n", app->filename);
        return STATUS_ERROR;
    }

    // initialize the buffer where the data packets will be stored
    unsigned char *packet = malloc(app->dataSize * sizeof(unsigned char));

    if (packet == NULL) {
        printf(RED "Error! Failed to allocate memory for the data packet.\n" RESET);
        return STATUS_ERROR;
    }

    // receive the data packets
    int statusCode = STATUS_SUCCESS;

    for (;;) {
        int bytesReceived = llRead(app->ll, packet);

        // ensure the packet was properly received
        if (bytesReceived < 0) {
            printf(RED "Error! Failed to receive packet.\n" RESET);
            statusCode = STATUS_ERROR;

            break;
        }


    }

    // close the file
    fclose(file);

    // free the packet
    free(packet);

    return statusCode;
}

////////////////////////////////////////////////
// API
////////////////////////////////////////////////
ApplicationLayer *appInit(const char *serialPort, _Bool isSender, int baudRate, int nTries, int timeout,
    const char *filepath, int dataSize) {
    // initialize the link layer
    LinkLayer *ll = llInit(serialPort, isSender, baudRate, nTries, timeout);

    if (ll == NULL) {
        return NULL;
    }

    // ensure that, if the program is the sender,
    // the file to be transferred exists
    if (isSender && filepath == NULL) {
        printf(RED "Error! The sender must provide a file to be transferred.\n" RESET);
        return NULL;
    }

    // verify if the file can be opened
    FILE *file = NULL;

    if (filepath) {
        file = fopen(filepath, isSender ? "rb" : "wb");

        if (file == NULL) {
            printf(RED "Error! Could not open '" BOLD "%s" RESET RED "'.\n" RESET, filepath);
            return NULL;
        }
    }

    // initialize the application
    ApplicationLayer *app = malloc(sizeof(ApplicationLayer));

    app->ll = ll;
    app->file = file;
    app->filename = strrchr(filepath, '/');
    app->filepath = filepath;
    app->dataSize = dataSize;

    // determine the size of the file
    if (isSender) {
        app->fileSize = getFileSize(file);
    }

    return app;
}

int appFree(ApplicationLayer *app) {
    // free the link layer
    int statusCode = llFree(app->ll);

    // close the file
    if (app->file && fclose(app->file) < 0) {
        statusCode = STATUS_ERROR;
    }

    return statusCode;
}

int appRun(ApplicationLayer *app) {
    _Bool isSender = app->ll->isSender;

    // establish communication with the other PC
    const char *otherPC = isSender
        ? "receiver"
        : "sender";

    printf("\n> Connecting to the %s...\n", otherPC);

    if (llOpen(app->ll) < 0) {
        printf(RED "Error! Failed to connect to the %s.\n" RESET
            "\n> Aborting...\n", otherPC);
        return STATUS_ERROR;
    }

    printf(GREEN "Success!\n" RESET);

    // transfer the file between PCs
    isSender
        ? sendFile(app)
        : receiveFile(app);

    // terminate the connection
    printf("\n> Disconnecting...\n");

    if (llClose(app->ll, TRUE) < 0) {
        printf(RED "\nError! Failed to terminate the connection with the %s.\n" RESET
            "\n> Aborting...\n", otherPC);
        return STATUS_ERROR;
    }

    printf(GREEN "Success!\n" RESET);
    return STATUS_SUCCESS;
}
