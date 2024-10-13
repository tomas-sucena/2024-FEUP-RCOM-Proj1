// Application layer protocol implementation

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../include/application_layer.h"
#include "../include/link_layer.h"
#include "../include/utils.h"

#define DATA_SIZE 5

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

/* SENDER */
static int sendControlPacket(unsigned char control, const char *filename, long fileSize, int dataSize) {
    // compute the size of the packet
    unsigned char L_fileSize = countBytes(fileSize);
    unsigned char L_filename = strlen(filename);
    unsigned char L_dataSize = countBytes(dataSize);

    int packetSize = 7 + L_fileSize + L_filename + L_dataSize;

    // initialize the packet
    unsigned char packet[packetSize], *ptr = packet;
    *ptr++ = control;

    // write the TLV (Type-Length-Value) corresponding to the file size
    *ptr++ = TYPE_FILE_SIZE;
    *ptr++ = L_fileSize;

    memcpy(ptr, &fileSize, L_fileSize * sizeof(unsigned char));
    ptr += L_fileSize;

    // write the TLV corresponding to the file name
    *ptr++ = TYPE_FILENAME;
    *ptr++ = L_filename;

    memcpy(ptr, filename, L_filename * sizeof(unsigned char));
    ptr += L_filename;

    // write the TLV corresponding to the size of each data packet
    *ptr++ = TYPE_DATA_SIZE;
    *ptr++ = L_dataSize;

    memcpy(ptr, &dataSize, L_dataSize * sizeof(unsigned char));

    // send the packet
    return llwrite(packet, packetSize);
}

static int sendFile(const char *filename) {
    // open the file
    FILE *file = fopen(filename, "rb");
    
    if (file == NULL) {
        printf(RED "\nError! Failed to open '" BOLD "%s" RESET RED "'.\n" RESET, filename);
        return -1;
    }

    // determine the size of the file
    fseek(file, 0, SEEK_END); // move the file pointer to the end of the file
    long fileSize = ftell(file);

    rewind(file); // return the file pointer to the beginning of the file

    // send the initial control packet  
    return sendControlPacket(CONTROL_START, filename, fileSize, DATA_SIZE);
}

/* RECEIVER */
static int receiveControlPacket(char *filename, long *fileSize, int *dataSize) {
    // receive data from the serial port
    unsigned char packet[MAX_PAYLOAD_SIZE];
    int packetSize = llread(packet);
    
    if (packetSize <= 0) {
        printf(RED "Error! Failed to receive data from the serial port.\n" RESET);
        return -1;
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
            return -1;
        }

        void *ptr;

        switch (T) {
            // parse the file size
            case TYPE_FILE_SIZE:
                ptr = fileSize;
                break;

            // parse the filename
            case TYPE_FILENAME:
                ptr = filename;
                break;

            // parse the size of a data packet
            case TYPE_DATA_SIZE:
                ptr = dataSize;
                break;

            default:
                printf(RED "Error! Received badly formatted packet.\n" RESET);
                return -1;
        }

        memcpy(ptr, packet + index, L * sizeof(unsigned char));
        index += L;
    }

    return 1;
}

static int receiveFile(const char *filename) {
    long fileSize;
    int packetSize;

    // receive the initial control packet
    printf("\n> Receiving file...\n");

    if (receiveControlPacket(filename, &fileSize, &packetSize) < 0) {
        return -1;
    }

    // create the file
    FILE *file = fopen(filename, "wb");

    if (file == NULL) {
        printf(RED "Error! Failed to create '%s'.", filename);
        return -1;
    }

    return 1;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    _Bool isSender = (role[0] == 't');

    // configure the connection
    LinkLayer connectionParameters = {
        .role = LlRx ^ isSender,
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout
    };
    memcpy(connectionParameters.serialPort, serialPort, 50);

    // establish communication with the other PC
    llopen(connectionParameters);

    if (isSender) {
        sendFile(filename);
    }
    else {
        receiveFile(filename);
    }

    llclose(FALSE);
}
