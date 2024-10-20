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
 * @brief Returns the minimum number of bytes needed to represent a number.
 * @param number an integer
 * @return the minimum number of bytes needed to represent the number
 */
static unsigned char countBytes(long number) {
    unsigned char bits = 0;

    for (; number; number >>= 1) {
        ++bits;
    }

    return 1 + (bits - 1) / 8; // NOTE: 1B = 8b
}

/**
 * @brief Extracts the filename from a path to a file.
 * 
 * @param filepath the path to a file
 * @return the filename
 */
static char *getFilename(const char *filepath) {
    if (filepath == NULL) {
        return NULL;
    }

    // find the last occurrence of the '/' character
    const char *slash = strrchr(filepath, '/');

    if (slash == NULL) {
        slash = filepath;
        --slash;
    }

    // initialize the filename
    char *filename = (char *) malloc(strlen(slash) * sizeof(char));
    strcpy(filename, slash + 1);

    return filename;
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

static void writeNumber(unsigned char *data, long number, unsigned char numBytes) {
    for (unsigned char index = numBytes; index > 0; ) {
        // append the least significant byte of the number
        data[--index] = (unsigned char) (number & 0xFF);

        // shift the number one byte to the right
        number >>= 8;
    }
}

static long readNumber(unsigned char *data, unsigned char numBytes) {
    long number = 0;

    for (unsigned char index = 0; index < numBytes; ++index) {
        // shift the number one byte to the left
        // to make space for the next byte
        number <<= 8;

        // copy the byte to the least significant byte of the number
        number |= data[index];
    }

    return number;
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

    writeNumber(ptr, app->fileSize, L_fileSize);
    ptr += L_fileSize;

    // write the TLV corresponding to the file name
    *ptr++ = TYPE_FILENAME;
    *ptr++ = L_filename;

    memcpy(ptr, app->filename, L_filename * sizeof(unsigned char));
    ptr += L_filename;

    // write the TLV corresponding to the size of each data packet
    *ptr++ = TYPE_DATA_SIZE;
    *ptr++ = L_dataSize;

    writeNumber(ptr, app->dataSize, L_dataSize);

    // send the packet via the serial port
    if (llWrite(app->ll, packet, packetSize) == packetSize) {
        return STATUS_SUCCESS;
    }

    // an error occurred in the link layer
    printf(RED "Error! Failed to send data via the serial port.\n" RESET);
    return STATUS_ERROR;
}

static int sendDataPackets(ApplicationLayer *app) {
    // initialize the buffer that will store the data packets
    unsigned char *dataPacket = (unsigned char *) malloc((3 + app->dataSize) * sizeof(unsigned char));

    if (dataPacket == NULL) {
        printf(RED "Error! Failed to allocate memory for the data packets." RESET "\n");
        return STATUS_ERROR;
    }

    // write the control byte
    dataPacket[0] = CONTROL_DATA;
    
    // send the data packets
    int statusCode = STATUS_SUCCESS;

    while (statusCode == STATUS_SUCCESS) {
        // read data from the file
        int bytesRead = (int) fread(dataPacket + 3, sizeof(unsigned char), app->dataSize, app->file);

        // verify if we have reached the end of the file
        if (bytesRead == 0) {
            break;
        }

        // write the number of data bytes
        dataPacket[1] = (unsigned char) (bytesRead >> 8);   // the most significant byte of the data size
        dataPacket[2] = (unsigned char) (bytesRead & 0xFF); // the least significant byte of the data size

        // send the data via the serial port
        int bytesWritten = llWrite(app->ll, dataPacket, 3 + bytesRead);

        if (bytesWritten < 0) {
            printf(RED "Error! Failed to receive data from the serial port.\n" RESET);            
            statusCode = STATUS_ERROR;
        }
    }

    // free the buffer that stored the data packets
    free(dataPacket);
    
    return statusCode;
}

static int sendFile(ApplicationLayer *app) {
    printf("\n> Sending '" BOLD "%s" RESET "'...\n", app->filename);

    // send the initial control packet
    if (sendControlPacket(app, CONTROL_START) < 0) {
        return STATUS_ERROR;
    }

    // send the data packets
    if (sendDataPackets(app) < 0) {
        return STATUS_ERROR;
    }

    // send the final control packet
    if (sendControlPacket(app, CONTROL_END) < 0) {
        return STATUS_ERROR;
    }

    printf(GREEN "Success!\n" RESET);
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
        unsigned char T = packet[index++];
        unsigned char L = packet[index++];

        // ensure there is no overflow
        if (index + L > packetSize) {
            printf(RED "Error! Received badly formatted packet.\n" RESET);
            return STATUS_ERROR;
        }

        switch (T) {
            // parse the file size
            case TYPE_FILE_SIZE:
                app->fileSize = readNumber(packet + index, L);
                break;

            // parse the filename
            case TYPE_FILENAME: {
                char filename[L + 1];
                
                memcpy(filename, packet + index, L * sizeof(char));
                filename[L] = '\0';

                // if the filename has not been set yet,
                // assign it the received filename
                if (app->filename == NULL) {
                    app->filename = (char *) malloc((L + 1) * sizeof(char));
                    strcpy(app->filename, filename);
                }

                printf("\n> Receiving '" BOLD "%s" RESET "'...\n", filename);
                break;
            }

            // parse the maximum number of data bytes that will be sent in a data packet
            case TYPE_DATA_SIZE:
                app->dataSize = readNumber(packet + index, L);
                break;

            default:
                printf(RED "Error! Received badly formatted packet.\n" RESET);
                return STATUS_ERROR;
        }

        index += L;
    }

    return STATUS_SUCCESS;
}

static int receiveDataPackets(ApplicationLayer *app) {
    // initialize the buffer that will store the data packets
    unsigned char *dataPacket = (unsigned char *) malloc((3 + app->dataSize) * sizeof(unsigned char));
    
    if (dataPacket == NULL) {
        printf(RED "Error! Failed to allocate memory for the data packets." RESET "\n");
        return STATUS_ERROR;
    }

    // receive the data packets
    _Bool done = FALSE;

    while (!done) {
        // receive data from the serial port
        int bytesRead = llRead(app->ll, dataPacket);

        if (bytesRead < 0) {
            printf(RED "Error! Failed to receive data from the serial port.\n" RESET);            
            break;
        }

        // parse the control byte
        switch (dataPacket[0]) {
            case CONTROL_DATA:
                break;

            default:
                printf(RED "Error! Received unexpected data packet.\n" RESET);

            case CONTROL_END:
                done = TRUE;
                continue;
        }

        // compute the number of data bytes received
        int dataSize = (dataPacket[1] << 8) | dataPacket[2];

        // append the data to the file
        if (fwrite(dataPacket + 3, sizeof(unsigned char), dataSize, app->file) < dataSize) {
            printf(RED "Error! Failed to write to '" BOLD "%s" RESET RED "'.\n" RESET, app->filename);
            return STATUS_ERROR;
        }
    }

    // free the buffer that stored the data packets
    free(dataPacket);

    return done
        ? STATUS_SUCCESS
        : STATUS_ERROR;
}

static int receiveFile(ApplicationLayer *app) {
    // receive the initial control packet
    if (receiveControlPacket(app) < 0) {
        return STATUS_ERROR;
    }

    // open the file, if that hasn't yet been done
    if (app->file == NULL) {
        app->file = fopen(app->filename, "wb");

        // ensure the file was properly opened
        if (app->file == NULL) {
            printf(RED "Error! Failed to open '%s'.\n", app->filename);
            return STATUS_ERROR;
        }
    }

    // receive the data packets
    int statusCode = receiveDataPackets(app);

    // close the file
    if (fclose(app->file) < 0) {
        printf(RED "Error! Failed to close '" BOLD "%s" RESET "'.\n" RESET, app->filename);
        statusCode = STATUS_ERROR;
    }

    app->file = NULL;
    return statusCode;
}

////////////////////////////////////////////////
// API
////////////////////////////////////////////////
ApplicationLayer *appInit(const char *serialPort, _Bool isSender, int baudRate, int nTries, int timeout,
    const char *filepath, int dataSize) {
    // ensure that, if the program is the sender,
    // the file to be transferred exists
    if (isSender && filepath == NULL) {
        printf(RED "Error! The sender must provide a file to be transferred.\n" RESET);
        return NULL;
    }

    // open the file
    FILE *file = NULL;

    if (filepath) {
        file = fopen(filepath, isSender ? "rb" : "wb");

        // ensure the file was properly opened
        if (file == NULL) {
            printf(RED "Error! Could not open '" BOLD "%s" RESET RED "'.\n" RESET, filepath);
            return NULL;
        }
    }

    // initialize the link layer
    LinkLayer *ll = llInit(serialPort, isSender, baudRate, nTries, timeout);

    if (ll == NULL) {
        fclose(file);
        return NULL;
    }

    // initialize the application
    ApplicationLayer *app = malloc(sizeof(ApplicationLayer));

    app->ll = ll;
    app->file = file;
    app->filename = getFilename(filepath);
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

    // free the filename
    free(app->filename);

    // free the application layer
    free(app);

    return statusCode;
}

int appRun(ApplicationLayer *app) {
    _Bool isSender = app->ll->isSender;

    // establish communication with the other PC
    const char *otherPC = isSender ? "receiver"
                                   : "sender";

    printf("\n> Connecting to the %s...\n", otherPC);

    if (llOpen(app->ll) < 0) {
        printf(RED "Error! Failed to connect to the %s.\n" RESET, otherPC);
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
        printf(RED "\nError! Failed to terminate the connection with the %s.\n" RESET, otherPC);
        return STATUS_ERROR;
    }

    printf(GREEN "Success!\n" RESET);
    return STATUS_SUCCESS;
}
