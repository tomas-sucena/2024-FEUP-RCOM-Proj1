// Application layer protocol implementation

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../include/application_layer.h"
#include "../include/utils.h"

#define CONTROL_START 1
#define CONTROL_DATA  2
#define CONTROL_END   3

#define TYPE_FILE_SIZE 0
#define TYPE_FILENAME  1
#define TYPE_DATA_SIZE 2

#define PROGRESS_BAR_SIZE 40

/**
 * @brief Returns the minimum number of bytes needed to represent a number.
 * @param number the number
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

/**
 * @brief Appends a number to a string.
 * @param str the string where the number will be appended
 * @param number the number
 * @param numDigits the minimum number of digits needed to write the number
 */
static void writeNumber(unsigned char *str, long number, unsigned char numDigits) {
    for (unsigned char index = numDigits; index > 0; ) {
        // append the least significant byte of the number
        str[--index] = (unsigned char) (number & 0xFF);

        // shift the number one byte to the right
        number >>= 8;
    }
}

/**
 * @brief Reads a number from a string.
 * @param str the string containing the number
 * @param numDigits the number of digits to be read
 * @return the number read
 */
static long readNumber(const unsigned char *str, unsigned char numDigits) {
    long number = 0;

    for (unsigned char index = 0; index < numDigits; ++index) {
        // shift the number one byte to the left
        // to make space for the next byte
        number <<= 8;

        // copy the byte to the least significant byte of the number
        number |= str[index];
    }

    return number;
}

/**
 * @brief Displays a progress bar on the console.
 * @param numBytesTransferred the number of data bytes that have been transferred
 * @param fileSize the size of the file to be transferred
 * @param eraseLine indicates if the current line should be erased before printing the progress bar
 */
static void printProgress(long numBytesTransferred, long fileSize, _Bool eraseLine) {
    // compute the percentage of the file that has been transferred
    double percentage = 100 * (double) numBytesTransferred / (double) fileSize;

    // compute the number of spaces in the progress bar that should be filled
    int numFilledSpaces = (int) percentage * PROGRESS_BAR_SIZE / 100;    

    // initialize and fill the progress bar
    char progressBar[PROGRESS_BAR_SIZE + 1];
    progressBar[PROGRESS_BAR_SIZE] = '\0';
    
    memset(progressBar, '=', numFilledSpaces * sizeof(char));
    memset(progressBar + numFilledSpaces, '-', (PROGRESS_BAR_SIZE - numFilledSpaces) * sizeof(char));

    // erase the previous line, if needed
    if (eraseLine) {
        printf(LINE_UP ERASE_LINE);
    }

    // print the progress bar
    printf("\r[%s] %.2f%% | %ld/%ld B\n", progressBar, percentage, numBytesTransferred, fileSize);
}

////////////////////////////////////////////////
// SENDER
////////////////////////////////////////////////
/**
 * @brief Creates a control packet and sends it via the serial port.
 * @param app the application
 * @param control the control byte of the control packet
 * @return 1 on success, -1 otherwise
 */
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

/**
 * @brief Creates a data packet and sends it via the serial port.
 * @param app the application layer
 * @return 1 on success, -1 otherwise
 */
static int sendDataPackets(ApplicationLayer *app) {
    // initialize the buffer that will store the data packets
    unsigned char *dataPacket = (unsigned char *) malloc((3 + app->dataSize) * sizeof(unsigned char));

    if (dataPacket == NULL) {
        printf(RED "Error! Failed to allocate memory for the data packets." RESET "\n");
        llClose(app->ll, FALSE); // notify the receiver that no data will be sent

        return STATUS_ERROR;
    }

    // write the control byte
    dataPacket[0] = CONTROL_DATA;
    
    // send the data packets
    int statusCode = STATUS_SUCCESS;
    long totalBytesWritten = 0;

    do {
        // print the current progress
        printProgress(totalBytesWritten, app->fileSize, totalBytesWritten > 0);

        // read data from the file
        int numBytesRead = (int) fread(dataPacket + 3, sizeof(unsigned char), app->dataSize, app->file);

        // ensure no errors occurred when reading from the file
        if (ferror(app->file)) {
            printf(RED "Error! Failed to read from '" BOLD "%s" R_BOLD "'.\n" RESET, app->filename);
            llClose(app->ll, FALSE); // notify the receiver that no more data will be sent

            statusCode = STATUS_ERROR;
            break;
        }

        // verify if we have reached the end of the file
        if (numBytesRead == 0) {
            break;
        }

        // write the number of data bytes
        dataPacket[1] = (unsigned char) (numBytesRead >> 8);   // the most significant byte of the data size
        dataPacket[2] = (unsigned char) (numBytesRead & 0xFF); // the least significant byte of the data size

        // send the data via the serial port
        if (llWrite(app->ll, dataPacket, 3 + numBytesRead) < 0) {
            printf(RED "Error! Failed to send data via the serial port.\n" RESET);            
            statusCode = STATUS_ERROR;
        }

        totalBytesWritten += numBytesRead;
    }
    while (statusCode == STATUS_SUCCESS);

    // free the buffer that stored the data packets
    free(dataPacket);
    
    return statusCode;
}

/**
 * @brief Sends a file via the serial port.
 * @param app the application layer
 * @return 1 on success, -1 otherwise
 */
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

    return STATUS_SUCCESS;
}

////////////////////////////////////////////////
// RECEIVER
////////////////////////////////////////////////
/**
 * @brief Receives and parses a data packet sent via the serial port.
 * @param app the application layer
 * @return 1 on success, -1 otherwise
 */
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
                app->dataSize = (int) readNumber(packet + index, L);
                break;

            default:
                printf(RED "Error! Received badly formatted packet.\n" RESET);
                return STATUS_ERROR;
        }

        index += L;
    }

    return STATUS_SUCCESS;
}

/**
 * @brief Receives and parses a data packet sent via the serial port.
 * @param app the application layer
 * @return 1 on success, -1 otherwise
 */
static int receiveDataPackets(ApplicationLayer *app) {
    // initialize the buffer that will store the data packets
    unsigned char *dataPacket = (unsigned char *) malloc((3 + app->dataSize) * sizeof(unsigned char));
    
    if (dataPacket == NULL) {
        printf(RED "Error! Failed to allocate memory for the data packets." RESET "\n");
        return STATUS_ERROR;
    }

    // receive the data packets
    _Bool done = FALSE;
    long totalBytesRead = 0;

    do {
        // print the current progress
        printProgress(totalBytesRead, app->fileSize, totalBytesRead > 0);

        // receive data from the serial port
        int numBytesRead = llRead(app->ll, dataPacket);

        if (numBytesRead < 0) {
            printf(RED "Error! Failed to receive data from the serial port.\n" RESET);
            break;
        }
        
        // verify if the sender requested to disconnect
        if (numBytesRead == 0) {
            printf(YELLOW "Warning! The sender requested to disconnect.\n" RESET);
            llClose(app->ll, FALSE);

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
            printf(RED "Error! Failed to write to '" BOLD "%s" R_BOLD "'.\n" RESET, app->filename);
            break;
        }

        totalBytesRead += dataSize;
    }
    while (!done);

    // free the buffer that stored the data packets
    free(dataPacket);

    return done
        ? STATUS_SUCCESS
        : STATUS_ERROR;
}

/**
 * @brief Receives a file sent via the serial port.
 * @param app the application layer
 * @return 1 on success, -1 otherwise
 */
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
    if (receiveDataPackets(app) < 0) {
        return STATUS_ERROR;
    }

    // close the file
    int statusCode = STATUS_SUCCESS;

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
/**
 * @brief Initializes and allocates memory for the application.
 * @param serialPort the filename of the serial port that will be used throughout the application
 * @param role string that denotes the role of the application (sender or receiver)
 * @param baudRate the baud rate (in bits/s) at which the serial port will transmit data
 * @param maxRetransmissions the maximum number of retransmissions for a single frame
 * @param timeout the maximum number of seconds before a timeout occurs
 * @param filepath the path to the file to be transferred
 * @param dataSize the maximum number of data bytes that will be sent in each data packet
 * @return a pointer to the application on success, NULL otherwise
 */
ApplicationLayer *appInit(const char *serialPort, const char *role, int baudRate, int maxRetransmissions, int timeout,
                          const char *filepath, int dataSize) {
    // initialize the link layer
    LinkLayer *ll = llInit(serialPort, role, baudRate, maxRetransmissions, timeout);

    if (ll == NULL) {
        return NULL;
    }

    // ensure that, if the program is the sender,
    // the file to be transferred exists
    _Bool isSender = ll->isSender;

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
            printf(RED "Error! Could not open '" BOLD "%s" R_BOLD "'.\n" RESET, filepath);
            return NULL;
        }
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

/**
 * @brief Deallocates the memory occupied by the application.
 * @param app the application layer
 * @return 1 on success, -1 otherwise
 */
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

/**
 * @brief Runs the application.
 * @param app the application
 * @return 1 on success, -1 otherwise
 */
int appRun(ApplicationLayer *app) {
    const char *otherPC;
    int (*transferFunction)(ApplicationLayer*);

    if (app->ll->isSender) {
        otherPC = "receiver";
        transferFunction = sendFile;
    }
    else {
        otherPC = "sender";
        transferFunction = receiveFile;
    }

    // establish communication with the other PC
    printf("\n> Connecting to the %s...\n", otherPC);

    if (llOpen(app->ll) < 0) {
        printf(RED "Error! Failed to connect to the %s.\n" RESET, otherPC);
        return STATUS_ERROR;
    }

    printf(GREEN "Success!\n" RESET);

    // transfer the file between PCs
    if (transferFunction(app) < 0) {
        return STATUS_ERROR;
    }

    printf(GREEN "Success!\n" RESET);    

    // terminate the connection
    printf("\n> Disconnecting...\n");

    if (llClose(app->ll, TRUE) < 0) {
        printf(RED "\nError! Failed to terminate the connection with the %s.\n" RESET, otherPC);
        return STATUS_ERROR;
    }

    printf(GREEN "Success!\n" RESET);
    return STATUS_SUCCESS;
}
