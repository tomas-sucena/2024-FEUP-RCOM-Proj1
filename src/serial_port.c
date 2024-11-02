// Serial port interface implementation

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../include/serial_port.h"
#include "../include/utils.h"

#define _POSIX_SOURCE 1 // POSIX compliant source

/**
 * @brief Configures and opens the serial port.
 * @param filename the filename of the serial port
 * @param baudRate the baud rate (in bits/s) at which the serial port will transmit data
 * @return a pointer to the serial port on success, NULL otherwise
 */
SerialPort *spInit(const char *filename, int baudRate) {
    // open with O_NONBLOCK to avoid hanging when CLOCAL
    // is not yet set on the serial port (changed later)
    int oflags = O_RDWR | O_NOCTTY | O_NONBLOCK;
    int fd = open(filename, oflags);
    
    if (fd < 0) {
        printf(RED "Error! Failed to open the serial port '" BOLD "%s" R_BOLD "'.\n" RESET, filename);
        return NULL;
    }

    // save the current port settings
    struct termios oldSettings;

    if (tcgetattr(fd, &oldSettings) < 0) {
        printf(RED "Error! Failed to save the current serial port settings.\n" RESET);
        return NULL;
    }

    // convert the baud rate to an appropriate flag
    tcflag_t br;

    switch (baudRate) {
        case 1200:
            br = B1200;
            break;

        case 1800:
            br = B1800;
            break;

        case 2400:
            br = B2400;
            break;

        case 4800:
            br = B4800;
            break;

        case 9600:
            br = B9600;
            break;

        case 19200:
            br = B19200;
            break;

        case 38400:
            br = B38400;
            break;

        case 57600:
            br = B57600;
            break;

        case 115200:
            br = B115200;
            break;

        default:
            printf(RED "Error! Unsupported baud rate (must be 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, or 115200).\n" RESET);
            return NULL;
    }

    // configure the serial port
    struct termios newSettings;
    memset(&newSettings, 0, sizeof(newSettings));

    newSettings.c_cflag = br | CS8 | CLOCAL | CREAD;
    newSettings.c_iflag = IGNPAR;
    newSettings.c_oflag = 0;

    // set input mode (non-canonical, no echo, ...)
    newSettings.c_lflag = 0;
    newSettings.c_cc[VTIME] = 1;
    newSettings.c_cc[VMIN] = 0; // non-blocking read

    tcflush(fd, TCIOFLUSH);

    // set the new port settings as effective
    if (tcsetattr(fd, TCSANOW, &newSettings) < 0) {
        printf(RED "Error! Failed to configure the serial port.\n" RESET);
        close(fd);

        return NULL;
    }

    // initialize the serial port
    SerialPort *sp = malloc(sizeof(SerialPort));

    sp->fd = fd;
    sp->baudRate = baudRate;
    sp->oldSettings = oldSettings;

    return sp;
}

/**
 * @brief Deallocates the memory occupied by the serial port.
 * @param sp the serial port
 * @return 1 on success, a negative value otherwise
 */
int spFree(SerialPort *sp) {
    int statusCode = STATUS_SUCCESS;

    // restore the original port settings
    if (tcsetattr(sp->fd, TCSANOW, &sp->oldSettings) == -1) {
        printf(RED "Error! Failed to restore the serial port settings.\n" RESET);
        statusCode = STATUS_ERROR;
    }

    // close the serial port
    if (close(sp->fd) < 0) {
        statusCode = STATUS_ERROR;
    }

    free(sp);
    return statusCode;
}

/**
 * @brief Writes a stream of bytes to the serial port.
 * @param sp the serial port
 * @param bytes the stream of bytes to be written
 * @param numBytes the number of bytes to be written
 * @return the number of bytes written on success, a negative value otherwise
 */
int spWrite(SerialPort *sp, const unsigned char *bytes, int numBytes) {
    return (int) write(sp->fd, bytes, numBytes * sizeof(unsigned char));
}

/**
 * @brief Reads a single byte from the serial port.
 * @param sp the serial port
 * @param byte pointer which will store the byte read
 * @return 1 on success, 0 if no byte was read, a negative value otherwise
 */
int spRead(SerialPort *sp, unsigned char *byte) {
    return (int) read(sp->fd, byte, 1);
}
