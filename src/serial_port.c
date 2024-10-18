// Serial port interface implementation

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "../include/serial_port.h"
#include "../include/utils.h"

#define _POSIX_SOURCE 1 // POSIX compliant source

SerialPort *spInit(const char *filename, int baudRate) {
    // open with O_NONBLOCK to avoid hanging when CLOCAL
    // is not yet set on the serial port (changed later)
    int oflags = O_RDWR | O_NOCTTY | O_NONBLOCK;
    int fd = open(filename, oflags);
    
    if (fd < 0) {
        printf(RED "Error! Failed to open the serial port '" BOLD "%s" RESET RED "'.\n" RESET, filename);
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
            printf(RED "Error! Unsupported baud rate (must be one of 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200).\n" RESET);
            return NULL;
    }

    // configure the serial port
    struct termios newSettings;
    memset(&newSettings, 0, sizeof(newSettings));

    newSettings.c_cflag = br | CS8 | CLOCAL | CREAD;
    newSettings.c_iflag = IGNPAR;
    newSettings.c_oflag = 0;

    // set input mode (non-canonical, no echo,...)
    newSettings.c_lflag = 0;
    newSettings.c_cc[VTIME] = 0; // Block reading
    newSettings.c_cc[VMIN] = 1;  // Byte by byte

    tcflush(fd, TCIOFLUSH);

    // set the new port settings as effective
    if (tcsetattr(fd, TCSANOW, &newSettings) < 0) {
        perror("tcsetattr");
        close(fd);

        return NULL;
    }

    // Clear O_NONBLOCK flag to ensure blocking reads
    //oflags ^= O_NONBLOCK;
    if (fcntl(fd, F_SETFL, oflags) == -1) {
        perror("fcntl");
        close(fd);

        return NULL;
    }

    // initialize the serial port
    SerialPort *port = malloc(sizeof(SerialPort));

    port->fd = fd;
    port->oldSettings = oldSettings;

    return port;
}

int spFree(SerialPort *port) {
    int statusCode = STATUS_SUCCESS;

    // restore the original port settings
    if (tcsetattr(port->fd, TCSANOW, &port->oldSettings) == -1) {
        perror("tcsetattr");
        statusCode = STATUS_ERROR;
    }

    // close the serial port
    if (close(port->fd) < 0) {
        statusCode = STATUS_ERROR;
    }

    free(port);
    return statusCode;
}

int spWrite(SerialPort *port, const unsigned char *data, int numBytes) {
    // ensure all the bytes were written to the serial port
    return write(port->fd, data, numBytes * sizeof(unsigned char));
}

int spRead(SerialPort *port, unsigned char *byte) {
    // ensure a single byte was read from the serial port
    return read(port->fd, byte, 1);
}
