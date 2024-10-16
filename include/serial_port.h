#ifndef _SERIAL_PORT_H_
#define _SERIAL_PORT_H_

#include <termios.h>

/**
 * @brief A struct that represents the serial port.
 */
typedef struct {
    int fd;                     /** the file descriptor of the serial port */
    struct termios oldSettings; /** the original serial port settings (to be restored on closing) */
} SerialPort;

/* API */
SerialPort *initSerialPort(const char *filename, int baudRate);
int freeSerialPort(SerialPort *port);

int writeBytes(SerialPort *port, const unsigned char *data, int numBytes);
int readByte(SerialPort *port, unsigned char *byte);

#endif // _SERIAL_PORT_H_
