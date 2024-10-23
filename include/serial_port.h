#ifndef _SERIAL_PORT_H_
#define _SERIAL_PORT_H_

#include <termios.h>

/**
 * @brief A struct that represents the serial port.
 */
typedef struct {
    int fd;                     /** the file descriptor of the serial port */
    int baudRate;               /** the baud rate */
    struct termios oldSettings; /** the original serial port settings (to be restored on closing) */
} SerialPort;

/* API */
SerialPort *spInit(const char *filename, int baudRate);
int spFree(SerialPort *sp);

int spWrite(SerialPort *sp, const unsigned char *data, int numBytes);
int spRead(SerialPort *sp, unsigned char *byte);

#endif // _SERIAL_PORT_H_
