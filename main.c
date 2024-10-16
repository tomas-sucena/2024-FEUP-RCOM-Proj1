// Main file of the serial port project.
// NOTE: This file must not be changed.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "include/application_layer.h"
#include "include/utils.h"

#define DEFAULT_N_TRIES 3
#define DEFAULT_TIMEOUT 4
#define DEFAULT_DATA_SIZE 200

// Arguments:
//   $1: /dev/ttySxx
//   $2: baud rate
//   $3: tx | rx
//   $4: filename
int main(int argc, char *argv[])
{
    if (argc < 4) {
        printf("Usage: %s /dev/ttySxx baudrate tx|rx filename\n", argv[0]);
        return EXIT_FAILURE;
    }

    const char *serialPort = argv[1];
    const int baudrate = atoi(argv[2]);
    const char *role = argv[3];
    const char *filepath = (argc < 5) ? argv[4] : NULL;

    // validate the baud rate
    switch (baudrate) {
        case 1200:
        case 1800:
        case 2400:
        case 4800:
        case 9600:
        case 19200:
        case 38400:
        case 57600:
        case 115200:
            break;

        default:
            printf(RED "Unsupported baud rate (must be one of 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200)\n." RESET);
            return EXIT_FAILURE;
    }

    // validate the role
    if (strcmp("tx", role) != 0 && strcmp("rx", role) != 0) {
        printf(RED "Error! Role must be \"tx\" or \"rx\".\n" RESET);
        return EXIT_FAILURE;
    }

    _Bool isSender = (role[0] == 't');

    // initialize the application layer
    ApplicationLayer *app = applicationLayer(isSender, filepath, DEFAULT_DATA_SIZE);

    if (app == NULL) {
        return EXIT_FAILURE;
    }
    
    // run the application
    printf("Starting link-layer protocol application\n"
           BOLD "  - Serial port:" RESET " %s\n"
           BOLD "  - Role:" RESET " %s\n"
           BOLD "  - Baudrate:" RESET " %d\n"
           BOLD "  - Number of tries:" RESET " %d\n"
           BOLD "  - Timeout:" RESET " %d\n"
           BOLD "  - Filename:" RESET " %s\n",
           serialPort,
           role,
           baudrate,
           DEFAULT_N_TRIES,
           DEFAULT_TIMEOUT,
           filepath);

    return EXIT_SUCCESS;
    //return applicationLayer(serialPort, (role[0] == 't'), baudrate, DEFAULT_N_TRIES, DEFAULT_TIMEOUT, filename);
}
