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

int main(int argc, char *argv[]) {
    // Arguments:
    //   $1: /dev/ttySxx
    //   $2: baud rate
    //   $3: tx | rx
    //   $4: filename
    if (argc < 4) {
        const char *program = argv[0];
        printf("> Application usage\n"
               BOLD "  - Sender:" RESET "   %s /dev/ttySxx <baud rate> tx <filepath> [data size]\n"
               BOLD "  - Receiver:" RESET " %s /dev/ttySxx <baud rate> rx [filename]\n",
               program, program);

        return EXIT_FAILURE;
    }

    const char *serialPort = argv[1];
    const int baudrate = atoi(argv[2]);
    const char *role = argv[3];
    const char *filepath = (argc < 5)
        ? NULL
        : argv[4];
    const int dataSize = (argc < 6)
        ? DEFAULT_DATA_SIZE
        : atoi(argv[5]);
    
    // initialize the application layer
    ApplicationLayer *app = appInit(serialPort, role, baudrate, DEFAULT_N_TRIES, DEFAULT_TIMEOUT,
        filepath, dataSize);

    if (app == NULL) {
        return EXIT_FAILURE;
    }
    
    // run the application
    printf("\n> Starting link-layer protocol application\n"
           BOLD "  - Serial port:" RESET " %s\n"
           BOLD "  - Role:" RESET " %s\n"
           BOLD "  - Baudrate:" RESET " %d bit/s\n"
           BOLD "  - Number of tries:" RESET " %d\n"
           BOLD "  - Timeout:" RESET " %ds\n"
           BOLD "  - File:" RESET " %s\n",
           serialPort,
           role,
           baudrate,
           DEFAULT_N_TRIES,
           DEFAULT_TIMEOUT,
           filepath);

    if (appRun(app) < 0) {
        printf("\n> Aborting...\n");
        appFree(app);

        return EXIT_FAILURE;
    }

    // free the application
    if (appFree(app) < 0) {
        return EXIT_FAILURE;
    }

    printf("\n> Exiting...\n");
    return EXIT_SUCCESS;
}
