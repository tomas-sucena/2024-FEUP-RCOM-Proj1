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
    const char *filepath = (argc < 5) ? NULL : argv[4];

    // validate the role
    if (strcmp("tx", role) != 0 && strcmp("rx", role) != 0) {
        printf(RED "Error! Role must be \"tx\" or \"rx\".\n" RESET);
        return EXIT_FAILURE;
    }

    // initialize the application layer
    ApplicationLayer *app = appInit(serialPort, (role[0] == 't'), baudrate, DEFAULT_N_TRIES, DEFAULT_TIMEOUT,
        filepath, DEFAULT_DATA_SIZE);

    if (app == NULL) {
        return EXIT_FAILURE;
    }
    
    // run the application
    printf("\n> Starting link-layer protocol application\n"
           BOLD "  - Serial port:" RESET " %s\n"
           BOLD "  - Role:" RESET " %s\n"
           BOLD "  - Baudrate:" RESET " %d Hz\n"
           BOLD "  - Number of tries:" RESET " %d\n"
           BOLD "  - Timeout:" RESET " %d s\n"
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
