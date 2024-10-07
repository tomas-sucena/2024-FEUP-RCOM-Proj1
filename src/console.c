#include <stdio.h>
#include <unistd.h>

#include "../include/console.h"

void loadingScreen(const char *message) {
    printf("%s", message);
    fflush(stdout); // flush the standard output so the message is output

    int dots = 0; // the number of trailing dots after the message

    while (loading) {
        usleep(500000); // sleep for 500 milisseconds

        if (dots < 3) {
            putchar('.');
            ++dots;
        }
        else {
            printf("\e[3D\e[0K"); // erase the dots
            dots = 0; // reset the dots
        }

        fflush(stdout); // flush the standard output so the dots are output
    }

    putchar('\n');
}
