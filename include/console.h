#ifndef _CONSOLE_H_
#define _CONSOLE_H_

// colors
#define RED     "\e[0;31m"
#define GREEN   "\e[0;32m"
#define YELLOW  "\e[0;33m"

// formatting
#define RESET   "\e[0m"
#define BOLD    "\e[1m"
#define FAINT   "\e[2m"

extern volatile _Bool loading;

void loadingScreen(const char *message);

#endif // _CONSOLE_H_
