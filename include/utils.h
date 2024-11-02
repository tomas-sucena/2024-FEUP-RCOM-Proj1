#ifndef _UTILS_H_
#define _UTILS_H_

// boolean values
#define FALSE 0
#define TRUE  1

// status codes
#define STATUS_SUCCESS  1
#define STATUS_ERROR   -1
#define STATUS_TIMEOUT -2  

/* CONSOLE */
// colors
#define RED     "\e[0;31m"
#define GREEN   "\e[0;32m"
#define YELLOW  "\e[0;33m"

// character formatting
#define RESET   "\e[0m"
#define BOLD    "\e[1m"
#define R_BOLD  "\e[22m"

// line formatting
#define ERASE_LINE "\e[2K"
#define LINE_UP    "\e[1F" 

#endif // _UTILS_H_
