#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdio.h>

#define air_printf(fmt, ...) \
  do { \
    if (CHATTY) \
      printf(fmt, ##__VA_ARGS__); \
  } while (0)

#endif // DEBUG_H_
