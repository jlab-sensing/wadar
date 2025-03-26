/* Copyright 2022 The MathWorks, Inc. */
#include "coder_platform.h"

int coderIsPC(void) {
#ifdef _WIN32
    return 1;
#else
    return 0;
#endif
}

int coderIsMAC(void) {
#ifdef __APPLE__
    return 1;
#else
    return 0;
#endif
}
