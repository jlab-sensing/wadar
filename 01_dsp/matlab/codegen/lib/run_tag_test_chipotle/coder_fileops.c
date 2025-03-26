/* Copyright 2022 The MathWorks, Inc. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "coder_fileops.h"

#ifdef _WIN32
#include <direct.h>
#define getcwd _getcwd
#else
#include <unistd.h>
#endif

/* returns length of file name */
int coderGetLenghtOfCwd(void) {
    char * currentDir = getcwd(NULL, 0);
    if(currentDir == NULL) {
        return -2;
    }
    int fm = (int)strlen(currentDir);
    free(currentDir);
    return fm;
}

/* returns pathname and its length. */
/* -1 if pathname doesnt fit in buffer. */
int coderGetCurrentDirectory(char* buffer, int fileNameMax) {
    if(getcwd(buffer, fileNameMax) != NULL) {
        return (int)strlen(buffer);
    } else {
        return -1;   
    }
}
