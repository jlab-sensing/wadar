/* Copyright 2022 The MathWorks, Inc. */
/* Header file declaring functions used to get current working directory. */
#ifndef CODER_FILEOPS_H
#define CODER_FILEOPS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* returns length of file name if malloc is allowed */
int coderGetLenghtOfCwd(void);

/* returns pathname of the current directory */
int coderGetCurrentDirectory(char* buffer, int fileNameMax);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CODER_FILEOPS_H */