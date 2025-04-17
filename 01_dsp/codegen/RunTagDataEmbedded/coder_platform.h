/* Copyright 2022 The MathWorks, Inc. */
/* Header file declaring functions used to determine platform. */
#ifndef CODER_PLATFORM_H
#define CODER_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief Returns 1 if running on PC, 0 otherwise.
 *
 */
int coderIsPC(void);

/**
 * @brief Returns 1 if running on MAC, 0 otherwise.
 *
 */
int coderIsMAC(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CODER_PLATFORM_H */
