/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: fileManager.h
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
 */

#ifndef FILEMANAGER_H
#define FILEMANAGER_H

/* Include Files */
#include "rtwtypes.h"
#include "run_tag_test_chipotle_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
int cfclose(double fid);

signed char cfopen(const emxArray_char_T *cfilename);

int fileManager(void);

void filedata_init(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for fileManager.h
 *
 * [EOF]
 */
