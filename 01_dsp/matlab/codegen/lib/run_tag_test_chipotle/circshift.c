/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: circshift.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

/* Include Files */
#include "circshift.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double a[512]
 *                const emxArray_real_T *p
 * Return Type  : void
 */
void circshift(double a[512], const emxArray_real_T *p)
{
  emxArray_boolean_T *b_shiftright;
  emxArray_int32_T *absp;
  double buffer[256];
  const double *p_data;
  int b_i;
  int dim;
  int j;
  int k;
  int *absp_data;
  boolean_T *shiftright_data;
  p_data = p->data;
  if (p->size[0] == 1) {
    int ns;
    boolean_T shiftright;
    ns = (int)p_data[0];
    shiftright = true;
    if ((int)p_data[0] > 512) {
      ns = (int)p_data[0] - ((int)((unsigned int)(int)p_data[0] >> 9) << 9);
    }
    if (ns > 256) {
      ns = 512 - ns;
      shiftright = false;
    }
    memset(&buffer[0], 0, 256U * sizeof(double));
    if (ns > 0) {
      if (shiftright) {
        int i;
        for (k = 0; k < ns; k++) {
          buffer[k] = a[(k - ns) + 512];
        }
        i = ns + 1;
        for (k = 512; k >= i; k--) {
          a[k - 1] = a[(k - ns) - 1];
        }
        memcpy(&a[0], &buffer[0], (unsigned int)ns * sizeof(double));
      } else {
        int i;
        memcpy(&buffer[0], &a[0], (unsigned int)ns * sizeof(double));
        i = 512 - ns;
        for (k = 0; k < i; k++) {
          a[k] = a[k + ns];
        }
        for (k = 0; k < ns; k++) {
          a[(k - ns) + 512] = buffer[k];
        }
      }
    }
  } else {
    int i;
    int i1;
    int i2;
    int stride;
    emxInit_int32_T(&absp, 1);
    i = p->size[0];
    i1 = absp->size[0];
    absp->size[0] = p->size[0];
    emxEnsureCapacity_int32_T(absp, i1);
    absp_data = absp->data;
    emxInit_boolean_T(&b_shiftright);
    i1 = b_shiftright->size[0];
    b_shiftright->size[0] = p->size[0];
    emxEnsureCapacity_boolean_T(b_shiftright, i1);
    shiftright_data = b_shiftright->data;
    for (k = 0; k < i; k++) {
      stride = (int)p_data[k];
      shiftright_data[k] = true;
      if (k + 1 <= 2) {
        i1 = 511 * k + 1;
      } else {
        i1 = 1;
      }
      if (i1 <= 1) {
        stride = 0;
      } else {
        if (stride > i1) {
          if ((unsigned short)i1 == 0) {
            i2 = MAX_int32_T;
          } else {
            i2 = (int)((unsigned int)stride / (unsigned short)i1);
          }
          stride -= i1 * i2;
        }
        if (stride > (i1 >> 1)) {
          stride = i1 - stride;
          shiftright_data[k] = false;
        }
      }
      absp_data[k] = stride;
    }
    stride = 1;
    i = (int)fmin(2.0, p->size[0]);
    for (dim = 0; dim < i; dim++) {
      int npages;
      int ns;
      int nv_tmp;
      int pagesize;
      i1 = 511 * dim;
      nv_tmp = 511 * dim + 1;
      i2 = absp_data[dim];
      ns = absp_data[dim] - 1;
      pagesize = stride * nv_tmp;
      npages = -511 * dim + 511;
      if ((nv_tmp > 1) && (absp_data[dim] > 0)) {
        for (b_i = 0; b_i <= npages; b_i++) {
          int pageroot;
          pageroot = b_i * pagesize;
          for (j = 0; j < stride; j++) {
            int b_i1;
            b_i1 = pageroot + j;
            if (shiftright_data[dim]) {
              int i3;
              for (k = 0; k <= ns; k++) {
                buffer[k] = a[b_i1 + (((k + i1) - i2) + 1) * stride];
              }
              i3 = i2 + 1;
              for (k = nv_tmp; k >= i3; k--) {
                a[b_i1 + (k - 1) * stride] = a[b_i1 + ((k - i2) - 1) * stride];
              }
              for (k = 0; k <= ns; k++) {
                a[b_i1 + k * stride] = buffer[k];
              }
            } else {
              int i3;
              for (k = 0; k <= ns; k++) {
                buffer[k] = a[b_i1 + k * stride];
              }
              i3 = (i1 - i2) + 1;
              for (k = 0; k < i3; k++) {
                a[b_i1 + k * stride] = a[b_i1 + (k + i2) * stride];
              }
              for (k = 0; k <= ns; k++) {
                a[b_i1 + (((k + i1) - i2) + 1) * stride] = buffer[k];
              }
            }
          }
        }
      }
      stride = pagesize;
    }
    emxFree_boolean_T(&b_shiftright);
    emxFree_int32_T(&absp);
  }
}

/*
 * File trailer for circshift.c
 *
 * [EOF]
 */
