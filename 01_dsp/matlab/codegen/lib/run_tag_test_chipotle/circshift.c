/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: circshift.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
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
  bool *shiftright_data;
  p_data = p->data;
  if (p->size[0] == 1) {
    int ns;
    bool shiftright;
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
        int u0;
        for (k = 0; k < ns; k++) {
          buffer[k] = a[(k - ns) + 512];
        }
        u0 = ns + 1;
        for (k = 512; k >= u0; k--) {
          a[k - 1] = a[(k - ns) - 1];
        }
        memcpy(&a[0], &buffer[0], (unsigned int)ns * sizeof(double));
      } else {
        int u0;
        memcpy(&buffer[0], &a[0], (unsigned int)ns * sizeof(double));
        u0 = 512 - ns;
        for (k = 0; k < u0; k++) {
          a[k] = a[k + ns];
        }
        for (k = 0; k < ns; k++) {
          a[(k - ns) + 512] = buffer[k];
        }
      }
    }
  } else {
    int ns;
    int stride;
    int u0;
    emxInit_int32_T(&absp, 1);
    stride = p->size[0];
    u0 = absp->size[0];
    absp->size[0] = p->size[0];
    emxEnsureCapacity_int32_T(absp, u0);
    absp_data = absp->data;
    emxInit_boolean_T(&b_shiftright);
    u0 = b_shiftright->size[0];
    b_shiftright->size[0] = p->size[0];
    emxEnsureCapacity_boolean_T(b_shiftright, u0);
    shiftright_data = b_shiftright->data;
    for (k = 0; k < stride; k++) {
      u0 = (int)p_data[k];
      shiftright_data[k] = true;
      if (k + 1 <= 2) {
        ns = 511 * k + 1;
      } else {
        ns = 1;
      }
      if (ns <= 1) {
        u0 = 0;
      } else {
        if (u0 > ns) {
          u0 -= ns * (int)((unsigned int)u0 / (unsigned short)ns);
        }
        if (u0 > (ns >> 1)) {
          u0 = ns - u0;
          shiftright_data[k] = false;
        }
      }
      absp_data[k] = u0;
    }
    stride = 1;
    ns = (int)fmin(2.0, p->size[0]);
    for (dim = 0; dim < ns; dim++) {
      int b_ns;
      int i;
      int i1;
      int npages;
      int nv;
      int pagesize;
      i = 511 * dim;
      nv = 511 * dim + 1;
      i1 = absp_data[dim];
      b_ns = absp_data[dim] - 1;
      pagesize = stride * nv;
      npages = -511 * dim + 511;
      if ((511 * dim > 0) && (absp_data[dim] > 0)) {
        for (b_i = 0; b_i <= npages; b_i++) {
          int pageroot;
          pageroot = b_i * pagesize;
          for (j = 0; j < stride; j++) {
            int b_i1;
            b_i1 = pageroot + j;
            if (shiftright_data[dim]) {
              for (k = 0; k <= b_ns; k++) {
                buffer[k] = a[b_i1 + (((k + i) - i1) + 1) * stride];
              }
              u0 = i1 + 1;
              for (k = nv; k >= u0; k--) {
                a[b_i1 + (k - 1) * stride] = a[b_i1 + ((k - i1) - 1) * stride];
              }
              for (k = 0; k <= b_ns; k++) {
                a[b_i1 + k * stride] = buffer[k];
              }
            } else {
              for (k = 0; k <= b_ns; k++) {
                buffer[k] = a[b_i1 + k * stride];
              }
              u0 = (i - i1) + 1;
              for (k = 0; k < u0; k++) {
                a[b_i1 + k * stride] = a[b_i1 + (k + i1) * stride];
              }
              for (k = 0; k <= b_ns; k++) {
                a[b_i1 + (((k + i) - i1) + 1) * stride] = buffer[k];
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
