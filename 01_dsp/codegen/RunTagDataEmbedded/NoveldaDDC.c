/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: NoveldaDDC.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

/* Include Files */
#include "NoveldaDDC.h"
#include "RunTagDataEmbedded_data.h"
#include "RunTagDataEmbedded_emxutil.h"
#include "RunTagDataEmbedded_types.h"
#include "conv.h"
#include "cos.h"
#include "ifWhileCond.h"
#include "linspace.h"
#include "mean.h"
#include "rt_nonfinite.h"
#include "sin.h"
#include "strcmp.h"
#include "string1.h"
#include <stdio.h>
#include <string.h>

/* Function Declarations */
static void binary_expand_op_1(emxArray_creal_T *in1,
                               const emxArray_real_T *in2, double in3,
                               const emxArray_real_T *in4,
                               const emxArray_real_T *in6);

/* Function Definitions */
/*
 * Arguments    : emxArray_creal_T *in1
 *                const emxArray_real_T *in2
 *                double in3
 *                const emxArray_real_T *in4
 *                const emxArray_real_T *in6
 * Return Type  : void
 */
static void binary_expand_op_1(emxArray_creal_T *in1,
                               const emxArray_real_T *in2, double in3,
                               const emxArray_real_T *in4,
                               const emxArray_real_T *in6)
{
  emxArray_creal_T *b_in2;
  creal_T *b_in2_data;
  const double *in2_data;
  const double *in4_data;
  const double *in6_data;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  in6_data = in6->data;
  in4_data = in4->data;
  in2_data = in2->data;
  emxInit_creal_T(&b_in2, 1);
  if (in4->size[1] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in4->size[1];
  }
  i = b_in2->size[0];
  b_in2->size[0] = loop_ub;
  emxEnsureCapacity_creal_T(b_in2, i);
  b_in2_data = b_in2->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in4->size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    double b_in4_re_tmp;
    double d;
    int in4_re_tmp;
    in4_re_tmp = i * stride_1_0;
    b_in4_re_tmp = in6_data[in4_re_tmp];
    d = in2_data[i * stride_0_0] - in3;
    b_in2_data[i].re = d * (in4_data[in4_re_tmp] + 0.0 * b_in4_re_tmp);
    b_in2_data[i].im = d * b_in4_re_tmp;
  }
  conv(b_in2, in1);
  emxFree_creal_T(&b_in2);
}

/*
 * basebandSignal = NoveldaDDC(rfSignal, chipSet, PGen, Fs, Fc)
 *
 *  Function to apply a digital downcovert (DDC) to a high frequency radar
 *  signal. Brings signal to baseband frequencies and provides an analytic
 *  signal (i.e. I & Q, in-phase & quadrature, outputs)
 *
 *  Inputs:
 *    rfSignal: high-frequency sampled UWB radar signal
 *     chipSet: 'X1-IPG0' or 'X1-IPG1' for NVA6100 or 'X2' for NVA620x
 *        PGen: 0 = slow, 1 = nominal, 3 = fast, for 'X1'
 *              0-10 (CF = 5.3GHz to 8.8 GHz), for 'X2'
 *          Fs: sampling rate in Hz (extracted directly from chip)
 *          Fc: center frequency for custom down-conversion (optional input)
 *
 *  Outputs:
 *    basebandSignal: downconverted, i.e. baseband, and filtered IQ radar
 *                    signal, note: use abs() on output to get envelope
 *                    magnitude
 *
 *  Created 5/11/2015
 *  Copyright FlatEarth, Inc.
 *
 *  5/12/2015 Update to include optional Center Frequency (CF) input.
 *
 *  3/1/2016 Update to base frameSize on length of rfsignal instead of
 *  NoveldaChipParams function
 *
 *  12/4/2017 Update hard-coded Fs for X4 chip
 *
 *
 * Arguments    : const emxArray_real_T *rfSignal
 *                const char chipSet_Value_data[]
 *                const int chipSet_Value_size[2]
 *                const double PGen_data[]
 *                int PGen_size
 *                double Fs
 *                emxArray_creal_T *basebandSignal
 * Return Type  : void
 */
void NoveldaDDC(const emxArray_real_T *rfSignal,
                const char chipSet_Value_data[],
                const int chipSet_Value_size[2], const double PGen_data[],
                int PGen_size, double Fs, emxArray_creal_T *basebandSignal)
{
  emxArray_creal_T *b_rfSignal;
  emxArray_real_T *r;
  emxArray_real_T *r2;
  creal_T *b_rfSignal_data;
  const double *rfSignal_data;
  double fH;
  double *r1;
  double *r3;
  int switch_expression_Value_size[2];
  int b_index;
  int fL;
  int i;
  int i1;
  char switch_expression_Value_data[7];
  char Sampler_idx_0;
  rfSignal_data = rfSignal->data;
  /*  Don't need to specify Fs if using X4 chip... */
  /*  Calculate Sampler for X1/X2 */
  if (Fs < 3.0E+10) {
    Sampler_idx_0 = '8';
  } else {
    Sampler_idx_0 = '4';
  }
  /*  If custom, user-specified CF (center frequency) is not provided... */
  /*  Call NoveldaChipParams function to get chip parameters */
  /*  Look up table to extract all Novelda pulse parameters depending on the */
  /*  function [fc, bw, bwr, vp, n, bw_hz, pwr_dBm, fs_hz] =
   * NoveldaChipParams(chipSet, PGen, Sampler) */
  /*  */
  /*  Inputs: */
  /*    chipSet:    'X1-IPG0' = NVA6100, medium-band pgen */
  /*                'X1-IPG1' = NVA6100, low-band pgen */
  /*                'X2' = NVA6201/2 */
  /*                'X4' = X4 SoC */
  /*  */
  /*    PGen:       0 = slow, 1 = nominal, 3 = fast, for 'X1' */
  /*                0-11 (CF = 5.3 GHz to 9.1 GHz), for 'X2' */
  /*                0 or 3: CF = 7.29 GHz (ETSI/FCC Compliant), 1 or 4: CF
   * = 8.748 GHz (KCC/FCC Compliant) for 'X4' */
  /*  */
  /*    Sampler:    '4mm' = 39 GS/s, '8mm' = 20 GS/s, or '4cm' = 3.8 GS/s, for
   * 'X1' */
  /*                '4mm' only for 'X2' or just omit value */
  /*                '6mm' only for 'X4' or just omit value */
  /*  */
  /*  Outputs:   */
  /*    fc      = center frequency */
  /*    bw      = fractional bandwidth */
  /*    bwr     = dB down for bandwidth extents */
  /*    vp      = instantaneous output voltage peak amplitude */
  /*    n       = frameSize, i.e. number of samplers */
  /*    bw_hz   = bandwidth in hertz */
  /*    pwr_dBm = mean/average output power in dBm (i.e. true RMS) */
  /*    fs_hz   = sampling rate in Hz */
  /*  */
  /*  Update 9/26/2017, Added X4 support */
  /*  */
  string_lower(chipSet_Value_data, chipSet_Value_size,
               switch_expression_Value_data, switch_expression_Value_size);
  if (string_eq(switch_expression_Value_data, switch_expression_Value_size)) {
    b_index = 0;
  } else if (b_string_eq(switch_expression_Value_data,
                         switch_expression_Value_size)) {
    b_index = 1;
  } else if (c_string_eq(switch_expression_Value_data,
                         switch_expression_Value_size)) {
    b_index = 2;
  } else if (d_string_eq(switch_expression_Value_data,
                         switch_expression_Value_size)) {
    b_index = 3;
  } else {
    b_index = -1;
  }
  switch (b_index) {
  case 0: {
    char switch_expression[3];
    bool b_PGen_data;
    /* -------------------NVA6100, Medium-Band PGen--------------------%% */
    /*  bwr dB down from normalized peak setting for fractional bandwidth
     * (changed to 6 dB power spectra bw, looks better) */
    /*  # of samplers, i.e. length of frame */
    /*  Pulse Generator */
    for (i = 0; i < PGen_size; i++) {
      b_PGen_data = (PGen_data[0] == 0.0);
    }
    if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
      b_index = 0;
    } else {
      for (i = 0; i < PGen_size; i++) {
        b_PGen_data = (PGen_data[0] == 1.0);
      }
      if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
        b_index = 1;
      } else {
        for (i = 0; i < PGen_size; i++) {
          b_PGen_data = (PGen_data[0] == 2.0);
        }
        if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
          b_index = 2;
        } else {
          b_index = -1;
        }
      }
    }
    switch (b_index) {
    case 0:
      fL = 660000000;
      /*  -10 dB low cutoff */
      fH = 7.145E+9;
      /*  -10 dB high cutoff */
      /*  470 mV instantaneous output amplitude (peak voltage)     */
      break;
    case 1:
      fL = 845000000;
      /*  -10 dB low cutoff */
      fH = 9.55E+9;
      /*  -10 dB high cutoff */
      /*  450 mV instantaneous output amplitude (peak voltage)   */
      break;
    case 2:
      fL = 1060000000;
      /*  -10 dB low cutoff */
      fH = 1.041E+10;
      /*  -10 dB high cutoff */
      /*  370 mV instantaneous output amplitude (peak voltage)   */
      break;
    }
    /*  Sampling Rate */
    switch_expression[0] = cv[(int)Sampler_idx_0];
    switch_expression[1] = cv[109];
    switch_expression[2] = cv[109];
    if (b_strcmp(switch_expression)) {
      b_index = 0;
    } else if (c_strcmp(switch_expression)) {
      b_index = 1;
    } else if (d_strcmp(switch_expression)) {
      b_index = 2;
    } else {
      b_index = -1;
    }
    switch (b_index) {
    case 0:
      /*  mean system sampling rate for 4mm */
      break;
    case 1:
      /*  mean system sampling rate for 8mm */
      break;
    case 2:
      /*  mean system sampling rate for 4 cm */
      break;
    default:
      /*  Default, mean system sampling rate for 4mm */
      printf("For X1 platform, Sampler as string input must be, 4mm, 8mm, or "
             "4cm...\n");
      fflush(stdout);
      break;
    }
    /*  Center frequency, bandwidth in hertz, and fractional bandwidth */
    fH = (fH + (double)fL) / 2.0;
    /*  Mean/Average output power in dBm (i.e. true RMS), Nominal, */
    /*  function of PRF @ 50 MHz */
  } break;
  case 1: {
    char switch_expression[3];
    bool b_PGen_data;
    /* ---------------------NVA6100, Low-Band PGen---------------------%% */
    /*  bwr dB down from normalized peak setting for fractional bandwidth
     * (change to 6 dB power spectra bw, looks better) */
    /*  # of samplers, i.e. length of frame */
    /*  500 mV instantaneous output amplitude (peak voltage)   */
    /*  Pulse Generator */
    for (i = 0; i < PGen_size; i++) {
      b_PGen_data = (PGen_data[0] == 0.0);
    }
    if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
      b_index = 0;
    } else {
      for (i = 0; i < PGen_size; i++) {
        b_PGen_data = (PGen_data[0] == 1.0);
      }
      if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
        b_index = 1;
      } else {
        for (i = 0; i < PGen_size; i++) {
          b_PGen_data = (PGen_data[0] == 2.0);
        }
        if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
          b_index = 2;
        } else {
          b_index = -1;
        }
      }
    }
    switch (b_index) {
    case 0:
      fL = 435000000;
      /*  -10 dB low cutoff */
      fH = 3.165E+9;
      /*  -10 dB high cutoff */
      break;
    case 1:
      fL = 450000000;
      /*  -10 dB low cutoff */
      fH = 3.555E+9;
      /*  -10 dB high cutoff */
      break;
    case 2:
      fL = 485000000;
      /*  -10 dB low cutoff */
      fH = 4.065E+9;
      /*  -10 dB high cutoff */
      break;
    }
    /*  Sampling Rate */
    switch_expression[0] = cv[(int)Sampler_idx_0];
    switch_expression[1] = cv[109];
    switch_expression[2] = cv[109];
    if (b_strcmp(switch_expression)) {
      b_index = 0;
    } else if (c_strcmp(switch_expression)) {
      b_index = 1;
    } else if (d_strcmp(switch_expression)) {
      b_index = 2;
    } else {
      b_index = -1;
    }
    switch (b_index) {
    case 0:
      /*  mean system sampling rate for 4mm */
      break;
    case 1:
      /*  mean system sampling rate for 8mm */
      break;
    case 2:
      /*  mean system sampling rate for 4 cm */
      break;
    default:
      /*  Default, mean system sampling rate for 4mm */
      printf("For X1 platform, Sampler as string input must be, 4mm, 8mm, or "
             "4cm...\n");
      fflush(stdout);
      break;
    }
    /*  Center frequency, bandwidth in hertz, and fractional bandwidth */
    fH = (fH + (double)fL) / 2.0;
    /*  Mean/Average output power in dBm (i.e. true RMS), Nominal, */
    /*  function of PRF @ 50 MHz */
  } break;
  case 2: {
    bool b_PGen_data;
    /* -----------------------------NVA6201---------------------------%% */
    /*  bwr dB down from normalized peak setting for fractional bandwidth (5 dB
     * power spectra bw) */
    /*  # of samplers, i.e. length of frame */
    /*  mean system sampling rate, 39 GS/s */
    /*  Pulse Generator */
    for (i = 0; i < PGen_size; i++) {
      b_PGen_data = (PGen_data[0] == 0.0);
    }
    if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
      b_index = 0;
    } else {
      for (i = 0; i < PGen_size; i++) {
        b_PGen_data = (PGen_data[0] == 1.0);
      }
      if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
        b_index = 1;
      } else {
        for (i = 0; i < PGen_size; i++) {
          b_PGen_data = (PGen_data[0] == 2.0);
        }
        if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
          b_index = 2;
        } else {
          for (i = 0; i < PGen_size; i++) {
            b_PGen_data = (PGen_data[0] == 3.0);
          }
          if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
            b_index = 3;
          } else {
            for (i = 0; i < PGen_size; i++) {
              b_PGen_data = (PGen_data[0] == 4.0);
            }
            if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
              b_index = 4;
            } else {
              for (i = 0; i < PGen_size; i++) {
                b_PGen_data = (PGen_data[0] == 5.0);
              }
              if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
                b_index = 5;
              } else {
                for (i = 0; i < PGen_size; i++) {
                  b_PGen_data = (PGen_data[0] == 6.0);
                }
                if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
                  b_index = 6;
                } else {
                  for (i = 0; i < PGen_size; i++) {
                    b_PGen_data = (PGen_data[0] == 7.0);
                  }
                  if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
                    b_index = 7;
                  } else {
                    for (i = 0; i < PGen_size; i++) {
                      b_PGen_data = (PGen_data[0] == 8.0);
                    }
                    if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
                      b_index = 8;
                    } else {
                      for (i = 0; i < PGen_size; i++) {
                        b_PGen_data = (PGen_data[0] == 9.0);
                      }
                      if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
                        b_index = 9;
                      } else {
                        for (i = 0; i < PGen_size; i++) {
                          b_PGen_data = (PGen_data[0] == 10.0);
                        }
                        if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
                          b_index = 10;
                        } else {
                          for (i = 0; i < PGen_size; i++) {
                            b_PGen_data = (PGen_data[0] == 11.0);
                          }
                          if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
                            b_index = 11;
                          } else {
                            b_index = -1;
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
    switch (b_index) {
    case 0:
      /*  PGSelect = 0; */
      fH = 5.3E+9;
      /*  center frequency */
      /*  bandwidth in hertz */
      /*  fractional bandwidth (~1.75 GHz @ 5.3 GHz) */
      /*  690 mV instanteous output voltage peak-to-peak */
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 1:
      /*  PGSelect = 1; */
      fH = 5.4E+9;
      /*  center frequency */
      /*  bandwidth in hertz */
      /*  fractional bandwidth (~1.80 GHz @ 5.4 GHz) */
      /*  690 mV instanteous voltage peak-to-peak  */
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 2:
      /*  PGSelect = 2; */
      fH = 5.7E+9;
      /*  center frequency */
      /*  bandwidth in hertz */
      /*  fractional bandwidth (~1.85 GHz @ 5.7 GHz) */
      /*  720 mV instanteous voltage peak-to-peak  */
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 3:
      /*  PGSelect = 3; */
      fH = 6.1E+9;
      /*  center frequency */
      /*  bandwidth in hertz */
      /*  fractional bandwidth (~2.05 GHz @ 6.1 GHz) */
      /*  710 mV instanteous voltage peak-to-peak */
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 4:
      /*  PGSelect = 4; */
      fH = 6.4E+9;
      /*  center frequency */
      /*  bandwidth in hertz */
      /*  fractional bandwidth (~2.15 GHz @ 6.4 GHz) */
      /*  720 mV instanteous voltage peak-to-peak  */
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 5:
      /*  PGSelect = 5; */
      fH = 6.8E+9;
      /*  center frequency */
      /*  bandwidth in hertz */
      /*  fractional bandwidth (~2.30 GHz @ 6.8 GHz) */
      /*  690 mV instanteous voltage peak-to-peak */
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 6:
      /*  PGSelect = 6; */
      fH = 7.3E+9;
      /*  center frequency */
      /*  bandwidth in hertz */
      /*  fractional bandwidth (~2.35 GHz @ 7.3 GHz) */
      /*  650 mV instanteous voltage peak-to-peak  */
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 7:
      /*  PGSelect = 7; */
      fH = 7.7E+9;
      /*  center frequency */
      /*  bandwidth in hertz */
      /*  fractional bandwidth (~2.50 GHz @ 7.7 GHz) */
      /*  620 mV instanteous output voltage peak-to-peak  */
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 8:
      /*  PGSelect = 8; */
      fH = 7.8E+9;
      /*  center frequency */
      /*  bandwidth in hertz */
      /*  fractional bandwidth (~2.50 GHz @ 7.8 GHz) */
      /*  620 mV instanteous output voltage peak-to-peak  */
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 9:
      /*  PGSelect = 9; */
      fH = 8.2E+9;
      /*  center frequency */
      /*  bandwidth in hertz */
      /*  fractional bandwidth (~2.65 GHz @ 8.2 GHz) */
      /*  570 mV instanteous output voltage peak-to-peak */
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 10:
      /*  PGSelect = 10 */
      fH = 8.8E+9;
      /*  center frequency */
      /*  bandwidth in hertz */
      /*  fractional bandwidth (~3.10 GHz @ 8.8 GHz) */
      /*  540 mV instanteous output voltage peak-to-peak */
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 11:
      /*  PGSelect = 11 */
      fH = 9.1E+9;
      /*  center frequency */
      /*  bandwidth in hertz....not sure about this? */
      /*  fractional bandwidth (~3.10 GHz @ 8.8 GHz) */
      /*  540 mV instanteous output voltage peak-to-peak */
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    }
  } break;
  case 3: {
    int exitg2;
    bool b_PGen_data;
    /* ------------------X4, Impulse Radar Transceiver SoC-----------------%% */
    /*  bwr dB down from normalized peak setting for fractional bandwidth (5 dB
     * power spectra bw) */
    /*  # of samplers, i.e. length of frame */
    /*  mean system sampling rate, 23.328 GS/s */
    /*  Pulse Generator */
    b_index = -1;
    fL = 0;
    do {
      exitg2 = 0;
      if (fL < 2) {
        i = 3 * fL;
        for (i1 = 0; i1 < PGen_size; i1++) {
          b_PGen_data = (i == PGen_data[0]);
        }
        if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
          b_index = 0;
          exitg2 = 1;
        } else {
          fL++;
        }
      } else {
        fL = 0;
        exitg2 = 2;
      }
    } while (exitg2 == 0);
    if (exitg2 != 1) {
      bool exitg1;
      exitg1 = false;
      while ((!exitg1) && (fL < 2)) {
        i = 3 * fL + 1;
        for (i1 = 0; i1 < PGen_size; i1++) {
          b_PGen_data = (i == PGen_data[0]);
        }
        if (ifWhileCond((bool *)&b_PGen_data, PGen_size)) {
          b_index = 1;
          exitg1 = true;
        } else {
          fL++;
        }
      }
    }
    switch (b_index) {
    case 0:
      /*  ETSI/FCC         */
      fH = 7.29E+9;
      /*  center frequency */
      /*  bandwidth in hertz....not sure about this? */
      /*  fractional bandwidth (~3.10 GHz @ 8.8 GHz) */
      /*  310 mV instanteous output voltage peak */
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF
       * @ 60.75 MHz) */
      break;
    case 1:
      /*  KCC/FCC */
      fH = 8.748E+9;
      /*  center frequency */
      /*  bandwidth in hertz....not sure about this? */
      /*  fractional bandwidth (~3.10 GHz @ 8.8 GHz) */
      /*  310 mV instanteous output voltage peak */
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF
       * @ 60.75 MHz) */
      break;
    }
  } break;
  }
  /*  Find size of rf signal */
  /*  Frame Size according to input rf signal */
  fL = rfSignal->size[0];
  if (fL < 1) {
    fL = 1;
  }
  /*  Digital Down-Convert parameters (normalized frequency index) */
  /*  Generate the complex sinusoid LO (local oscillator) to mix into the signal
   */
  /* phi=-pi/3; */
  fH = 6.2831853071795862 * (fH / Fs * (double)fL);
  emxInit_real_T(&r, 2);
  linspace(fL, r);
  i = r->size[0] * r->size[1];
  r->size[0] = 1;
  emxEnsureCapacity_real_T(r, i);
  r1 = r->data;
  fL = r->size[1] - 1;
  for (i = 0; i <= fL; i++) {
    r1[i] *= fH;
  }
  /*  Adjust LO size, just in case */
  /*  Digital Downconvert (the DDC) via direct multiplication */
  /*  subtracting the mean removes DC offset */
  fH = mean(rfSignal);
  /*  % LPF Design to eliminate the upper mixing frequencies after the DDC (21
   */
  /*  % tap LPF, may need to be tuned based on the rate of desired roll-off) */
  /*  fn = bw_Hz / 2 / Fs;            % normalized cutoff frequency */
  /*  M = 20;                         % filter order, equal to # of filter taps
   * - 1 */
  /*  lpfWeights = zeros(1, M + 1);   % allocate memory */
  /*  for n = 0:M */
  /*      if n ~= M/2 */
  /*          % LPF based on sinc function (21 taps) */
  /*          lpfWeights(n + 1) = sin((2* pi * fn) * (n - M/2)) / (pi * (n -
   * M/2)); */
  /*      else */
  /*          % Needed for filter with odd number of taps */
  /*          lpfWeights(n + 1) = 2 * fn; */
  /*      end */
  /*  end */
  /*  window = filterDesign(M + 1, 'hamming');    % create window to smooth
   * filter */
  /*  filterWeights = window .* lpfWeights;       % direct multiply weights for
   * final weights */
  /*  LPF Design to eliminate the upper mixing frequencies after the DDC (21-
   * tap hamming window) */
  /*  filter order, equal to # of filter taps - 1 */
  /*  M = 10; */
  /*  window = filterDesign(M + 1, 'hamming');            % window */
  /*  normalized weights */
  /*  Baseband signal using convolution (provides downcoverted, filtered
   * analytic signal) */
  emxInit_real_T(&r2, 2);
  i = r2->size[0] * r2->size[1];
  r2->size[0] = 1;
  fL = r->size[1];
  r2->size[1] = r->size[1];
  emxEnsureCapacity_real_T(r2, i);
  r3 = r2->data;
  for (i = 0; i < fL; i++) {
    r3[i] = r1[i];
  }
  b_sin(r2);
  r3 = r2->data;
  b_cos(r);
  r1 = r->data;
  if (rfSignal->size[0] == r2->size[1]) {
    emxInit_creal_T(&b_rfSignal, 1);
    fL = rfSignal->size[0];
    i = b_rfSignal->size[0];
    b_rfSignal->size[0] = rfSignal->size[0];
    emxEnsureCapacity_creal_T(b_rfSignal, i);
    b_rfSignal_data = b_rfSignal->data;
    for (i = 0; i < fL; i++) {
      double d;
      d = rfSignal_data[i] - fH;
      b_rfSignal_data[i].re = d * (r3[i] + 0.0 * r1[i]);
      b_rfSignal_data[i].im = d * r1[i];
    }
    conv(b_rfSignal, basebandSignal);
    emxFree_creal_T(&b_rfSignal);
  } else {
    binary_expand_op_1(basebandSignal, rfSignal, fH, r2, r);
  }
  emxFree_real_T(&r2);
  emxFree_real_T(&r);
}

/*
 * File trailer for NoveldaDDC.c
 *
 * [EOF]
 */
