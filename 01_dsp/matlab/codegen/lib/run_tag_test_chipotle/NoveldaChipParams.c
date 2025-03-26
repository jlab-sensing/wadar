/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: NoveldaChipParams.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

/* Include Files */
#include "NoveldaChipParams.h"
#include "ifWhileCond.h"
#include "rt_nonfinite.h"
#include "string1.h"
#include <string.h>

/* Function Definitions */
/*
 * Look up table to extract all Novelda pulse parameters depending on the
 *  function [fc, bw, bwr, vp, n, bw_hz, pwr_dBm, fs_hz] =
 * NoveldaChipParams(chipSet, PGen, Sampler)
 *
 *
 *  Inputs:
 *    chipSet:    'X1-IPG0' = NVA6100, medium-band pgen
 *                'X1-IPG1' = NVA6100, low-band pgen
 *                'X2' = NVA6201/2
 *                'X4' = X4 SoC
 *
 *    PGen:       0 = slow, 1 = nominal, 3 = fast, for 'X1'
 *                0-11 (CF = 5.3 GHz to 9.1 GHz), for 'X2'
 *                0 or 3: CF = 7.29 GHz (ETSI/FCC Compliant), 1 or 4: CF = 8.748
 * GHz (KCC/FCC Compliant) for 'X4'
 *
 *    Sampler:    '4mm' = 39 GS/s, '8mm' = 20 GS/s, or '4cm' = 3.8 GS/s, for
 * 'X1' '4mm' only for 'X2' or just omit value '6mm' only for 'X4' or just omit
 * value
 *
 *  Outputs:
 *    fc      = center frequency
 *    bw      = fractional bandwidth
 *    bwr     = dB down for bandwidth extents
 *    vp      = instantaneous output voltage peak amplitude
 *    n       = frameSize, i.e. number of samplers
 *    bw_hz   = bandwidth in hertz
 *    pwr_dBm = mean/average output power in dBm (i.e. true RMS)
 *    fs_hz   = sampling rate in Hz
 *
 *
 *  Update 9/26/2017, Added X4 support
 *
 *
 * Arguments    : const char chipSet_Value_data[]
 *                const int chipSet_Value_size[2]
 *                const double PGen_data[]
 *                int PGen_size
 *                double *bw
 *                double *bwr
 *                double *vp
 *                double *n
 *                double *bw_hz
 *                double *pwr_dBm
 *                double *fs_hz
 * Return Type  : double
 */
double NoveldaChipParams(const char chipSet_Value_data[],
                         const int chipSet_Value_size[2],
                         const double PGen_data[], int PGen_size, double *bw,
                         double *bwr, double *vp, double *n, double *bw_hz,
                         double *pwr_dBm, double *fs_hz)
{
  double fH;
  double fc;
  int switch_expression_Value_size[2];
  int b_index;
  int fL;
  int i;
  int i1;
  char switch_expression_Value_data[7];
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
    boolean_T b_PGen_data;
    /* -------------------NVA6100, Medium-Band PGen--------------------%% */
    *bwr = 12.0;
    /*  bwr dB down from normalized peak setting for fractional bandwidth
     * (changed to 6 dB power spectra bw, looks better) */
    *n = 512.0;
    /*  # of samplers, i.e. length of frame */
    /*  Pulse Generator */
    for (i = 0; i < PGen_size; i++) {
      b_PGen_data = (PGen_data[0] == 0.0);
    }
    if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
      b_index = 0;
    } else {
      for (i = 0; i < PGen_size; i++) {
        b_PGen_data = (PGen_data[0] == 1.0);
      }
      if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
        b_index = 1;
      } else {
        for (i = 0; i < PGen_size; i++) {
          b_PGen_data = (PGen_data[0] == 2.0);
        }
        if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
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
      *vp = 0.47;
      /*  470 mV instantaneous output amplitude (peak voltage)     */
      break;
    case 1:
      fL = 845000000;
      /*  -10 dB low cutoff */
      fH = 9.55E+9;
      /*  -10 dB high cutoff */
      *vp = 0.45;
      /*  450 mV instantaneous output amplitude (peak voltage)   */
      break;
    case 2:
      fL = 1060000000;
      /*  -10 dB low cutoff */
      fH = 1.041E+10;
      /*  -10 dB high cutoff */
      *vp = 0.37;
      /*  370 mV instantaneous output amplitude (peak voltage)   */
      break;
    }
    /*  Sampling Rate */
    *fs_hz = 3.9E+10;
    /*  mean system sampling rate for 4mm */
    /*  Center frequency, bandwidth in hertz, and fractional bandwidth */
    fc = (fH + (double)fL) / 2.0;
    *bw_hz = fH - (double)fL;
    *bw = *bw_hz / fc;
    /*  Mean/Average output power in dBm (i.e. true RMS), Nominal, */
    /*  function of PRF @ 50 MHz */
    *pwr_dBm = -19.0;
  } break;
  case 1: {
    boolean_T b_PGen_data;
    /* ---------------------NVA6100, Low-Band PGen---------------------%% */
    *bwr = 12.0;
    /*  bwr dB down from normalized peak setting for fractional bandwidth
     * (change to 6 dB power spectra bw, looks better) */
    *n = 512.0;
    /*  # of samplers, i.e. length of frame */
    *vp = 0.5;
    /*  500 mV instantaneous output amplitude (peak voltage)   */
    /*  Pulse Generator */
    for (i = 0; i < PGen_size; i++) {
      b_PGen_data = (PGen_data[0] == 0.0);
    }
    if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
      b_index = 0;
    } else {
      for (i = 0; i < PGen_size; i++) {
        b_PGen_data = (PGen_data[0] == 1.0);
      }
      if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
        b_index = 1;
      } else {
        for (i = 0; i < PGen_size; i++) {
          b_PGen_data = (PGen_data[0] == 2.0);
        }
        if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
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
    *fs_hz = 3.9E+10;
    /*  mean system sampling rate for 4mm */
    /*  Center frequency, bandwidth in hertz, and fractional bandwidth */
    fc = (fH + (double)fL) / 2.0;
    *bw_hz = fH - (double)fL;
    *bw = *bw_hz / fc;
    /*  Mean/Average output power in dBm (i.e. true RMS), Nominal, */
    /*  function of PRF @ 50 MHz */
    *pwr_dBm = -14.0;
  } break;
  case 2: {
    boolean_T b_PGen_data;
    /* -----------------------------NVA6201---------------------------%% */
    *bwr = 10.0;
    /*  bwr dB down from normalized peak setting for fractional bandwidth (5 dB
     * power spectra bw) */
    *n = 256.0;
    /*  # of samplers, i.e. length of frame */
    *fs_hz = 3.9E+10;
    /*  mean system sampling rate, 39 GS/s */
    /*  Pulse Generator */
    for (i = 0; i < PGen_size; i++) {
      b_PGen_data = (PGen_data[0] == 0.0);
    }
    if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
      b_index = 0;
    } else {
      for (i = 0; i < PGen_size; i++) {
        b_PGen_data = (PGen_data[0] == 1.0);
      }
      if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
        b_index = 1;
      } else {
        for (i = 0; i < PGen_size; i++) {
          b_PGen_data = (PGen_data[0] == 2.0);
        }
        if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
          b_index = 2;
        } else {
          for (i = 0; i < PGen_size; i++) {
            b_PGen_data = (PGen_data[0] == 3.0);
          }
          if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
            b_index = 3;
          } else {
            for (i = 0; i < PGen_size; i++) {
              b_PGen_data = (PGen_data[0] == 4.0);
            }
            if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
              b_index = 4;
            } else {
              for (i = 0; i < PGen_size; i++) {
                b_PGen_data = (PGen_data[0] == 5.0);
              }
              if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
                b_index = 5;
              } else {
                for (i = 0; i < PGen_size; i++) {
                  b_PGen_data = (PGen_data[0] == 6.0);
                }
                if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
                  b_index = 6;
                } else {
                  for (i = 0; i < PGen_size; i++) {
                    b_PGen_data = (PGen_data[0] == 7.0);
                  }
                  if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
                    b_index = 7;
                  } else {
                    for (i = 0; i < PGen_size; i++) {
                      b_PGen_data = (PGen_data[0] == 8.0);
                    }
                    if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
                      b_index = 8;
                    } else {
                      for (i = 0; i < PGen_size; i++) {
                        b_PGen_data = (PGen_data[0] == 9.0);
                      }
                      if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
                        b_index = 9;
                      } else {
                        for (i = 0; i < PGen_size; i++) {
                          b_PGen_data = (PGen_data[0] == 10.0);
                        }
                        if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
                          b_index = 10;
                        } else {
                          for (i = 0; i < PGen_size; i++) {
                            b_PGen_data = (PGen_data[0] == 11.0);
                          }
                          if (ifWhileCond((boolean_T *)&b_PGen_data,
                                          PGen_size)) {
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
      fc = 5.3E+9;
      /*  center frequency */
      *bw_hz = 1.75E+9;
      /*  bandwidth in hertz */
      *bw = 0.330188679245283;
      /*  fractional bandwidth (~1.75 GHz @ 5.3 GHz) */
      *vp = 0.345;
      /*  690 mV instanteous output voltage peak-to-peak */
      *pwr_dBm = -10.7;
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 1:
      /*  PGSelect = 1; */
      fc = 5.4E+9;
      /*  center frequency */
      *bw_hz = 1.8E+9;
      /*  bandwidth in hertz */
      *bw = 0.33333333333333331;
      /*  fractional bandwidth (~1.80 GHz @ 5.4 GHz) */
      *vp = 0.345;
      /*  690 mV instanteous voltage peak-to-peak  */
      *pwr_dBm = -10.8;
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 2:
      /*  PGSelect = 2; */
      fc = 5.7E+9;
      /*  center frequency */
      *bw_hz = 1.85E+9;
      /*  bandwidth in hertz */
      *bw = 0.32456140350877194;
      /*  fractional bandwidth (~1.85 GHz @ 5.7 GHz) */
      *vp = 0.36;
      /*  720 mV instanteous voltage peak-to-peak  */
      *pwr_dBm = -11.2;
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 3:
      /*  PGSelect = 3; */
      fc = 6.1E+9;
      /*  center frequency */
      *bw_hz = 2.05E+9;
      /*  bandwidth in hertz */
      *bw = 0.33606557377049179;
      /*  fractional bandwidth (~2.05 GHz @ 6.1 GHz) */
      *vp = 0.355;
      /*  710 mV instanteous voltage peak-to-peak */
      *pwr_dBm = -11.8;
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 4:
      /*  PGSelect = 4; */
      fc = 6.4E+9;
      /*  center frequency */
      *bw_hz = 2.15E+9;
      /*  bandwidth in hertz */
      *bw = 0.3359375;
      /*  fractional bandwidth (~2.15 GHz @ 6.4 GHz) */
      *vp = 0.36;
      /*  720 mV instanteous voltage peak-to-peak  */
      *pwr_dBm = -12.0;
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 5:
      /*  PGSelect = 5; */
      fc = 6.8E+9;
      /*  center frequency */
      *bw_hz = 2.3E+9;
      /*  bandwidth in hertz */
      *bw = 0.33823529411764708;
      /*  fractional bandwidth (~2.30 GHz @ 6.8 GHz) */
      *vp = 0.345;
      /*  690 mV instanteous voltage peak-to-peak */
      *pwr_dBm = -12.6;
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 6:
      /*  PGSelect = 6; */
      fc = 7.3E+9;
      /*  center frequency */
      *bw_hz = 2.35E+9;
      /*  bandwidth in hertz */
      *bw = 0.32191780821917809;
      /*  fractional bandwidth (~2.35 GHz @ 7.3 GHz) */
      *vp = 0.325;
      /*  650 mV instanteous voltage peak-to-peak  */
      *pwr_dBm = -13.3;
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 7:
      /*  PGSelect = 7; */
      fc = 7.7E+9;
      /*  center frequency */
      *bw_hz = 2.5E+9;
      /*  bandwidth in hertz */
      *bw = 0.32467532467532467;
      /*  fractional bandwidth (~2.50 GHz @ 7.7 GHz) */
      *vp = 0.31;
      /*  620 mV instanteous output voltage peak-to-peak  */
      *pwr_dBm = -14.0;
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 8:
      /*  PGSelect = 8; */
      fc = 7.8E+9;
      /*  center frequency */
      *bw_hz = 2.5E+9;
      /*  bandwidth in hertz */
      *bw = 0.32051282051282054;
      /*  fractional bandwidth (~2.50 GHz @ 7.8 GHz) */
      *vp = 0.31;
      /*  620 mV instanteous output voltage peak-to-peak  */
      *pwr_dBm = -14.0;
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 9:
      /*  PGSelect = 9; */
      fc = 8.2E+9;
      /*  center frequency */
      *bw_hz = 2.65E+9;
      /*  bandwidth in hertz */
      *bw = 0.32317073170731708;
      /*  fractional bandwidth (~2.65 GHz @ 8.2 GHz) */
      *vp = 0.285;
      /*  570 mV instanteous output voltage peak-to-peak */
      *pwr_dBm = -14.8;
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 10:
      /*  PGSelect = 10 */
      fc = 8.8E+9;
      /*  center frequency */
      *bw_hz = 3.1E+9;
      /*  bandwidth in hertz */
      *bw = 0.35227272727272729;
      /*  fractional bandwidth (~3.10 GHz @ 8.8 GHz) */
      *vp = 0.27;
      /*  540 mV instanteous output voltage peak-to-peak */
      *pwr_dBm = -16.4;
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    case 11:
      /*  PGSelect = 11 */
      fc = 9.1E+9;
      /*  center frequency */
      *bw_hz = 3.1E+9;
      /*  bandwidth in hertz....not sure about this? */
      *bw = 0.34065934065934067;
      /*  fractional bandwidth (~3.10 GHz @ 8.8 GHz) */
      *vp = 0.26;
      /*  540 mV instanteous output voltage peak-to-peak */
      *pwr_dBm = -16.7;
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF @ 100
       * MHz) */
      break;
    }
  } break;
  case 3: {
    int exitg2;
    boolean_T b_PGen_data;
    /* ------------------X4, Impulse Radar Transceiver SoC-----------------%% */
    *bwr = 10.0;
    /*  bwr dB down from normalized peak setting for fractional bandwidth (5 dB
     * power spectra bw) */
    *n = 1536.0;
    /*  # of samplers, i.e. length of frame */
    *fs_hz = 2.3328E+10;
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
        if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
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
      boolean_T exitg1;
      exitg1 = false;
      while ((!exitg1) && (fL < 2)) {
        i = 3 * fL + 1;
        for (i1 = 0; i1 < PGen_size; i1++) {
          b_PGen_data = (i == PGen_data[0]);
        }
        if (ifWhileCond((boolean_T *)&b_PGen_data, PGen_size)) {
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
      fc = 7.29E+9;
      /*  center frequency */
      *bw_hz = 1.4E+9;
      /*  bandwidth in hertz....not sure about this? */
      *bw = 0.19204389574759945;
      /*  fractional bandwidth (~3.10 GHz @ 8.8 GHz) */
      *vp = 0.31;
      /*  310 mV instanteous output voltage peak */
      *pwr_dBm = -14.0;
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF
       * @ 60.75 MHz) */
      break;
    case 1:
      /*  KCC/FCC */
      fc = 8.748E+9;
      /*  center frequency */
      *bw_hz = 1.5E+9;
      /*  bandwidth in hertz....not sure about this? */
      *bw = 0.17146776406035666;
      /*  fractional bandwidth (~3.10 GHz @ 8.8 GHz) */
      *vp = 0.31;
      /*  310 mV instanteous output voltage peak */
      *pwr_dBm = -14.0;
      /*  mean/average output power in dBm (i.e. true RMS, function of PRF
       * @ 60.75 MHz) */
      break;
    }
  } break;
  }
  return fc;
}

/*
 * File trailer for NoveldaChipParams.c
 *
 * [EOF]
 */
