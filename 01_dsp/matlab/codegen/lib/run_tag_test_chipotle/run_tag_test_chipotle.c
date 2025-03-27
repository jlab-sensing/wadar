/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: run_tag_test_chipotle.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 27-Mar-2025 00:17:05
 */

/* Include Files */
#include "run_tag_test_chipotle.h"
#include "NoveldaChipParams.h"
#include "NoveldaDDC.h"
#include "abs.h"
#include "circshift.h"
#include "colon.h"
#include "fft.h"
#include "fileManager.h"
#include "findpeaks.h"
#include "fread.h"
#include "fullfile.h"
#include "ifWhileCond.h"
#include "mean.h"
#include "minOrMax.h"
#include "mrdivide_helper.h"
#include "mtimes.h"
#include "norm.h"
#include "proc_frames.h"
#include "pwd1.h"
#include "round.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_data.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_initialize.h"
#include "run_tag_test_chipotle_types.h"
#include "smoothdata.h"
#include "strcat.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Type Definitions */
#ifndef struct_emxArray_real_T_1
#define struct_emxArray_real_T_1
struct emxArray_real_T_1 {
  double data[1];
};
#endif /* struct_emxArray_real_T_1 */
#ifndef typedef_emxArray_real_T_1
#define typedef_emxArray_real_T_1
typedef struct emxArray_real_T_1 emxArray_real_T_1;
#endif /* typedef_emxArray_real_T_1 */

/* Function Definitions */
/*
 * results = run_tag_test(captureName, localDataPath, tagHz)
 *
 *  Function to run the tag test on Novelda radar data. This function will
 *  process the data capture file, calculate the FFT, find the peak bin, and
 *  calculate the SNR.
 *
 *  Inputs:
 *    captureName: Name of the data capture file.
 *    localDataPath: Path to the data capture file.
 *    tagHz: Tag frequency in Hz.
 *
 *  Outputs:
 *    peakBin: Peak bin of the tag frequency.
 *    SNRdB: Signal-to-noise ratio in dB.
 *
 * Arguments    : const char captureName_data[]
 *                const int captureName_size[2]
 *                const char localDataPath_data[]
 *                const int localDataPath_size[2]
 *                double tagHz
 *                double *peakBin
 *                double *SNRdB
 * Return Type  : void
 */
void run_tag_test_chipotle(const char captureName_data[],
                           const int captureName_size[2],
                           const char localDataPath_data[],
                           const int localDataPath_size[2], double tagHz,
                           double *peakBin, double *SNRdB)
{
  static const char b_cv[7] = {'X', 'X', '-', 'X', 'X', 'X', 'X'};
  static const char cv1[7] = {'X', '1', '-', 'I', 'P', 'G', 'O'};
  static const char cv2[7] = {'X', '1', '-', 'I', 'P', 'G', '1'};
  emxArray_char_T *fileName;
  emxArray_char_T *r;
  emxArray_creal_T *b_captureFT;
  emxArray_creal_T *c_captureFT;
  emxArray_creal_T *captureFT;
  emxArray_creal_T *framesBB;
  emxArray_real_T *b_fileid;
  emxArray_real_T *b_frameTot;
  emxArray_real_T *b_tagFT;
  emxArray_real_T *correlatedTagFT;
  emxArray_real_T *frameTot;
  emxArray_real_T *r1;
  emxArray_real_T *tagFT;
  emxArray_real_T *y;
  emxArray_real_T_1 c_fileid;
  creal_T *captureFT_data;
  creal_T *framesBB_data;
  double corrArray[512];
  double dacMin_data;
  double dacStep_data;
  double frameRate_data;
  double fs_hz;
  double iterations_data;
  double magic_data;
  double numFrames_data;
  double pps_data;
  double *b_frameTot_data;
  double *frameTot_data;
  int chipSet_Value_size[2];
  int b_i;
  int dacMin_size;
  int i;
  int magic_size;
  int pps_size;
  signed char fileid;
  char *fileName_data;
  bool b_magic_data;
  if (!isInitialized_run_tag_test_chipotle) {
    run_tag_test_chipotle_initialize();
  }
  /*  [frameTot, framesBB, captureFT, tagFT] = proc_frames(localDataPath,
   * captureName, tagHz, frameRate) */
  /*  */
  /*  Function to process Novelda radar data captures. This function reads in a
   */
  /*  radar data capture file and processes the data to extract the radar frames
   */
  /*  and the baseband signal.  */
  /*  */
  /*  Inputs: */
  /*    localDataPath: Path to the data capture file. */
  /*    captureName: Name of the data capture file. */
  /*    tagHz: Tag frequency in Hz. */
  /*    frameRate: Frame rate in Hz. */
  /*  */
  /*  Outputs: */
  /*    frameTot: Raw radar data frames. */
  /*    basebandSignal: Downconverted, i.e. baseband, and filtered IQ radar  */
  /*                    signal, note: use abs() on output to get envelope */
  /*                    magnitude. */
  /*  Setting default parameters */
  /*  To make the MATLAB Coder stop complaining about execution paths. */
  emxInit_creal_T(&framesBB, 2);
  i = framesBB->size[0] * framesBB->size[1];
  framesBB->size[0] = 1;
  framesBB->size[1] = 1;
  emxEnsureCapacity_creal_T(framesBB, i);
  framesBB_data = framesBB->data;
  framesBB_data[0].re = 0.0;
  framesBB_data[0].im = 0.0;
  frameRate_data = 0.0;
  /*  Load Capture */
  emxInit_char_T(&fileName);
  pwd(fileName);
  emxInit_char_T(&r);
  b_strcat(fileName, localDataPath_data, localDataPath_size, r);
  fullfile(r, captureName_data, captureName_size, fileName);
  emxFree_char_T(&r);
  fileid = cfopen(fileName);
  magic_size = b_fread(fileid, (double *)&magic_data);
  /*  Check the magic number */
  for (i = 0; i < magic_size; i++) {
    b_magic_data = (magic_data != 4.27805917E+9);
  }
  emxInit_real_T(&frameTot, 2);
  emxInit_real_T(&b_frameTot, 1);
  emxInit_creal_T(&captureFT, 1);
  emxInit_real_T(&r1, 2);
  emxInit_real_T(&b_fileid, 1);
  if (ifWhileCond((bool *)&b_magic_data, magic_size)) {
    i = fileName->size[0] * fileName->size[1];
    fileName->size[0] = 1;
    fileName->size[1] = captureName_size[1] + 1;
    emxEnsureCapacity_char_T(fileName, i);
    fileName_data = fileName->data;
    magic_size = captureName_size[1];
    for (i = 0; i < magic_size; i++) {
      fileName_data[i] = captureName_data[i];
    }
    fileName_data[captureName_size[1]] = '\x00';
    printf("Wrong data format: %s!\n", &fileName_data[0]);
    fflush(stdout);
    fileManager();
  } else {
    double pgen_data;
    int pgen_size;
    char chipSet_Value_data[7];
    c_fread(fileid, (double *)&iterations_data);
    /*  Sweep controller settings */
    pps_size = c_fread(fileid, (double *)&pps_data);
    dacMin_size = c_fread(fileid, (double *)&dacMin_data);
    c_fread(fileid, c_fileid.data);
    c_fread(fileid, (double *)&dacStep_data);
    magic_size = c_fread(fileid, (double *)&magic_data);
    /*  2 for X2 (Ancho), 10 for X1-IPGO (Cayenne), 11 for X1-IPG1 (Chipotle) */
    /*  Dummy variables because the MATLAB  Coder keeps yelling at me */
    chipSet_Value_size[0] = 1;
    chipSet_Value_size[1] = 7;
    for (i = 0; i < 7; i++) {
      chipSet_Value_data[i] = b_cv[i];
    }
    pgen_size = 1;
    pgen_data = 0.0;
    for (i = 0; i < magic_size; i++) {
      b_magic_data = (magic_data == 2.0);
    }
    if (ifWhileCond((bool *)&b_magic_data, magic_size)) {
      magic_size = 0;
    } else {
      for (i = 0; i < magic_size; i++) {
        b_magic_data = (magic_data == 10.0);
      }
      if (ifWhileCond((bool *)&b_magic_data, magic_size)) {
        magic_size = 1;
      } else {
        for (i = 0; i < magic_size; i++) {
          b_magic_data = (magic_data == 11.0);
        }
        if (ifWhileCond((bool *)&b_magic_data, magic_size)) {
          magic_size = 2;
        } else {
          magic_size = -1;
        }
      }
    }
    switch (magic_size) {
    case 0:
      chipSet_Value_size[0] = 1;
      chipSet_Value_size[1] = 2;
      chipSet_Value_data[0] = 'X';
      chipSet_Value_data[1] = '2';
      d_fread(fileid, c_fileid.data);
      /*  Sampling Rate */
      pgen_size = c_fread(fileid, (double *)&pgen_data);
      /*  Radar Specifier settings */
      d_fread(fileid, c_fileid.data);
      d_fread(fileid, c_fileid.data);
      break;
    case 1:
      chipSet_Value_size[0] = 1;
      chipSet_Value_size[1] = 7;
      for (i = 0; i < 7; i++) {
        chipSet_Value_data[i] = cv1[i];
      }
      e_fread(fileid, c_fileid.data);
      /*  Sampling Rate */
      pgen_size = c_fread(fileid, (double *)&pgen_data);
      /*  Radar Specifier settings */
      c_fread(fileid, c_fileid.data);
      c_fread(fileid, c_fileid.data);
      break;
    case 2:
      chipSet_Value_size[0] = 1;
      chipSet_Value_size[1] = 7;
      for (i = 0; i < 7; i++) {
        chipSet_Value_data[i] = cv2[i];
      }
      e_fread(fileid, c_fileid.data);
      /*  Sampling Rate */
      pgen_size = c_fread(fileid, (double *)&pgen_data);
      /*  Radar Specifier settings */
      c_fread(fileid, c_fileid.data);
      c_fread(fileid, c_fileid.data);
      break;
    }
    c_fread(fileid, (double *)&magic_data);
    /*  Number of samplers in a frame */
    c_fread(fileid, (double *)&numFrames_data);
    /*  Number of frames in a capture */
    c_fread(fileid, c_fileid.data);
    /*  Number of runs in capture */
    magic_size = c_fread(fileid, (double *)&frameRate_data);
    /*  Frames per second */
    f_fread(fileid, numFrames_data, b_fileid);
    g_fread(fileid, numFrames_data * magic_data, b_frameTot);
    /*  Radar frames */
    /*  DAC normalization */
    for (i = 0; i < pps_size; i++) {
      pps_data *= iterations_data;
    }
    mrdiv(b_frameTot, (double *)&pps_data, pps_size, r1);
    mtimes(r1, (double *)&dacStep_data, b_frameTot);
    frameTot_data = b_frameTot->data;
    if (b_frameTot->size[0] == dacMin_size) {
      pps_size = b_frameTot->size[0];
      for (i = 0; i < pps_size; i++) {
        frameTot_data[0] += dacMin_data;
      }
      i = frameTot->size[0] * frameTot->size[1];
      frameTot->size[0] = (int)magic_data;
      frameTot->size[1] = (int)numFrames_data;
      emxEnsureCapacity_real_T(frameTot, i);
      b_frameTot_data = frameTot->data;
      magic_size = (int)magic_data * (int)numFrames_data;
      for (i = 0; i < magic_size; i++) {
        b_frameTot_data[i] = frameTot_data[i];
      }
    } else {
      binary_expand_op(frameTot, b_frameTot, (double *)&dacMin_data,
                       &dacMin_size, (double *)&magic_data,
                       (double *)&numFrames_data);
      b_frameTot_data = frameTot->data;
    }
    /*  Process out the weird spike */
    i = (int)numFrames_data;
    for (b_i = 0; b_i < i; b_i++) {
      magic_size = frameTot->size[0];
      dacMin_size = b_frameTot->size[0];
      b_frameTot->size[0] = frameTot->size[0];
      emxEnsureCapacity_real_T(b_frameTot, dacMin_size);
      frameTot_data = b_frameTot->data;
      for (dacMin_size = 0; dacMin_size < magic_size; dacMin_size++) {
        frameTot_data[dacMin_size] =
            b_frameTot_data[dacMin_size + frameTot->size[0] * b_i];
      }
      if (maximum(b_frameTot) > 8191.0) {
        if ((unsigned int)b_i + 1U > 1U) {
          dacMin_size = b_frameTot->size[0];
          b_frameTot->size[0] = frameTot->size[0];
          emxEnsureCapacity_real_T(b_frameTot, dacMin_size);
          frameTot_data = b_frameTot->data;
          for (dacMin_size = 0; dacMin_size < magic_size; dacMin_size++) {
            frameTot_data[dacMin_size] =
                b_frameTot_data[dacMin_size + frameTot->size[0] * (b_i - 1)];
          }
          for (dacMin_size = 0; dacMin_size < magic_size; dacMin_size++) {
            b_frameTot_data[dacMin_size + frameTot->size[0] * b_i] =
                frameTot_data[dacMin_size];
          }
        } else {
          dacMin_size = b_frameTot->size[0];
          b_frameTot->size[0] = frameTot->size[0];
          emxEnsureCapacity_real_T(b_frameTot, dacMin_size);
          frameTot_data = b_frameTot->data;
          for (dacMin_size = 0; dacMin_size < magic_size; dacMin_size++) {
            frameTot_data[dacMin_size] =
                b_frameTot_data[dacMin_size + frameTot->size[0]];
          }
          for (dacMin_size = 0; dacMin_size < magic_size; dacMin_size++) {
            b_frameTot_data[dacMin_size] = frameTot_data[dacMin_size];
          }
        }
      }
    }
    d_fread(fileid, c_fileid.data);
    /*  Estimated FPS (good to check against frameRate) */
    /*  TODO - insert comment here */
    magic_data = h_fread(fileid, b_frameTot);
    cfclose(fileid);
    if (magic_data != 0.0) {
      printf("FILE READ ERROR: %f data remains! Check that file format matches "
             "read code\n",
             floor(magic_data));
      fflush(stdout);
    } else {
      /*  Unused but potentially useful radar parameters */
      NoveldaChipParams(chipSet_Value_data, chipSet_Value_size,
                        (double *)&pgen_data, pgen_size, &iterations_data,
                        &pps_data, &dacMin_data, &dacStep_data, &numFrames_data,
                        &magic_data, &fs_hz);
      /*  Baseband Conversion */
      i = framesBB->size[0] * framesBB->size[1];
      framesBB->size[0] = frameTot->size[0];
      framesBB->size[1] = frameTot->size[1];
      emxEnsureCapacity_creal_T(framesBB, i);
      framesBB_data = framesBB->data;
      magic_size = frameTot->size[0] * frameTot->size[1];
      for (i = 0; i < magic_size; i++) {
        framesBB_data[i].re = 0.0;
        framesBB_data[i].im = 0.0;
      }
      i = frameTot->size[1];
      for (b_i = 0; b_i < i; b_i++) {
        magic_size = frameTot->size[0];
        dacMin_size = b_frameTot->size[0];
        b_frameTot->size[0] = frameTot->size[0];
        emxEnsureCapacity_real_T(b_frameTot, dacMin_size);
        frameTot_data = b_frameTot->data;
        for (dacMin_size = 0; dacMin_size < magic_size; dacMin_size++) {
          frameTot_data[dacMin_size] =
              b_frameTot_data[dacMin_size + frameTot->size[0] * b_i];
        }
        NoveldaDDC(b_frameTot, chipSet_Value_data, chipSet_Value_size,
                   (double *)&pgen_data, pgen_size, fs_hz, captureFT);
        captureFT_data = captureFT->data;
        pps_size = framesBB->size[0];
        for (dacMin_size = 0; dacMin_size < pps_size; dacMin_size++) {
          framesBB_data[dacMin_size + framesBB->size[0] * b_i] =
              captureFT_data[dacMin_size];
        }
      }
    }
  }
  emxFree_real_T(&b_fileid);
  emxFree_real_T(&r1);
  emxFree_char_T(&fileName);
  emxFree_real_T(&frameTot);
  /*  [captureFT, tagFT] = proc_fft(framesBB, frameRate, tagHz) */
  /*  */
  /*  Function to process the FFT of Novelda radar data captures. This function
   * reads in */
  /*  the already processed radar data frames and processes the data to extract
   * the FFT */
  /*  of the radar frames and the FFT of the tag signal. This is a Stage 2
   * processing */
  /*  function. */
  /*  */
  /*  Inputs: */
  /*    framesBB: Downconverted, i.e. baseband, and filtered IQ radar signal. */
  /*    frameRate: Frame rate in Hz. */
  /*  */
  /*  Outputs: */
  /*   captureFT: FFT of the radar frames. */
  /*   tagFT: FFT of the tag signal. */
  /*  Capture FT */
  emxInit_creal_T(&b_captureFT, 2);
  fft(framesBB, framesBB->size[1], b_captureFT);
  framesBB_data = b_captureFT->data;
  /*  Find Tag FT */
  /*  [tagFT, tagIndex] = tag_index(captureFT, frameRate, tagHz) */
  /*  */
  /*  Function to find the tag frequency index in the FFT of the radar data */
  /*  captures.  */
  /*  */
  /*  Inputs: */
  /*    captureFT: FFT of the radar frames */
  /*    frameRate: Frame rate in Hz */
  /*    tagHz: Tag frequency in Hz */
  /*  */
  /*  Outputs: */
  /*    tagFT: FFT of the tag signal */
  /*  Find Tag FT */
  pps_data = tagHz / frameRate_data;
  iterations_data = pps_data * (double)b_captureFT->size[1];
  pps_size = b_captureFT->size[0];
  i = captureFT->size[0];
  captureFT->size[0] = b_captureFT->size[0];
  emxEnsureCapacity_creal_T(captureFT, i);
  captureFT_data = captureFT->data;
  for (i = 0; i < pps_size; i++) {
    captureFT_data[i] =
        framesBB_data[i + b_captureFT->size[0] * ((int)iterations_data - 1)];
  }
  emxInit_real_T(&tagFT, 1);
  b_abs(captureFT, tagFT);
  for (b_i = 0; b_i < 5; b_i++) {
    magic_size = (int)(iterations_data - 2.0) + b_i;
    i = captureFT->size[0];
    captureFT->size[0] = pps_size;
    emxEnsureCapacity_creal_T(captureFT, i);
    captureFT_data = captureFT->data;
    for (i = 0; i < pps_size; i++) {
      captureFT_data[i] =
          framesBB_data[i + b_captureFT->size[0] * (magic_size - 1)];
    }
    b_abs(captureFT, b_frameTot);
    frameTot_data = b_frameTot->data;
    if (maximum(b_frameTot) > maximum(tagFT)) {
      magic_size = b_frameTot->size[0];
      i = tagFT->size[0];
      tagFT->size[0] = b_frameTot->size[0];
      emxEnsureCapacity_real_T(tagFT, i);
      b_frameTot_data = tagFT->data;
      for (i = 0; i < magic_size; i++) {
        b_frameTot_data[i] = frameTot_data[i];
      }
    }
  }
  smoothdata(tagFT);
  /*  [tagPeakBin] = tag_correlation(correlatedTagFT, tagFT) */
  /*  */
  /*  Function to get the peak bin of the tag signal in the radar data capture.
   */
  /*  This function uses the correlatedTagFT and tagFT to find the peak bin of
   */
  /*  the tag signal using Pearson correlation. This is a Stage 3 processing */
  /*  function. */
  /*  */
  /*  Inputs: */
  /*    correlatedTagFT: FFT of the correlated tag signal. This should be a
   * capture */
  /*                     of the tag where the tag's signal is very strong.
   * Optimal */
  /*                     capture requires free space permittivity (aka air) and
   */
  /*                     a clear line of sight between the radar and the tag. */
  /*    tagFT: FFT of the tag signal.  */
  /*  */
  /*  Outputs: */
  /*    tagPeakBin: Peak bin of the tag signal in the radar data capture. */
  /*  Find the bin corresponding to the largest peak in the correlated FT */
  magic_data = maximum(tagFT);
  emxInit_real_T(&correlatedTagFT, 1);
  findpeaks(tagFT, magic_data * 0.9, b_frameTot, correlatedTagFT);
  /*  Find the bin corresponding to the largest peak in the tag FT */
  emxInit_real_T(&b_tagFT, 1);
  findpeaks(tagFT, magic_data * 0.9, b_frameTot, b_tagFT);
  b_frameTot_data = b_tagFT->data;
  /*  Commented out for now */
  /*  if (size(tagFT, 1) > 1) */
  /*      if (tagFT(2) - tagFT(1) < 50)  % if double peak, invalid radar capture
   */
  /*          procSuccess = false; */
  /*          fprintf("Double peak detected. Please reorient the radar.\n") */
  /*      end */
  /*  end */
  /*  Select peak bin correlated to template capture */
  magic_data = b_norm(correlatedTagFT);
  iterations_data = magic_data - fabs(magic_data);
  magic_data -= magic_data;
  for (b_i = 0; b_i < 512; b_i++) {
    /*  Pearson Correlation */
    corrArray[b_i] =
        iterations_data * magic_data /
        sqrt(iterations_data * iterations_data * (magic_data * magic_data));
  }
  circshift(corrArray, correlatedTagFT);
  emxFree_real_T(&correlatedTagFT);
  b_maximum(corrArray, &magic_size);
  *peakBin = b_frameTot_data[0];
  i = b_tagFT->size[0];
  for (b_i = 0; b_i < i; b_i++) {
    if (fabs(b_frameTot_data[b_i] - (double)magic_size) < *peakBin) {
      *peakBin = b_frameTot_data[b_i];
    }
  }
  emxFree_real_T(&b_tagFT);
  /*  Find tag frequency index */
  iterations_data = pps_data * (double)framesBB->size[1];
  emxFree_creal_T(&framesBB);
  i = captureFT->size[0];
  captureFT->size[0] = b_captureFT->size[0];
  emxEnsureCapacity_creal_T(captureFT, i);
  captureFT_data = captureFT->data;
  for (i = 0; i < pps_size; i++) {
    captureFT_data[i] =
        framesBB_data[i + b_captureFT->size[0] * ((int)iterations_data - 1)];
  }
  b_abs(captureFT, tagFT);
  magic_size = 0;
  for (b_i = 0; b_i < 5; b_i++) {
    magic_data = (iterations_data - 2.0) + (double)b_i;
    i = captureFT->size[0];
    captureFT->size[0] = pps_size;
    emxEnsureCapacity_creal_T(captureFT, i);
    captureFT_data = captureFT->data;
    for (i = 0; i < pps_size; i++) {
      captureFT_data[i] =
          framesBB_data[i + b_captureFT->size[0] * ((int)magic_data - 1)];
    }
    b_abs(captureFT, b_frameTot);
    frameTot_data = b_frameTot->data;
    if (maximum(b_frameTot) > maximum(tagFT)) {
      magic_size = b_frameTot->size[0];
      i = tagFT->size[0];
      tagFT->size[0] = b_frameTot->size[0];
      emxEnsureCapacity_real_T(tagFT, i);
      b_frameTot_data = tagFT->data;
      for (i = 0; i < magic_size; i++) {
        b_frameTot_data[i] = frameTot_data[i];
      }
      magic_size = (int)magic_data;
    }
  }
  emxFree_creal_T(&captureFT);
  emxFree_real_T(&tagFT);
  emxFree_real_T(&b_frameTot);
  /*  SNRdb = tag_snr(captureFT, freqIndex, peakBin) */
  /*  */
  /*  Function to calculate the signal-to-noise ratio (SNR) in decibels of a */
  /*  tag signal in the FFT of radar data captures. */
  /*  */
  /*  Inputs: */
  /*    captureFT: FFT of the radar frames */
  /*    freqIndex: Index of the tag frequency in the FFT */
  /*    peakBin: Peak bin of the tag signal */
  /*  */
  /*  Outputs: */
  /*    SNRdb: Signal-to-noise ratio in decibels */
  emxInit_real_T(&y, 2);
  magic_data = (double)magic_size * 0.945;
  iterations_data = (double)magic_size * 0.955;
  if (iterations_data < magic_data) {
    y->size[0] = 1;
    y->size[1] = 0;
  } else if (floor(magic_data) == magic_data) {
    i = y->size[0] * y->size[1];
    y->size[0] = 1;
    pps_size = (int)(iterations_data - magic_data);
    y->size[1] = pps_size + 1;
    emxEnsureCapacity_real_T(y, i);
    b_frameTot_data = y->data;
    for (i = 0; i <= pps_size; i++) {
      b_frameTot_data[i] = magic_data + (double)i;
    }
  } else {
    eml_float_colon(magic_data, iterations_data, y);
  }
  b_round(y);
  b_frameTot_data = y->data;
  emxInit_creal_T(&c_captureFT, 2);
  i = c_captureFT->size[0] * c_captureFT->size[1];
  c_captureFT->size[0] = 1;
  pps_size = y->size[1];
  c_captureFT->size[1] = y->size[1];
  emxEnsureCapacity_creal_T(c_captureFT, i);
  captureFT_data = c_captureFT->data;
  for (i = 0; i < pps_size; i++) {
    captureFT_data[i] =
        framesBB_data[((int)*peakBin +
                       b_captureFT->size[0] * ((int)b_frameTot_data[i] - 1)) -
                      1];
  }
  d_abs(c_captureFT, y);
  emxFree_creal_T(&c_captureFT);
  *SNRdB =
      10.0 * log10(c_abs(framesBB_data[((int)*peakBin + b_captureFT->size[0] *
                                                            (magic_size - 1)) -
                                       1]) /
                   b_mean(y));
  emxFree_real_T(&y);
  emxFree_creal_T(&b_captureFT);
}

/*
 * File trailer for run_tag_test_chipotle.c
 *
 * [EOF]
 */
