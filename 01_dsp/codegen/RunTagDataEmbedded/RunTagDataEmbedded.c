/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: RunTagDataEmbedded.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

/* Include Files */
#include "RunTagDataEmbedded.h"
#include "NoveldaChipParams.h"
#include "NoveldaDDC.h"
#include "ProcessFrames.h"
#include "RunTagDataEmbedded_data.h"
#include "RunTagDataEmbedded_emxutil.h"
#include "RunTagDataEmbedded_initialize.h"
#include "RunTagDataEmbedded_types.h"
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
#include "pwd1.h"
#include "round.h"
#include "rt_nonfinite.h"
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
 * Arguments    : const char captureName_data[]
 *                const int captureName_size[2]
 *                const char localDataPath_data[]
 *                const int localDataPath_size[2]
 *                double tagHz
 *                double *peakBin
 *                double *SNRdB
 * Return Type  : void
 */
void RunTagDataEmbedded(const char captureName_data[],
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
  emxArray_real_T *a__1;
  emxArray_real_T *a__2;
  emxArray_real_T *b_fileid;
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
  double *a__1_data;
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
  if (!isInitialized_RunTagDataEmbedded) {
    RunTagDataEmbedded_initialize();
  }
  /*  [frameTot, framesBB, captureFT, tagFT] = ProcessFrames(localDataPath,
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
  /*  MEX generation keeps complaining if I don't do this */
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
  emxInit_real_T(&a__1, 2);
  emxInit_real_T(&frameTot, 1);
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
    /*  Number of frames in capture */
    c_fread(fileid, c_fileid.data);
    /*  Number of runs in capture */
    magic_size = c_fread(fileid, (double *)&frameRate_data);
    /*  Frames per second */
    f_fread(fileid, numFrames_data, b_fileid);
    /*  Array of time data */
    g_fread(fileid, numFrames_data * magic_data, frameTot);
    /*  Radar frames */
    /*  DAC normalization */
    for (i = 0; i < pps_size; i++) {
      pps_data *= iterations_data;
    }
    mrdiv(frameTot, (double *)&pps_data, pps_size, r1);
    mtimes(r1, (double *)&dacStep_data, frameTot);
    frameTot_data = frameTot->data;
    if (frameTot->size[0] == dacMin_size) {
      pps_size = frameTot->size[0];
      for (i = 0; i < pps_size; i++) {
        frameTot_data[0] += dacMin_data;
      }
      i = a__1->size[0] * a__1->size[1];
      a__1->size[0] = (int)magic_data;
      a__1->size[1] = (int)numFrames_data;
      emxEnsureCapacity_real_T(a__1, i);
      a__1_data = a__1->data;
      magic_size = (int)magic_data * (int)numFrames_data;
      for (i = 0; i < magic_size; i++) {
        a__1_data[i] = frameTot_data[i];
      }
    } else {
      binary_expand_op(a__1, frameTot, (double *)&dacMin_data, &dacMin_size,
                       (double *)&magic_data, (double *)&numFrames_data);
      a__1_data = a__1->data;
    }
    /*  Process out the weird spike */
    i = (int)numFrames_data;
    for (b_i = 0; b_i < i; b_i++) {
      magic_size = a__1->size[0];
      dacMin_size = frameTot->size[0];
      frameTot->size[0] = a__1->size[0];
      emxEnsureCapacity_real_T(frameTot, dacMin_size);
      frameTot_data = frameTot->data;
      for (dacMin_size = 0; dacMin_size < magic_size; dacMin_size++) {
        frameTot_data[dacMin_size] =
            a__1_data[dacMin_size + a__1->size[0] * b_i];
      }
      if (maximum(frameTot) > 8191.0) {
        if ((unsigned int)b_i + 1U > 1U) {
          dacMin_size = frameTot->size[0];
          frameTot->size[0] = a__1->size[0];
          emxEnsureCapacity_real_T(frameTot, dacMin_size);
          frameTot_data = frameTot->data;
          for (dacMin_size = 0; dacMin_size < magic_size; dacMin_size++) {
            frameTot_data[dacMin_size] =
                a__1_data[dacMin_size + a__1->size[0] * (b_i - 1)];
          }
          for (dacMin_size = 0; dacMin_size < magic_size; dacMin_size++) {
            a__1_data[dacMin_size + a__1->size[0] * b_i] =
                frameTot_data[dacMin_size];
          }
        } else {
          dacMin_size = frameTot->size[0];
          frameTot->size[0] = a__1->size[0];
          emxEnsureCapacity_real_T(frameTot, dacMin_size);
          frameTot_data = frameTot->data;
          for (dacMin_size = 0; dacMin_size < magic_size; dacMin_size++) {
            frameTot_data[dacMin_size] = a__1_data[dacMin_size + a__1->size[0]];
          }
          for (dacMin_size = 0; dacMin_size < magic_size; dacMin_size++) {
            a__1_data[dacMin_size] = frameTot_data[dacMin_size];
          }
        }
      }
    }
    d_fread(fileid, c_fileid.data);
    /*  Estimated FPS (good to check against frameRate) */
    /*  TODO - insert comment here */
    magic_data = h_fread(fileid, frameTot);
    cfclose(fileid);
    if (magic_data != 0.0) {
      printf("FILE READ ERROR: %f data remains! Check that file format matches "
             "read code\n",
             magic_data);
      fflush(stdout);
    } else {
      /*  Unused but potentially useful radar parameters */
      NoveldaChipParams(chipSet_Value_data, chipSet_Value_size,
                        (double *)&pgen_data, pgen_size, &iterations_data,
                        &pps_data, &dacMin_data, &dacStep_data, &numFrames_data,
                        &magic_data, &fs_hz);
      /*  Baseband Conversion */
      i = framesBB->size[0] * framesBB->size[1];
      framesBB->size[0] = a__1->size[0];
      framesBB->size[1] = a__1->size[1];
      emxEnsureCapacity_creal_T(framesBB, i);
      framesBB_data = framesBB->data;
      magic_size = a__1->size[0] * a__1->size[1];
      for (i = 0; i < magic_size; i++) {
        framesBB_data[i].re = 0.0;
        framesBB_data[i].im = 0.0;
      }
      i = a__1->size[1];
      for (b_i = 0; b_i < i; b_i++) {
        magic_size = a__1->size[0];
        dacMin_size = frameTot->size[0];
        frameTot->size[0] = a__1->size[0];
        emxEnsureCapacity_real_T(frameTot, dacMin_size);
        frameTot_data = frameTot->data;
        for (dacMin_size = 0; dacMin_size < magic_size; dacMin_size++) {
          frameTot_data[dacMin_size] =
              a__1_data[dacMin_size + a__1->size[0] * b_i];
        }
        NoveldaDDC(frameTot, chipSet_Value_data, chipSet_Value_size,
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
  emxFree_real_T(&a__1);
  /*  [captureFT, tagFT] = ProcessFFT(framesBB, frameRate, tagHz) */
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
  emxFree_creal_T(&framesBB);
  /*  Find Tag FT */
  /*  [tagFT, tagIndex] = TagIndex(captureFT, frameRate, tagHz) */
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
  pps_data = tagHz / frameRate_data * (double)b_captureFT->size[1];
  pps_size = b_captureFT->size[0];
  i = captureFT->size[0];
  captureFT->size[0] = b_captureFT->size[0];
  emxEnsureCapacity_creal_T(captureFT, i);
  captureFT_data = captureFT->data;
  for (i = 0; i < pps_size; i++) {
    captureFT_data[i] =
        framesBB_data[i + b_captureFT->size[0] * ((int)pps_data - 1)];
  }
  emxInit_real_T(&tagFT, 1);
  b_abs(captureFT, tagFT);
  for (b_i = 0; b_i < 5; b_i++) {
    magic_size = (int)(pps_data - 2.0) + b_i;
    i = captureFT->size[0];
    captureFT->size[0] = pps_size;
    emxEnsureCapacity_creal_T(captureFT, i);
    captureFT_data = captureFT->data;
    for (i = 0; i < pps_size; i++) {
      captureFT_data[i] =
          framesBB_data[i + b_captureFT->size[0] * (magic_size - 1)];
    }
    b_abs(captureFT, frameTot);
    frameTot_data = frameTot->data;
    if (maximum(frameTot) > maximum(tagFT)) {
      magic_size = frameTot->size[0];
      i = tagFT->size[0];
      tagFT->size[0] = frameTot->size[0];
      emxEnsureCapacity_real_T(tagFT, i);
      a__1_data = tagFT->data;
      for (i = 0; i < magic_size; i++) {
        a__1_data[i] = frameTot_data[i];
      }
    }
  }
  smoothdata(tagFT);
  /*  [tagPeakBin] = TagLocateCorrelation(correlatedTagFT, tagFT) */
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
  findpeaks(tagFT, magic_data * 0.9, frameTot, correlatedTagFT);
  /*  Find the bin corresponding to the largest peak in the tag FT */
  emxInit_real_T(&a__2, 1);
  findpeaks(tagFT, magic_data * 0.9, a__2, frameTot);
  frameTot_data = frameTot->data;
  emxFree_real_T(&tagFT);
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
  *peakBin = frameTot_data[0];
  i = frameTot->size[0];
  for (b_i = 0; b_i < i; b_i++) {
    if (fabs(frameTot_data[b_i] - (double)magic_size) < *peakBin) {
      *peakBin = frameTot_data[b_i];
    }
  }
  /*  [tagFT, tagIndex] = TagIndex(captureFT, frameRate, tagHz) */
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
  i = captureFT->size[0];
  captureFT->size[0] = b_captureFT->size[0];
  emxEnsureCapacity_creal_T(captureFT, i);
  captureFT_data = captureFT->data;
  for (i = 0; i < pps_size; i++) {
    captureFT_data[i] =
        framesBB_data[i + b_captureFT->size[0] * ((int)pps_data - 1)];
  }
  b_abs(captureFT, a__2);
  magic_size = (int)pps_data;
  for (b_i = 0; b_i < 5; b_i++) {
    magic_data = (pps_data - 2.0) + (double)b_i;
    i = captureFT->size[0];
    captureFT->size[0] = pps_size;
    emxEnsureCapacity_creal_T(captureFT, i);
    captureFT_data = captureFT->data;
    for (i = 0; i < pps_size; i++) {
      captureFT_data[i] =
          framesBB_data[i + b_captureFT->size[0] * ((int)magic_data - 1)];
    }
    b_abs(captureFT, frameTot);
    frameTot_data = frameTot->data;
    if (maximum(frameTot) > maximum(a__2)) {
      magic_size = frameTot->size[0];
      i = a__2->size[0];
      a__2->size[0] = frameTot->size[0];
      emxEnsureCapacity_real_T(a__2, i);
      a__1_data = a__2->data;
      for (i = 0; i < magic_size; i++) {
        a__1_data[i] = frameTot_data[i];
      }
      magic_size = (int)magic_data;
    }
  }
  emxFree_creal_T(&captureFT);
  emxFree_real_T(&a__2);
  emxFree_real_T(&frameTot);
  /*  SNRdb = TagSNR(captureFT, freqIndex, peakBin) */
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
    a__1_data = y->data;
    for (i = 0; i <= pps_size; i++) {
      a__1_data[i] = magic_data + (double)i;
    }
  } else {
    eml_float_colon(magic_data, iterations_data, y);
  }
  b_round(y);
  a__1_data = y->data;
  emxInit_creal_T(&c_captureFT, 2);
  i = c_captureFT->size[0] * c_captureFT->size[1];
  c_captureFT->size[0] = 1;
  pps_size = y->size[1];
  c_captureFT->size[1] = y->size[1];
  emxEnsureCapacity_creal_T(c_captureFT, i);
  captureFT_data = c_captureFT->data;
  for (i = 0; i < pps_size; i++) {
    captureFT_data[i] = framesBB_data
        [((int)*peakBin + b_captureFT->size[0] * ((int)a__1_data[i] - 1)) - 1];
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
  printf("Peak Bin: %f\n", *peakBin);
  fflush(stdout);
  printf("SNRdB: %f\n", *SNRdB);
  fflush(stdout);
}

/*
 * File trailer for RunTagDataEmbedded.c
 *
 * [EOF]
 */
