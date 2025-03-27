/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_run_tag_test_chipotle_info.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 27-Mar-2025 00:17:05
 */

/* Include Files */
#include "_coder_run_tag_test_chipotle_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[8] = {
      "789ced59cb6ed3401475a0bc054d176587c412688992b64a29ac1ca76d4a9bbedc88b80d"
      "4a1d7b9c381d3fea479a941f4042a24880c406566cd8f11bac90d8b2"
      "e01b9090582111634f121b598ee468da58b99bc9cdb17d4eee78ce5c3b446c251f2308e2"
      "0661c7ed297bbceee471673c47b8c38bc79cf1b22747718118739d87",
      "f01367e414d9004dc34e6456029d33794512655636765a2a2034a02bb001f87f882042b0"
      "234a80ee4dd6ad4c5aea813a8905599fa91ae00e685322b49ade5508"
      "7b934e3ddef9fcdeb13eeb91f6a947dc83ef2d3ea51e96ea90ad948e589ed54ac95499d7"
      "d592c41ad6779a29970db65a36806e94b99aa82a060409a9ab530da9",
      "f35a804e84ab9ac29505ad3d4176b510ff7e48fe314fdee5b711aec66abd7caf42f24dfb"
      "f2b9f18079e92987351bc1f518ef539f77ec1e6fafb0930fdf6338f9"
      "5e70bf1671f2a1382dbea6cff5fabdbf6efaf0c53df8528322abd40625d2204fca0d9d51"
      "e6b699a5ae8ecd009e201d844f8eebfa677d9d869de789007d085f57",
      "1a00f22cd5b6ee4db6ad03ed3583d23119a003e19cc2032d21b6375b4d666142373451ae"
      "1283dbef660274203c60befe2b973d6bf8fce0cb7bbcfefae7f5b3af"
      "38f95044dd5f0bb34cd33c12c9f9dd626bd680a22992b97a66e4afb8fc356c5f7a35401f"
      "c21dc3c866293b3f6bbe1a769ea6027420bc3f5f6d97c931545cebff",
      "25e67ef5cd934a01271f8aa8fba9bc5a5d80e9546e77c65c5ec817299aaf1d0bb9919fe2"
      "5aa7a37e75d4afba8f1ff5ab38f846fdea60ae3f2cef53f743eaf47b"
      "0f1e7710bb61168c41f15df4e5b3115e312b100c6e9fbbebcbe7c6fb7a6e100c6497d8d6"
      "b78ab91ffdf4e31b89930f45d4fd5227d55ceb30b93c671e2a857ae6",
      "182eaf6c3523e497677d9d867dbebf12a00fe196598b320fd07d35acbe79cf97cf8d07cc"
      "47a71cb8fb4cdcbef9fbbccee3e4431175dfcc92bba698df6636d26b"
      "c2ce833cdf6a302bb5c5e8f8e6b0f49961fd73dc937b7522dc12c1299a06206b888a3cb4"
      "fe99f4e573e37df8674f391212aef53e81d93fa7df3e9ac4c98722ea",
      "feb996528d7a436fd0cc3cb75594d364f2f1cc6c84feb71f16ffdc0fa9f392af4e1bb1c8"
      "75593b35bf7c1e92ef8e2f9f1befc32fdb65c0e793b730fbe4cffb1f"
      "3fe3e44311759f2c1417320730935acdae415aa8d0cc215c2d44c027ff021225341e",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 11064U, &nameCaptureInfo);
  return nameCaptureInfo;
}

/*
 * Arguments    : void
 * Return Type  : mxArray *
 */
mxArray *emlrtMexFcnProperties(void)
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[9] = {"Version",
                                    "ResolvedFunctions",
                                    "Checksum",
                                    "EntryPoints",
                                    "CoverageInfo",
                                    "IsPolymorphic",
                                    "PropertyList",
                                    "UUID",
                                    "ClassEntryPointIsHandle"};
  const char_T *epFieldName[8] = {
      "QualifiedName",    "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "ResolvedFilePath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 3);
  emlrtSetField(xEntryPoints, 0, "QualifiedName",
                emlrtMxCreateString("run_tag_test_chipotle"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(3.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "ResolvedFilePath",
      emlrtMxCreateString(
          "C:\\jlab\\wadar\\01_dsp\\matlab\\run_tag_test_chipotle.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739702.53746527783));
  emlrtSetField(xEntryPoints, 0, "Constructor",
                emlrtMxCreateLogicalScalar(false));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("24.2.0.2712019 (R2024b)"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("4WoUKMhGaUiWqfOVlG2edC"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/*
 * File trailer for _coder_run_tag_test_chipotle_info.c
 *
 * [EOF]
 */
