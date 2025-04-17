/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_RunTagDataEmbedded_info.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

/* Include Files */
#include "_coder_RunTagDataEmbedded_info.h"
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
      "789ced585d6fd25018eecc34d344c544f717cca22143d816e7152b5fcb3664500526460e"
      "ed01baf503fa011dfe00bdd3c41b6f8c5ef91ffc095ef903fc196a62"
      "6234969533da2ecd6129391b5ddf9bc3db879ee7e17de9735ea0e63677e6288aba415951"
      "5fb6d6eba33c325a2f51ce70e373a3f5aa2b4771999a77dc87f037a3",
      "9595250d1a9a95484084c77772b2c84b40d298c30ea414a8ca420f724748931720c38bb0"
      "644ff2c34cccd8a0e364080d5fd36dc81e94749152daea58a1604f8e"
      "ebf1dee3f3ce4f588fb8473d222efc59fa39bd5edb1740a3d6071c506acbb1179cdaa989"
      "401b5e2bea12035a29a081b4d8801c07b9a8e8d0d9f1a9d3dd17b74e",
      "8417149985aa9a51cc16a936feba4ffe79573ee6b710b60d14cac6f7d6275fd493cf8963"
      "fae228c7b025b87adc9c509f7b1dbf7fc1bafee9fb11448aef91fafb"
      "0f493e1467c56778ec37e9f76bd1832fe2c2e5fc96b45ac8367b7aa6246d49319e8e55d2"
      "a9b18e028607a783f2c849ed7fde9f53bf7dbe85d187f0bcdc830207",
      "e836df290053053a6ba6a5e30e4607c25999834a94370f5b45024254d5145e6a51d33bef"
      "1e6074201cd3af13e51a1d77c4fce0eb07b2fefaf7ddcb6f24f95004"
      "dd5f9fc4ab86dee7936b7b95c3b826f03a9fcced6f84fe4aca5ffdcea5d730fa103e328c"
      "548ab6f2f3e6ab7efb740fa303e193f9aa5926d7ef87ba07ffacceab",
      "c6a0dd27c98722e87eba19db1d2c6b46b7f9505ddb6854aa46934976433f25f69c86f36a"
      "38af3adf1fceab24f8c279753afb5f94ff53279d5bd1e09c6128077f"
      "dd27ff154f7e0be164bd21c07377ee8dcb41783efdf791ac7f7e892ce548f2a108ba7f32"
      "d96437b551619bbb8936cbec4a0cdf7b5ca183e39fb3fa9c4eaa6fc1",
      "958ff55988e9da9b12078d59f5c9254f3e278ea93f2a031a3303eb93edcff7439fa4a6ef"
      "933bad72bcb1974c0f9e1a6a2b2bb41afd7c4f0de74ce273a6df7edf"
      "c6e844b8a9625b668106695951a000345e96ec3aea3e759cd647fdf667c593cf89e37df4"
      "44598e3a14543f6dbdfab148920f45d0fdb450665720cd26b2abd9ad",
      "523e17d3f5d2413313fa29693fadfbd419ce9f5684f367387fdaf70be7cfd3ed7f51fc12"
      "e75f267b295f3cbb39f3b54fbebb9e7c4e1cef976619ccda07d5277f"
      "fd4c3749f2a108ba4fae6d6b8942d51894e472b598d9c9958b87896e6ef67df23f3cab70"
      "bf",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 12408U, &nameCaptureInfo);
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
                emlrtMxCreateString("RunTagDataEmbedded"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(3.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "ResolvedFilePath",
                emlrtMxCreateString(
                    "C:\\jlab\\wadar\\01_dsp\\matlab\\RunTagDataEmbedded.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739704.40603009262));
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
                emlrtMxCreateString("XsEKEpoorFvuSJog3k93eC"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/*
 * File trailer for _coder_RunTagDataEmbedded_info.c
 *
 * [EOF]
 */
