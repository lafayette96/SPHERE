/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: untitled_types.h
 *
 * Code generated for Simulink model 'untitled'.
 *
 * Model version                  : 1.0
 * Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
 * C/C++ source code generated on : Sat Nov 21 16:15:54 2020
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_untitled_types_h_
#define RTW_HEADER_untitled_types_h_
#include "rtwtypes.h"

/* Model Code Variants */
#ifndef DEFINED_TYPEDEF_FOR_struct_7qvRRPGYj24v4pTFv1qMpE_
#define DEFINED_TYPEDEF_FOR_struct_7qvRRPGYj24v4pTFv1qMpE_

typedef struct {
  uint8_T SimulinkDiagnostic;
  uint8_T Model[66];
  uint8_T Block[89];
  uint8_T OutOfRangeInputValue;
  uint8_T NoRuleFired;
  uint8_T EmptyOutputFuzzySet;
} struct_7qvRRPGYj24v4pTFv1qMpE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_nDiNttezQ8pHMZv76leKsH_
#define DEFINED_TYPEDEF_FOR_struct_nDiNttezQ8pHMZv76leKsH_

typedef struct {
  uint8_T type[6];
  int32_T origTypeLength;
  real_T params[4];
  int32_T origParamLength;
} struct_nDiNttezQ8pHMZv76leKsH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_6VzYiVW2hAWQCI9jAYOVeF_
#define DEFINED_TYPEDEF_FOR_struct_6VzYiVW2hAWQCI9jAYOVeF_

typedef struct {
  struct_nDiNttezQ8pHMZv76leKsH mf[3];
  int32_T origNumMF;
} struct_6VzYiVW2hAWQCI9jAYOVeF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_qNRhZavWHK4UBBn9W24nJF_
#define DEFINED_TYPEDEF_FOR_struct_qNRhZavWHK4UBBn9W24nJF_

typedef struct {
  uint8_T type[7];
  uint8_T andMethod[3];
  uint8_T orMethod[3];
  uint8_T defuzzMethod[8];
  uint8_T impMethod[3];
  uint8_T aggMethod[3];
  real_T inputRange[4];
  real_T outputRange[2];
  struct_6VzYiVW2hAWQCI9jAYOVeF inputMF[2];
  struct_6VzYiVW2hAWQCI9jAYOVeF outputMF;
  real_T antecedent[12];
  real_T consequent[6];
  real_T connection[6];
  real_T weight[6];
  int32_T numSamples;
  int32_T numInputs;
  int32_T numOutputs;
  int32_T numRules;
  int32_T numInputMFs[2];
  int32_T numCumInputMFs[2];
  int32_T numOutputMFs;
  int32_T numCumOutputMFs;
  real_T outputSamplePoints[101];
  int32_T orrSize[2];
  int32_T aggSize[2];
  int32_T irrSize[2];
  int32_T rfsSize[2];
  int32_T sumSize[2];
  int32_T inputFuzzySetType;
} struct_qNRhZavWHK4UBBn9W24nJF;

#endif

/* Parameters (default storage) */
typedef struct P_untitled_T_ P_untitled_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_untitled_T RT_MODEL_untitled_T;

#endif                                 /* RTW_HEADER_untitled_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
