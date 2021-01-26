/*
 * fuzz.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "fuzz".
 *
 * Model version              : 1.3
 * Simulink Coder version : 9.3 (R2020a) 18-Nov-2019
 * C source code generated on : Thu Nov 19 01:15:23 2020
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "fuzz.h"
#include "fuzz_private.h"

/* Block signals (default storage) */
B_fuzz_T fuzz_B;

/* Block states (default storage) */
DW_fuzz_T fuzz_DW;

/* External inputs (root inport signals with default storage) */
ExtU_fuzz_T fuzz_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_fuzz_T fuzz_Y;

/* Real-time model */
RT_MODEL_fuzz_T fuzz_M_;
RT_MODEL_fuzz_T *const fuzz_M = &fuzz_M_;

/* Forward declaration for local functions */
static real_T fuzz_trapmf(real_T x, const real_T params[4]);
static real_T fuzz_trimf(real_T x, const real_T params[3]);
static void fuzz_trapmf_k(const real_T x[101], const real_T params[4], real_T y
  [101]);
static void fuzz_createMamdaniOutputMFCache(const real_T outputSamplePoints[101],
  real_T outputMFCache[303]);
static real_T fuzz_evaluateAndMethod(const real_T x[2]);
static real_T fuzz_evaluateOrMethod(const real_T x[2]);
static void rate_scheduler(void);

/*
 *   This function updates active task flag for each subrate.
 * The function is called at model base rate, hence the
 * generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (fuzz_M->Timing.TaskCounters.TID[2])++;
  if ((fuzz_M->Timing.TaskCounters.TID[2]) > 4) {/* Sample time: [0.07142855s, 0.0s] */
    fuzz_M->Timing.TaskCounters.TID[2] = 0;
  }
}

/* Function for MATLAB Function: '<S2>/Evaluate Rule Antecedents' */
static real_T fuzz_trapmf(real_T x, const real_T params[4])
{
  real_T b_y1;
  real_T y2;
  b_y1 = 0.0;
  y2 = 0.0;
  if (x >= params[1]) {
    b_y1 = 1.0;
  }

  if (x < params[0]) {
    b_y1 = 0.0;
  }

  if ((params[0] <= x) && (x < params[1]) && (params[0] != params[1])) {
    b_y1 = 1.0 / (params[1] - params[0]) * (x - params[0]);
  }

  if (x <= params[2]) {
    y2 = 1.0;
  }

  if (x > params[3]) {
    y2 = 0.0;
  }

  if ((params[2] < x) && (x <= params[3]) && (params[2] != params[3])) {
    y2 = 1.0 / (params[3] - params[2]) * (params[3] - x);
  }

  return fmin(b_y1, y2);
}

/* Function for MATLAB Function: '<S2>/Evaluate Rule Antecedents' */
static real_T fuzz_trimf(real_T x, const real_T params[3])
{
  real_T y;
  y = 0.0;
  if ((params[0] < x) && (x < 0.0)) {
    y = 1.0 / (0.0 - params[0]) * (x - params[0]);
  }

  if ((0.0 < x) && (x < params[2])) {
    y = (params[2] - x) * (1.0 / params[2]);
  }

  if (x == 0.0) {
    y = 1.0;
  }

  return y;
}

/* Function for MATLAB Function: '<S2>/Evaluate Rule Consequents' */
static void fuzz_trapmf_k(const real_T x[101], const real_T params[4], real_T y
  [101])
{
  real_T a;
  real_T b;
  real_T c;
  real_T d;
  int32_T i;
  real_T b_y1;
  real_T y2;
  a = params[0];
  b = params[1];
  c = params[2];
  d = params[3];
  for (i = 0; i < 101; i++) {
    b_y1 = 0.0;
    y2 = 0.0;
    if (x[i] >= b) {
      b_y1 = 1.0;
    }

    if (x[i] < a) {
      b_y1 = 0.0;
    }

    if ((a <= x[i]) && (x[i] < b) && (a != b)) {
      b_y1 = (x[i] - a) * 0.025;
    }

    if (x[i] <= c) {
      y2 = 1.0;
    }

    if (x[i] > d) {
      y2 = 0.0;
    }

    if ((c < x[i]) && (x[i] <= d) && (c != d)) {
      y2 = (0.0 - x[i]) * 0.025;
    }

    y[i] = fmin(b_y1, y2);
  }
}

/* Function for MATLAB Function: '<S2>/Evaluate Rule Consequents' */
static void fuzz_createMamdaniOutputMFCache(const real_T outputSamplePoints[101],
  real_T outputMFCache[303])
{
  real_T tmp[101];
  int32_T i;
  int32_T outputMFCache_tmp;
  static const real_T b[4] = { -60.0, -60.0, -40.0, 0.0 };

  static const real_T c[4] = { 0.0, 40.0, 60.0, 60.0 };

  fuzz_trapmf_k(outputSamplePoints, b, tmp);
  for (i = 0; i < 101; i++) {
    outputMFCache[3 * i] = tmp[i];
    outputMFCache_tmp = 3 * i + 1;
    outputMFCache[outputMFCache_tmp] = 0.0;
    if ((-40.0 < outputSamplePoints[i]) && (outputSamplePoints[i] < 0.0)) {
      outputMFCache[outputMFCache_tmp] = (outputSamplePoints[i] - -40.0) * 0.025;
    }

    if ((0.0 < outputSamplePoints[i]) && (outputSamplePoints[i] < 40.0)) {
      outputMFCache[outputMFCache_tmp] = (40.0 - outputSamplePoints[i]) * 0.025;
    }

    if (outputSamplePoints[i] == 0.0) {
      outputMFCache[outputMFCache_tmp] = 1.0;
    }
  }

  fuzz_trapmf_k(outputSamplePoints, c, tmp);
  for (i = 0; i < 101; i++) {
    outputMFCache[3 * i + 2] = tmp[i];
  }
}

/* Function for MATLAB Function: '<S2>/Evaluate Rule Consequents' */
static real_T fuzz_evaluateAndMethod(const real_T x[2])
{
  real_T y;
  if ((x[0] > x[1]) || (rtIsNaN(x[0]) && (!rtIsNaN(x[1])))) {
    y = x[1];
  } else {
    y = x[0];
  }

  return y;
}

/* Function for MATLAB Function: '<S2>/Evaluate Rule Consequents' */
static real_T fuzz_evaluateOrMethod(const real_T x[2])
{
  real_T y;
  if ((x[0] < x[1]) || (rtIsNaN(x[0]) && (!rtIsNaN(x[1])))) {
    y = x[1];
  } else {
    y = x[0];
  }

  return y;
}

/* Model step function */
void fuzz_step(void)
{
  {
    real_T *lastU;
    real_T inputMFCache[6];
    int32_T ruleID;
    int32_T inputID;
    real_T outputMFCache[303];
    real_T area;
    real_T rtb_Derivative;
    real_T rtb_antecedentOutputs[6];
    real_T rtb_aggregatedOutputs[101];
    real_T tmp[3];
    real_T outputMFCache_0[2];
    real_T rtb_aggregatedOutputs_0[2];
    int8_T g;
    real_T rtb_antecedentOutputs_p;
    static const real_T c[4] = { -60.0, -60.0, -40.0, 0.0 };

    static const real_T d[4] = { 0.0, 40.0, 60.0, 60.0 };

    static const real_T e[4] = { -120.0, -120.0, -80.0, 0.0 };

    static const real_T f[4] = { 0.0, 80.0, 120.0, 120.0 };

    static const int8_T g_0[12] = { 1, 2, 3, 0, 0, 0, 0, 0, 0, 1, 2, 3 };

    static const real_T b[6] = { 0.7, 0.6, 0.7, 0.4, 0.15, 0.5 };

    static const int8_T b_0[6] = { 3, 2, 1, 1, 2, 3 };

    /* Gain: '<Root>/Gain1' incorporates:
     *  Inport: '<Root>/error'
     */
    fuzz_B.Gain1 = fuzz_P.Gain1_Gain * fuzz_U.error;

    /* Derivative: '<Root>/Derivative' incorporates:
     *  Inport: '<Root>/error'
     */
    rtb_Derivative = fuzz_M->Timing.t[0];
    if ((fuzz_DW.TimeStampA >= rtb_Derivative) && (fuzz_DW.TimeStampB >=
         rtb_Derivative)) {
      rtb_Derivative = 0.0;
    } else {
      area = fuzz_DW.TimeStampA;
      lastU = &fuzz_DW.LastUAtTimeA;
      if (fuzz_DW.TimeStampA < fuzz_DW.TimeStampB) {
        if (fuzz_DW.TimeStampB < rtb_Derivative) {
          area = fuzz_DW.TimeStampB;
          lastU = &fuzz_DW.LastUAtTimeB;
        }
      } else {
        if (fuzz_DW.TimeStampA >= rtb_Derivative) {
          area = fuzz_DW.TimeStampB;
          lastU = &fuzz_DW.LastUAtTimeB;
        }
      }

      rtb_Derivative = (fuzz_U.error - *lastU) / (rtb_Derivative - area);
    }

    /* End of Derivative: '<Root>/Derivative' */

    /* Gain: '<Root>/Gain' */
    fuzz_B.Gain = fuzz_P.Gain_Gain * rtb_Derivative;

    /* Outputs for Atomic SubSystem: '<S1>/Fuzzy Logic Controller' */
    /* MATLAB Function: '<S2>/Evaluate Rule Antecedents' incorporates:
     *  SignalConversion generated from: '<S4>/ SFunction '
     */
    rtb_Derivative = 0.0;
    inputMFCache[0] = fuzz_trapmf(fuzz_B.Gain1, c);
    tmp[0] = -40.0;
    tmp[1] = 0.0;
    tmp[2] = 40.0;
    inputMFCache[1] = fuzz_trimf(fuzz_B.Gain1, tmp);
    inputMFCache[2] = fuzz_trapmf(fuzz_B.Gain1, d);
    inputMFCache[3] = fuzz_trapmf(fuzz_B.Gain, e);
    tmp[0] = -80.0;
    tmp[1] = 0.0;
    tmp[2] = 80.0;
    inputMFCache[4] = fuzz_trimf(fuzz_B.Gain, tmp);
    inputMFCache[5] = fuzz_trapmf(fuzz_B.Gain, f);
    for (ruleID = 0; ruleID < 6; ruleID++) {
      area = 1.0;
      if (g_0[ruleID] != 0) {
        area = inputMFCache[g_0[ruleID] - 1];
        if (!(1.0 > area)) {
          area = 1.0;
        }
      }

      rtb_antecedentOutputs_p = area;
      g = g_0[ruleID + 6];
      if (g != 0) {
        area = inputMFCache[g + 2];
        if (!(rtb_antecedentOutputs_p > area)) {
          area = rtb_antecedentOutputs_p;
        }
      }

      rtb_antecedentOutputs_p = area * b[ruleID];
      rtb_Derivative += rtb_antecedentOutputs_p;
      rtb_antecedentOutputs[ruleID] = rtb_antecedentOutputs_p;
    }

    /* MATLAB Function: '<S2>/Evaluate Rule Consequents' incorporates:
     *  Constant: '<S2>/Output Sample Points'
     */
    memset(&rtb_aggregatedOutputs[0], 0, 101U * sizeof(real_T));
    fuzz_createMamdaniOutputMFCache(fuzz_P.OutputSamplePoints_Value,
      outputMFCache);
    for (ruleID = 0; ruleID < 6; ruleID++) {
      outputMFCache_0[1] = rtb_antecedentOutputs[ruleID];
      for (inputID = 0; inputID < 101; inputID++) {
        outputMFCache_0[0] = outputMFCache[(3 * inputID + b_0[ruleID]) - 1];
        rtb_aggregatedOutputs_0[0] = rtb_aggregatedOutputs[inputID];
        rtb_aggregatedOutputs_0[1] = fuzz_evaluateAndMethod(outputMFCache_0);
        rtb_aggregatedOutputs[inputID] = fuzz_evaluateOrMethod
          (rtb_aggregatedOutputs_0);
      }
    }

    /* End of MATLAB Function: '<S2>/Evaluate Rule Consequents' */

    /* MATLAB Function: '<S2>/Defuzzify Outputs' incorporates:
     *  Constant: '<S2>/Output Sample Points'
     *  MATLAB Function: '<S2>/Evaluate Rule Antecedents'
     */
    if (rtb_Derivative == 0.0) {
      /* Outport: '<Root>/u' */
      fuzz_Y.u = 0.0;
    } else {
      rtb_Derivative = 0.0;
      area = 0.0;
      for (ruleID = 0; ruleID < 101; ruleID++) {
        area += rtb_aggregatedOutputs[ruleID];
      }

      if (area == 0.0) {
        /* Outport: '<Root>/u' incorporates:
         *  Constant: '<S2>/Output Sample Points'
         */
        fuzz_Y.u = (fuzz_P.OutputSamplePoints_Value[0] +
                    fuzz_P.OutputSamplePoints_Value[100]) / 2.0;
      } else {
        for (ruleID = 0; ruleID < 101; ruleID++) {
          rtb_Derivative += fuzz_P.OutputSamplePoints_Value[ruleID] *
            rtb_aggregatedOutputs[ruleID];
        }

        /* Outport: '<Root>/u' incorporates:
         *  Constant: '<S2>/Output Sample Points'
         */
        fuzz_Y.u = 1.0 / area * rtb_Derivative;
      }
    }

    /* End of MATLAB Function: '<S2>/Defuzzify Outputs' */
    /* End of Outputs for SubSystem: '<S1>/Fuzzy Logic Controller' */
    if (fuzz_M->Timing.TaskCounters.TID[2] == 0) {
      /* ZeroOrderHold: '<S1>/Zero-Order Hold' */
      fuzz_B.ZeroOrderHold[0] = fuzz_B.Gain1;
      fuzz_B.ZeroOrderHold[1] = fuzz_B.Gain;
    }
  }

  /* Matfile logging */
  rt_UpdateTXYLogVars(fuzz_M->rtwLogInfo, (fuzz_M->Timing.t));

  {
    real_T *lastU;

    /* Update for Derivative: '<Root>/Derivative' incorporates:
     *  Inport: '<Root>/error'
     */
    if (fuzz_DW.TimeStampA == (rtInf)) {
      fuzz_DW.TimeStampA = fuzz_M->Timing.t[0];
      lastU = &fuzz_DW.LastUAtTimeA;
    } else if (fuzz_DW.TimeStampB == (rtInf)) {
      fuzz_DW.TimeStampB = fuzz_M->Timing.t[0];
      lastU = &fuzz_DW.LastUAtTimeB;
    } else if (fuzz_DW.TimeStampA < fuzz_DW.TimeStampB) {
      fuzz_DW.TimeStampA = fuzz_M->Timing.t[0];
      lastU = &fuzz_DW.LastUAtTimeA;
    } else {
      fuzz_DW.TimeStampB = fuzz_M->Timing.t[0];
      lastU = &fuzz_DW.LastUAtTimeB;
    }

    *lastU = fuzz_U.error;

    /* End of Update for Derivative: '<Root>/Derivative' */
    if (fuzz_M->Timing.TaskCounters.TID[2] == 0) {
    }
  }

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.0s, 0.0s] */
    if ((rtmGetTFinal(fuzz_M)!=-1) &&
        !((rtmGetTFinal(fuzz_M)-fuzz_M->Timing.t[0]) > fuzz_M->Timing.t[0] *
          (DBL_EPSILON))) {
      rtmSetErrorStatus(fuzz_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++fuzz_M->Timing.clockTick0)) {
    ++fuzz_M->Timing.clockTickH0;
  }

  fuzz_M->Timing.t[0] = fuzz_M->Timing.clockTick0 * fuzz_M->Timing.stepSize0 +
    fuzz_M->Timing.clockTickH0 * fuzz_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.01428571s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.01428571, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    fuzz_M->Timing.clockTick1++;
    if (!fuzz_M->Timing.clockTick1) {
      fuzz_M->Timing.clockTickH1++;
    }
  }

  rate_scheduler();
}

/* Model initialize function */
void fuzz_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)fuzz_M, 0,
                sizeof(RT_MODEL_fuzz_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&fuzz_M->solverInfo, &fuzz_M->Timing.simTimeStep);
    rtsiSetTPtr(&fuzz_M->solverInfo, &rtmGetTPtr(fuzz_M));
    rtsiSetStepSizePtr(&fuzz_M->solverInfo, &fuzz_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&fuzz_M->solverInfo, (&rtmGetErrorStatus(fuzz_M)));
    rtsiSetRTModelPtr(&fuzz_M->solverInfo, fuzz_M);
  }

  rtsiSetSimTimeStep(&fuzz_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&fuzz_M->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(fuzz_M, &fuzz_M->Timing.tArray[0]);
  rtmSetTFinal(fuzz_M, 9.999997);
  fuzz_M->Timing.stepSize0 = 0.01428571;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = NULL;
    fuzz_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(fuzz_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(fuzz_M->rtwLogInfo, (NULL));
    rtliSetLogT(fuzz_M->rtwLogInfo, "tout");
    rtliSetLogX(fuzz_M->rtwLogInfo, "");
    rtliSetLogXFinal(fuzz_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(fuzz_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(fuzz_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(fuzz_M->rtwLogInfo, 0);
    rtliSetLogDecimation(fuzz_M->rtwLogInfo, 1);
    rtliSetLogY(fuzz_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(fuzz_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(fuzz_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &fuzz_B), 0,
                sizeof(B_fuzz_T));

  /* states (dwork) */
  (void) memset((void *)&fuzz_DW, 0,
                sizeof(DW_fuzz_T));

  /* external inputs */
  fuzz_U.error = 0.0;

  /* external outputs */
  fuzz_Y.u = 0.0;

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(fuzz_M->rtwLogInfo, 0.0, rtmGetTFinal(fuzz_M),
    fuzz_M->Timing.stepSize0, (&rtmGetErrorStatus(fuzz_M)));

  /* InitializeConditions for Derivative: '<Root>/Derivative' */
  fuzz_DW.TimeStampA = (rtInf);
  fuzz_DW.TimeStampB = (rtInf);
}

/* Model terminate function */
void fuzz_terminate(void)
{
  /* (no terminate code required) */
}
