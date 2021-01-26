/*
 * fuzz.h
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

#ifndef RTW_HEADER_fuzz_h_
#define RTW_HEADER_fuzz_h_
#include <math.h>
#include <string.h>
#include <float.h>
#include <stddef.h>
#ifndef fuzz_COMMON_INCLUDES_
# define fuzz_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* fuzz_COMMON_INCLUDES_ */

#include "fuzz_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
# define rtmGetRTWLogInfo(rtm)         ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Gain1;                        /* '<Root>/Gain1' */
  real_T Gain;                         /* '<Root>/Gain' */
  real_T ZeroOrderHold[2];             /* '<S1>/Zero-Order Hold' */
} B_fuzz_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T TimeStampA;                   /* '<Root>/Derivative' */
  real_T LastUAtTimeA;                 /* '<Root>/Derivative' */
  real_T TimeStampB;                   /* '<Root>/Derivative' */
  real_T LastUAtTimeB;                 /* '<Root>/Derivative' */
} DW_fuzz_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T error;                        /* '<Root>/error' */
} ExtU_fuzz_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T u;                            /* '<Root>/u' */
} ExtY_fuzz_T;

/* Parameters (default storage) */
struct P_fuzz_T_ {
  real_T OutputSamplePoints_Value[101];/* Expression: fis.outputSamplePoints
                                        * Referenced by: '<S2>/Output Sample Points'
                                        */
  real_T Gain1_Gain;                   /* Expression: 1
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T Gain_Gain;                    /* Expression: 1
                                        * Referenced by: '<Root>/Gain'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_fuzz_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    struct {
      uint8_T TID[3];
    } TaskCounters;

    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[3];
  } Timing;
};

/* Block parameters (default storage) */
extern P_fuzz_T fuzz_P;

/* Block signals (default storage) */
extern B_fuzz_T fuzz_B;

/* Block states (default storage) */
extern DW_fuzz_T fuzz_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_fuzz_T fuzz_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_fuzz_T fuzz_Y;

/* Model entry point functions */
extern void fuzz_initialize(void);
extern void fuzz_step(void);
extern void fuzz_terminate(void);

/* Real-time Model object */
extern RT_MODEL_fuzz_T *const fuzz_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'fuzz'
 * '<S1>'   : 'fuzz/Fuzzy Logic  Controller  with Ruleviewer'
 * '<S2>'   : 'fuzz/Fuzzy Logic  Controller  with Ruleviewer/Fuzzy Logic Controller'
 * '<S3>'   : 'fuzz/Fuzzy Logic  Controller  with Ruleviewer/Fuzzy Logic Controller/Defuzzify Outputs'
 * '<S4>'   : 'fuzz/Fuzzy Logic  Controller  with Ruleviewer/Fuzzy Logic Controller/Evaluate Rule Antecedents'
 * '<S5>'   : 'fuzz/Fuzzy Logic  Controller  with Ruleviewer/Fuzzy Logic Controller/Evaluate Rule Consequents'
 */
#endif                                 /* RTW_HEADER_fuzz_h_ */
