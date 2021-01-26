/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: untitled.h
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

#ifndef RTW_HEADER_untitled_h_
#define RTW_HEADER_untitled_h_
#include <stddef.h>
#ifndef untitled_COMMON_INCLUDES_
# define untitled_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* untitled_COMMON_INCLUDES_ */

#include "untitled_types.h"
#include "MW_target_hardware_resources.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T dv[101];
  real_T ZeroOrderHold[2];             /* '<S2>/Zero-Order Hold' */
} B_untitled_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T TimeStampA;                   /* '<S1>/Derivative' */
  real_T LastUAtTimeA;                 /* '<S1>/Derivative' */
  real_T TimeStampB;                   /* '<S1>/Derivative' */
  real_T LastUAtTimeB;                 /* '<S1>/Derivative' */
} DW_untitled_T;

/* Parameters (default storage) */
struct P_untitled_T_ {
  real_T OutputSamplePoints_Value[101];/* Expression: fis.outputSamplePoints
                                        * Referenced by: '<S3>/Output Sample Points'
                                        */
  real_T Gain1_Gain;                   /* Expression: 1
                                        * Referenced by: '<S1>/Gain1'
                                        */
  real_T Gain_Gain;                    /* Expression: 1
                                        * Referenced by: '<S1>/Gain'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_untitled_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
extern P_untitled_T untitled_P;

/* Block signals (default storage) */
extern B_untitled_T untitled_B;

/* Block states (default storage) */
extern DW_untitled_T untitled_DW;

/* Model entry point functions */
extern void untitled_initialize(void);
extern void untitled_step(void);
extern void untitled_terminate(void);

/* Real-time Model object */
extern RT_MODEL_untitled_T *const untitled_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Rate Transition3' : Unused code path elimination
 * Block '<S1>/Saturation' : Unused code path elimination
 * Block '<S1>/Zero-Order Hold2' : Unused code path elimination
 * Block '<S1>/Rate Transition5' : Eliminated since input and output rates are identical
 */

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
 * '<Root>' : 'untitled'
 * '<S1>'   : 'untitled/Fuzzy Controller'
 * '<S2>'   : 'untitled/Fuzzy Controller/Fuzzy Logic  Controller  with Ruleviewer'
 * '<S3>'   : 'untitled/Fuzzy Controller/Fuzzy Logic  Controller  with Ruleviewer/Fuzzy Logic Controller'
 * '<S4>'   : 'untitled/Fuzzy Controller/Fuzzy Logic  Controller  with Ruleviewer/Fuzzy Logic Controller/Defuzzify Outputs'
 * '<S5>'   : 'untitled/Fuzzy Controller/Fuzzy Logic  Controller  with Ruleviewer/Fuzzy Logic Controller/Evaluate Rule Antecedents'
 * '<S6>'   : 'untitled/Fuzzy Controller/Fuzzy Logic  Controller  with Ruleviewer/Fuzzy Logic Controller/Evaluate Rule Consequents'
 */
#endif                                 /* RTW_HEADER_untitled_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
