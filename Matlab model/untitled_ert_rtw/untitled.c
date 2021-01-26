/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: untitled.c
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

#include "untitled.h"
#include "untitled_private.h"

/* Block signals (default storage) */
B_untitled_T untitled_B;

/* Block states (default storage) */
DW_untitled_T untitled_DW;

/* Real-time model */
RT_MODEL_untitled_T untitled_M_;
RT_MODEL_untitled_T *const untitled_M = &untitled_M_;

/* Model step function */
void untitled_step(void)
{
  {
    real_T *lastU;
    real_T lastTime;
    real_T rtb_Derivative;

    /* Derivative: '<S1>/Derivative' */
    rtb_Derivative = untitled_M->Timing.t[0];
    if ((untitled_DW.TimeStampA >= rtb_Derivative) && (untitled_DW.TimeStampB >=
         rtb_Derivative)) {
      rtb_Derivative = 0.0;
    } else {
      lastTime = untitled_DW.TimeStampA;
      lastU = &untitled_DW.LastUAtTimeA;
      if (untitled_DW.TimeStampA < untitled_DW.TimeStampB) {
        if (untitled_DW.TimeStampB < rtb_Derivative) {
          lastTime = untitled_DW.TimeStampB;
          lastU = &untitled_DW.LastUAtTimeB;
        }
      } else {
        if (untitled_DW.TimeStampA >= rtb_Derivative) {
          lastTime = untitled_DW.TimeStampB;
          lastU = &untitled_DW.LastUAtTimeB;
        }
      }

      rtb_Derivative = (0.0 - *lastU) / (rtb_Derivative - lastTime);
    }

    /* End of Derivative: '<S1>/Derivative' */

    /* ZeroOrderHold: '<S2>/Zero-Order Hold' incorporates:
     *  Gain: '<S1>/Gain'
     *  Gain: '<S1>/Gain1'
     */
    untitled_B.ZeroOrderHold[0] = untitled_P.Gain1_Gain * 0.0;
    untitled_B.ZeroOrderHold[1] = untitled_P.Gain_Gain * rtb_Derivative;
  }

  {
    real_T *lastU;

    /* Update for Derivative: '<S1>/Derivative' */
    if (untitled_DW.TimeStampA == (rtInf)) {
      untitled_DW.TimeStampA = untitled_M->Timing.t[0];
      lastU = &untitled_DW.LastUAtTimeA;
    } else if (untitled_DW.TimeStampB == (rtInf)) {
      untitled_DW.TimeStampB = untitled_M->Timing.t[0];
      lastU = &untitled_DW.LastUAtTimeB;
    } else if (untitled_DW.TimeStampA < untitled_DW.TimeStampB) {
      untitled_DW.TimeStampA = untitled_M->Timing.t[0];
      lastU = &untitled_DW.LastUAtTimeA;
    } else {
      untitled_DW.TimeStampB = untitled_M->Timing.t[0];
      lastU = &untitled_DW.LastUAtTimeB;
    }

    *lastU = 0.0;

    /* End of Update for Derivative: '<S1>/Derivative' */
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  untitled_M->Timing.t[0] =
    ((time_T)(++untitled_M->Timing.clockTick0)) * untitled_M->Timing.stepSize0;

  {
    /* Update absolute timer for sample time: [0.07142855s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.07142855, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     */
    untitled_M->Timing.clockTick1++;
  }
}

/* Model initialize function */
void untitled_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&untitled_M->solverInfo,
                          &untitled_M->Timing.simTimeStep);
    rtsiSetTPtr(&untitled_M->solverInfo, &rtmGetTPtr(untitled_M));
    rtsiSetStepSizePtr(&untitled_M->solverInfo, &untitled_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&untitled_M->solverInfo, (&rtmGetErrorStatus
      (untitled_M)));
    rtsiSetRTModelPtr(&untitled_M->solverInfo, untitled_M);
  }

  rtsiSetSimTimeStep(&untitled_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&untitled_M->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(untitled_M, &untitled_M->Timing.tArray[0]);
  untitled_M->Timing.stepSize0 = 0.07142855;

  /* InitializeConditions for Derivative: '<S1>/Derivative' */
  untitled_DW.TimeStampA = (rtInf);
  untitled_DW.TimeStampB = (rtInf);
}

/* Model terminate function */
void untitled_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
