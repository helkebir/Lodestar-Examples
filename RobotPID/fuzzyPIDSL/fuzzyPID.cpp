/*
 * fuzzyPID.cpp
 *
 * Classroom License -- for classroom instructional use only.  Not for
 * government, commercial, academic research, or other organizational use.
 *
 * Code generation for model "fuzzyPID".
 *
 * Model version              : 1.1
 * Simulink Coder version : 9.6 (R2021b) 14-May-2021
 * C++ source code generated on : Sun Feb 27 20:20:13 2022
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "fuzzyPID.h"
#include "fuzzyPID_private.h"

/* Model step function */
void fuzzyPIDModelClass::step()
{
  real_T tmp;

  /* Delay: '<Root>/Delay One Step' */
  fuzzyPID_B.DelayOneStep = fuzzyPID_DW.DelayOneStep_DSTATE;

  /* Sum: '<Root>/Sum' incorporates:
   *  Inport: '<Root>/Input'
   *  Inport: '<Root>/Input1'
   */
  fuzzyPID_B.Sum = fuzzyPID_U.Input - fuzzyPID_U.Input1;

  /* Sum: '<Root>/Sum1' */
  fuzzyPID_B.Sum1 = fuzzyPID_B.DelayOneStep + fuzzyPID_B.Sum;

  /* Gain: '<Root>/Gain5' */
  tmp = 1.0 / fuzzyPID_P.T;

  /* Gain: '<Root>/Gain5' */
  fuzzyPID_B.Gain5 = tmp * fuzzyPID_B.Sum1;

  /* Gain: '<Root>/Gain6' */
  fuzzyPID_B.Gain6 = fuzzyPID_P.Kp * fuzzyPID_B.Gain5;

  /* Delay: '<Root>/Delay One Step1' */
  fuzzyPID_B.DelayOneStep1 = fuzzyPID_DW.DelayOneStep1_DSTATE;

  /* Sum: '<Root>/Sum2' */
  fuzzyPID_B.Sum2 = fuzzyPID_B.Sum - fuzzyPID_B.DelayOneStep1;

  /* Gain: '<Root>/Gain' */
  tmp = 1.0 / fuzzyPID_P.T;

  /* Gain: '<Root>/Gain' */
  fuzzyPID_B.Gain = tmp * fuzzyPID_B.Sum2;

  /* Gain: '<Root>/Gain1' */
  fuzzyPID_B.Gain1 = fuzzyPID_P.Kd * fuzzyPID_B.Gain;

  /* Sum: '<Root>/Sum5' */
  fuzzyPID_B.Sum5 = fuzzyPID_B.Gain6 + fuzzyPID_B.Gain1;

  /* Gain: '<Root>/Gain7' */
  fuzzyPID_B.Gain7 = fuzzyPID_P.KuPD * fuzzyPID_B.Sum5;

  /* Delay: '<Root>/Delay One Step4' */
  fuzzyPID_B.DelayOneStep4 = fuzzyPID_DW.DelayOneStep4_DSTATE;

  /* Sum: '<Root>/Sum6' */
  fuzzyPID_B.Sum6 = fuzzyPID_B.Gain7 - fuzzyPID_B.DelayOneStep4;

  /* Gain: '<Root>/Gain2' */
  fuzzyPID_B.Gain2 = fuzzyPID_P.K * fuzzyPID_B.Gain;

  /* Delay: '<Root>/Delay One Step2' */
  fuzzyPID_B.DelayOneStep2 = fuzzyPID_DW.DelayOneStep2_DSTATE;

  /* Gain: '<Root>/Gain3' */
  fuzzyPID_B.Gain3 = fuzzyPID_P.Ki * fuzzyPID_B.DelayOneStep2;

  /* Sum: '<Root>/Sum3' */
  fuzzyPID_B.Sum3 = fuzzyPID_B.Gain2 + fuzzyPID_B.Gain3;

  /* Gain: '<Root>/Gain4' */
  fuzzyPID_B.Gain4 = fuzzyPID_P.KuI * fuzzyPID_B.Sum3;

  /* Delay: '<Root>/Delay One Step3' */
  fuzzyPID_B.DelayOneStep3 = fuzzyPID_DW.DelayOneStep3_DSTATE;

  /* Sum: '<Root>/Sum4' */
  fuzzyPID_B.Sum4 = fuzzyPID_B.Gain4 + fuzzyPID_B.DelayOneStep3;

  /* Outport: '<Root>/Output' incorporates:
   *  Sum: '<Root>/Sum7'
   */
  fuzzyPID_Y.Output = fuzzyPID_B.Sum6 + fuzzyPID_B.Sum4;

  /* Update for Delay: '<Root>/Delay One Step' */
  fuzzyPID_DW.DelayOneStep_DSTATE = fuzzyPID_B.Sum;

  /* Update for Delay: '<Root>/Delay One Step1' */
  fuzzyPID_DW.DelayOneStep1_DSTATE = fuzzyPID_B.Sum;

  /* Update for Delay: '<Root>/Delay One Step4' */
  fuzzyPID_DW.DelayOneStep4_DSTATE = fuzzyPID_B.Sum6;

  /* Update for Delay: '<Root>/Delay One Step2' */
  fuzzyPID_DW.DelayOneStep2_DSTATE = fuzzyPID_B.Sum;

  /* Update for Delay: '<Root>/Delay One Step3' */
  fuzzyPID_DW.DelayOneStep3_DSTATE = fuzzyPID_B.Sum4;

  /* Matfile logging */
//  rt_UpdateTXYLogVars((&fuzzyPID_M)->rtwLogInfo, (&(&fuzzyPID_M)
//    ->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.2s, 0.0s] */
    if ((rtmGetTFinal((&fuzzyPID_M))!=-1) &&
        !((rtmGetTFinal((&fuzzyPID_M))-(&fuzzyPID_M)->Timing.taskTime0) >
          (&fuzzyPID_M)->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus((&fuzzyPID_M), "Simulation finished");
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
  if (!(++(&fuzzyPID_M)->Timing.clockTick0)) {
    ++(&fuzzyPID_M)->Timing.clockTickH0;
  }

  (&fuzzyPID_M)->Timing.taskTime0 = (&fuzzyPID_M)->Timing.clockTick0 *
    (&fuzzyPID_M)->Timing.stepSize0 + (&fuzzyPID_M)->Timing.clockTickH0 *
    (&fuzzyPID_M)->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void fuzzyPIDModelClass::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));
  rtmSetTFinal((&fuzzyPID_M), 10.0);
  (&fuzzyPID_M)->Timing.stepSize0 = 0.2;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (nullptr);
    (&fuzzyPID_M)->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo((&fuzzyPID_M)->rtwLogInfo, (nullptr));
    rtliSetLogXSignalPtrs((&fuzzyPID_M)->rtwLogInfo, (nullptr));
    rtliSetLogT((&fuzzyPID_M)->rtwLogInfo, "tout");
    rtliSetLogX((&fuzzyPID_M)->rtwLogInfo, "");
    rtliSetLogXFinal((&fuzzyPID_M)->rtwLogInfo, "");
    rtliSetLogVarNameModifier((&fuzzyPID_M)->rtwLogInfo, "rt_");
    rtliSetLogFormat((&fuzzyPID_M)->rtwLogInfo, 4);
    rtliSetLogMaxRows((&fuzzyPID_M)->rtwLogInfo, 0);
    rtliSetLogDecimation((&fuzzyPID_M)->rtwLogInfo, 1);
    rtliSetLogY((&fuzzyPID_M)->rtwLogInfo, "");
    rtliSetLogYSignalInfo((&fuzzyPID_M)->rtwLogInfo, (nullptr));
    rtliSetLogYSignalPtrs((&fuzzyPID_M)->rtwLogInfo, (nullptr));
  }

  /* Matfile logging */
//  rt_StartDataLoggingWithStartTime((&fuzzyPID_M)->rtwLogInfo, 0.0, rtmGetTFinal
//    ((&fuzzyPID_M)), (&fuzzyPID_M)->Timing.stepSize0, (&rtmGetErrorStatus
//    ((&fuzzyPID_M))));

  /* InitializeConditions for Delay: '<Root>/Delay One Step' */
  fuzzyPID_DW.DelayOneStep_DSTATE = fuzzyPID_P.DelayOneStep_InitialCondition;

  /* InitializeConditions for Delay: '<Root>/Delay One Step1' */
  fuzzyPID_DW.DelayOneStep1_DSTATE = fuzzyPID_P.DelayOneStep1_InitialCondition;

  /* InitializeConditions for Delay: '<Root>/Delay One Step4' */
  fuzzyPID_DW.DelayOneStep4_DSTATE = fuzzyPID_P.DelayOneStep4_InitialCondition;

  /* InitializeConditions for Delay: '<Root>/Delay One Step2' */
  fuzzyPID_DW.DelayOneStep2_DSTATE = fuzzyPID_P.DelayOneStep2_InitialCondition;

  /* InitializeConditions for Delay: '<Root>/Delay One Step3' */
  fuzzyPID_DW.DelayOneStep3_DSTATE = fuzzyPID_P.DelayOneStep3_InitialCondition;
}

/* Model terminate function */
void fuzzyPIDModelClass::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
fuzzyPIDModelClass::fuzzyPIDModelClass() :
  fuzzyPID_U(),
  fuzzyPID_Y(),
  fuzzyPID_B(),
  fuzzyPID_DW(),
  fuzzyPID_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
fuzzyPIDModelClass::~fuzzyPIDModelClass()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_fuzzyPID_T * fuzzyPIDModelClass::getRTM()
{
  return (&fuzzyPID_M);
}
