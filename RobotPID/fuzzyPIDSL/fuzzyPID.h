/*
 * fuzzyPID.h
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

#ifndef RTW_HEADER_fuzzyPID_h_
#define RTW_HEADER_fuzzyPID_h_
#include <cfloat>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "fuzzyPID_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

/* Block signals (default storage) */
struct B_fuzzyPID_T {
  real_T DelayOneStep;                 /* '<Root>/Delay One Step' */
  real_T Sum;                          /* '<Root>/Sum' */
  real_T Sum1;                         /* '<Root>/Sum1' */
  real_T Gain5;                        /* '<Root>/Gain5' */
  real_T Gain6;                        /* '<Root>/Gain6' */
  real_T DelayOneStep1;                /* '<Root>/Delay One Step1' */
  real_T Sum2;                         /* '<Root>/Sum2' */
  real_T Gain;                         /* '<Root>/Gain' */
  real_T Gain1;                        /* '<Root>/Gain1' */
  real_T Sum5;                         /* '<Root>/Sum5' */
  real_T Gain7;                        /* '<Root>/Gain7' */
  real_T DelayOneStep4;                /* '<Root>/Delay One Step4' */
  real_T Sum6;                         /* '<Root>/Sum6' */
  real_T Gain2;                        /* '<Root>/Gain2' */
  real_T DelayOneStep2;                /* '<Root>/Delay One Step2' */
  real_T Gain3;                        /* '<Root>/Gain3' */
  real_T Sum3;                         /* '<Root>/Sum3' */
  real_T Gain4;                        /* '<Root>/Gain4' */
  real_T DelayOneStep3;                /* '<Root>/Delay One Step3' */
  real_T Sum4;                         /* '<Root>/Sum4' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_fuzzyPID_T {
  real_T DelayOneStep_DSTATE;          /* '<Root>/Delay One Step' */
  real_T DelayOneStep1_DSTATE;         /* '<Root>/Delay One Step1' */
  real_T DelayOneStep4_DSTATE;         /* '<Root>/Delay One Step4' */
  real_T DelayOneStep2_DSTATE;         /* '<Root>/Delay One Step2' */
  real_T DelayOneStep3_DSTATE;         /* '<Root>/Delay One Step3' */
};

/* External inputs (root inport signals with default storage) */
struct ExtU_fuzzyPID_T {
  real_T Input;                        /* '<Root>/Input' */
  real_T Input1;                       /* '<Root>/Input1' */
};

/* External outputs (root outports fed by signals with default storage) */
struct ExtY_fuzzyPID_T {
  real_T Output;                       /* '<Root>/Output' */
};

/* Parameters (default storage) */
struct P_fuzzyPID_T_ {
  real_T K;                            /* Variable: K
                                        * Referenced by: '<Root>/Gain2'
                                        */
  real_T Kd;                           /* Variable: Kd
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T Ki;                           /* Variable: Ki
                                        * Referenced by: '<Root>/Gain3'
                                        */
  real_T Kp;                           /* Variable: Kp
                                        * Referenced by: '<Root>/Gain6'
                                        */
  real_T KuI;                          /* Variable: KuI
                                        * Referenced by: '<Root>/Gain4'
                                        */
  real_T KuPD;                         /* Variable: KuPD
                                        * Referenced by: '<Root>/Gain7'
                                        */
  real_T T;                            /* Variable: T
                                        * Referenced by:
                                        *   '<Root>/Gain'
                                        *   '<Root>/Gain5'
                                        */
  real_T DelayOneStep_InitialCondition;/* Expression: 0.0
                                        * Referenced by: '<Root>/Delay One Step'
                                        */
  real_T DelayOneStep1_InitialCondition;/* Expression: 0.0
                                         * Referenced by: '<Root>/Delay One Step1'
                                         */
  real_T DelayOneStep4_InitialCondition;/* Expression: 0.0
                                         * Referenced by: '<Root>/Delay One Step4'
                                         */
  real_T DelayOneStep2_InitialCondition;/* Expression: 0.0
                                         * Referenced by: '<Root>/Delay One Step2'
                                         */
  real_T DelayOneStep3_InitialCondition;/* Expression: 0.0
                                         * Referenced by: '<Root>/Delay One Step3'
                                         */
};

/* Real-time Model Data Structure */
struct tag_RTM_fuzzyPID_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Class declaration for model fuzzyPID */
class fuzzyPIDModelClass
{
  /* public data and function members */
 public:
  /* Copy Constructor */
  fuzzyPIDModelClass(fuzzyPIDModelClass const&) =delete;

  /* Assignment Operator */
  fuzzyPIDModelClass& operator= (fuzzyPIDModelClass const&) & = delete;

  /* Real-Time Model get method */
  RT_MODEL_fuzzyPID_T * getRTM();

  /* External inputs */
  ExtU_fuzzyPID_T fuzzyPID_U;

  /* External outputs */
  ExtY_fuzzyPID_T fuzzyPID_Y;

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  static void terminate();

  /* Constructor */
  fuzzyPIDModelClass();

  /* Destructor */
  ~fuzzyPIDModelClass();

  /* private data and function members */
 private:
  /* Block signals */
  B_fuzzyPID_T fuzzyPID_B;

  /* Block states */
  DW_fuzzyPID_T fuzzyPID_DW;

  /* Tunable parameters */
  static P_fuzzyPID_T fuzzyPID_P;

  /* Real-Time Model */
  RT_MODEL_fuzzyPID_T fuzzyPID_M;
};

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
 * '<Root>' : 'fuzzyPID'
 */
#endif                                 /* RTW_HEADER_fuzzyPID_h_ */
