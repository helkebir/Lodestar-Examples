/*
 * fuzzyPID_data.cpp
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

/* Block parameters (default storage) */
P_fuzzyPID_T fuzzyPIDModelClass::fuzzyPID_P{
  /* Variable: K
   * Referenced by: '<Root>/Gain2'
   */
  1.0,

  /* Variable: Kd
   * Referenced by: '<Root>/Gain1'
   */
  0.1,

  /* Variable: Ki
   * Referenced by: '<Root>/Gain3'
   */
  0.15,

  /* Variable: Kp
   * Referenced by: '<Root>/Gain6'
   */
  -10.0,

  /* Variable: KuI
   * Referenced by: '<Root>/Gain4'
   */
  1.0,

  /* Variable: KuPD
   * Referenced by: '<Root>/Gain7'
   */
  1.0,

  /* Variable: T
   * Referenced by:
   *   '<Root>/Gain'
   *   '<Root>/Gain5'
   */
  0.0001,

  /* Expression: 0.0
   * Referenced by: '<Root>/Delay One Step'
   */
  0.0,

  /* Expression: 0.0
   * Referenced by: '<Root>/Delay One Step1'
   */
  0.0,

  /* Expression: 0.0
   * Referenced by: '<Root>/Delay One Step4'
   */
  0.0,

  /* Expression: 0.0
   * Referenced by: '<Root>/Delay One Step2'
   */
  0.0,

  /* Expression: 0.0
   * Referenced by: '<Root>/Delay One Step3'
   */
  0.0
};
