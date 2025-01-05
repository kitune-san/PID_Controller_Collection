/**
 * @file  PID.c
 * @brief PID Controller Collection (Type: Speed, Integer(Non-float))
 *
 * Copyright (c) 2025, kitune-san
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stddef.h>
#include <limits.h>
#include "PID.h"

/**
 * @brief   Initialize PID State (All Zero)
 * @param   [out] state       PID state
 * @return  None
 */
void Initialize_PID_State(PID_State *state)
{
  static const PID_State init = {0};
  if (state == NULL) {
    return;
  }

#if 0
  state->P[0]       = 0;
  state->P[1]       = 0;
  state->P[2]       = 0;

  state->E[0]       = 0;
  state->E[1]       = 0;
  state->E[2]       = 0;

  state->dD[0]      = 0;
  state->dD[1]      = 0;

  state->error_flag = 0;
#else
  *state            = init;
#endif
  return;
}

/**
 * @brief   Int Addition (v1 + v2)
 * @param   [in]  v1          Variable1
 * @param   [in]  v2          Variable2
 * @param   [out] error_flag  Set error if calculation is not possible
 * @return  v1 + v2
 */
static int Add_Int(int v1, int v2, int *error_flag)
{
  if ((v2 > 0) && (v1 > (INT_MAX - v2))) {
    if (error_flag != NULL) *error_flag = PID_ERROR;
    return INT_MAX;
  }
  if ((v2 < 0) && (v1 < (INT_MIN - v2))) {
    if (error_flag != NULL) *error_flag = PID_ERROR;
    return INT_MIN;
  }
  return v1 + v2;
}

/**
 * @brief   Int subtraction (v1 - v2)
 * @param   [in]  v1          Variable1
 * @param   [in]  v2          Variable2
 * @param   [out] error_flag  Set error if calculation is not possible
 * @return  v1 - v2
 */
static int Sub_Int(int v1, int v2, int *error_flag)
{
  if ((v2 > 0) && (v1 < (INT_MIN + v2))) {
    if (error_flag != NULL) *error_flag = PID_ERROR;
    return INT_MIN;
  }
  if ((v2 < 0) && (v1 > (INT_MAX + v2))) {
    if (error_flag != NULL) *error_flag = PID_ERROR;
    return INT_MAX;
  }
  return v1 - v2;
}

/**
 * @brief   Short int subtraction (v1 - v2)
 * @param   [in]  v1          Variable1
 * @param   [in]  v2          Variable2
 * @param   [out] error_flag  Set error if calculation is not possible
 * @return  v1 - v2
 */
static short int Sub_Short_Int(short int v1, short int v2, int *error_flag)
{
  if ((v2 > 0) && (v1 < (SHRT_MIN + v2))) {
    if (error_flag != NULL) *error_flag = PID_ERROR;
    return SHRT_MIN;
  }
  if ((v2 < 0) && (v1 > (SHRT_MAX + v2))) {
    if (error_flag != NULL) *error_flag = PID_ERROR;
    return SHRT_MAX;
  }
  return v1 - v2;
}

/**
 * @brief   Integral Controller (for speed-type PID)
 * @param   [in]  param       PID parameters
 * @param   [in]  error       Error
 * @param   [out] error_flag  Set error if calculation is not possible
 * @return  Tc/Ti*e
 */
static int Integral_Controller(const PID_Parameter *param, short int error, int *error_flag)
{
  if (param == NULL) {
    if (error_flag != NULL) *error_flag = PID_ERROR;
    return 0;
  }
  if (param->Ti == 0) {
    return 0;
  }
  return ((int)error * param->Tc) / param->Ti;
}

/**
 * @brief   Differential Controller (for speed-type PID)
 * @param   [in]  param       PID parameters
 * @param   [in]  history[3]  Error(or PV) history
 * @param   [in]  D_1         ΔD[1]
 * @param   [out] error_flag  Set error if calculation is not possible
 * @return  Td/(Tc+ηTd) * (ΔD[1]*η + e[0]-2*e[1]+e[2])
 */
static int Differential_Controller(const PID_Parameter *param, short int history[3], int D_1, int *error_flag)
{
  int calc = 0;
  int div = 0;

  if ((param == NULL) || (history == NULL)) {
    if (error_flag != NULL) *error_flag = PID_ERROR;
    return 0;
  }
  if (param->Td == 0) {
    return 0;
  }

  calc = (int)history[0] - (int)2*history[1];
  calc = calc + (int)history[2];
  if (param->eta_den != 0) {
    calc = Add_Int(calc, (D_1 / param->eta_den), error_flag);
  }

  if (calc < SHRT_MIN) {
    calc = SHRT_MIN;
    if (error_flag != NULL) *error_flag = PID_ERROR;
  }
  if (calc > SHRT_MAX) {
    calc = SHRT_MAX;
    if (error_flag != NULL) *error_flag = PID_ERROR;
  }
  calc = calc * param->Td;

  if (param->eta_den != 0) {
    div = (int)param->Tc + (param->Td / param->eta_den);
  } else {
    div = (int)param->Tc + param->Td;
  }
  if (div == 0) {
    if (error_flag != NULL) *error_flag = PID_ERROR;
    if (calc < 0) {
      return INT_MIN;
    } else if (calc > 0) {
      return INT_MAX;
    } else {
      return 0;
    }
  }
  return calc / div;
}

/**
 * @brief   PID Controller (Reverse/Direct Common)
 * @param   [in]  param       PID parameters
 * @param   [io]  state       PID state
 * @param   [in]  error       Error
 * @param   [in]  history[3]  Error(or PV) history
 * @return  ΔMV (Manipulative Variable)
 */
static short int PID_Common(const PID_Parameter *param, PID_State *state, short int error, short int history[3])
{
  int I = 0;
  int dMV = 0;

  if (state == NULL) {
    return 0;
  }
  if (param == NULL) {
    state->error_flag = PID_ERROR;
    return 0;
  }

  /* Integral control */
  I = Integral_Controller(param, error, &(state->error_flag));

  /* Differential control */
  state->dD[1] = state->dD[0];
  state->dD[0] = Differential_Controller(param, history, state->dD[1], &(state->error_flag));

  /* Proportional control */
  dMV = Sub_Int(error,  state->E[1],  &(state->error_flag));
  dMV = Add_Int(dMV,    I,            &(state->error_flag));
  dMV = Add_Int(dMV,    state->dD[0], &(state->error_flag));
  if (dMV < SHRT_MIN) {
    dMV = SHRT_MIN;
    state->error_flag = PID_ERROR;
  }
  if (dMV > SHRT_MAX) {
    dMV = SHRT_MAX;
    state->error_flag = PID_ERROR;
  }
  dMV = (dMV * param->Kp_num) / param->Kp_den;

  if (dMV < SHRT_MIN) {
    dMV = SHRT_MIN;
    state->error_flag = PID_ERROR;
  }
  if (dMV > SHRT_MAX) {
    dMV = SHRT_MAX;
    state->error_flag = PID_ERROR;
  }
  return (short int)dMV;
}

/**
 * @brief   PID Controller (Reverse Acting Mode)
 * @param   [in]  param       PID parameters
 * @param   [io]  state       PID state
 * @param   [in]  sv          Set point Variable
 * @param   [in]  pv          Manipulative Variable
 * @return  ΔMV (Manipulative Variable)
 * @note    e = SV - PV
 * <br>     MV = Kp { e + Tc/Ti*Σe + Td/Tc*(e[0]-e[1]) }
 * <br>     ΔMV = MV[0] - MV[1]
 * <br>         = Kp { e[0] + Tc/Ti*Σe + Td/Tc*(e[0]-e[1]) } - Kp { e[1] + Tc/Ti*Σ[i=1,n](e[i]) + Td/Tc*(e[1]-e[2]) }
 * <br>         = Kp { e[0] - e[1] + Tc/Ti*e[0] + Td/Tc*(e[0]-2*e[1]+e[2]) }
 * <br>
 * <br>     * Replace differential terms with Inexact differential
 * <br>     ΔD[0] = Td/(Tc+ηTd) * (ΔD[1]*η + e[0]-2*e[1]+e[2])
 * <br>     ΔMV = Kp { e[0] - e[1] + Tc/Ti*e[0] + ΔD[0]}
 * <br>
 * <br>     == BLOCK DIAGRAM ==
 * <br>           +              +         MV
 * <br>     SV ----o------+-------o---[P]------[Target]---+--->PV
 * <br>            |-     |      +|+                      |
 * <br>            |      +--[I]--o                       |
 * <br>            |      |       |+                      |
 * <br>            |      +--[D]--+                       |
 * <br>            |                                      |
 * <br>            +--------------------------------------+
 */
short int PID_Reverse_Acting(const PID_Parameter *param, PID_State *state, short int sv, short int pv)
{
  if (state == NULL) {
    return 0;
  }
  state->E[2] = state->E[1];
  state->E[1] = state->E[0];
  state->E[0] = Sub_Short_Int(sv, pv, &(state->error_flag));
  return PID_Common(param, state, state->E[0], state->E);
}

/**
 * @brief   PID Controller (Direct Acting Mode)
 * @param   [in]  param       PID parameters
 * @param   [io]  state       PID state
 * @param   [in]  sv          Set point Variable
 * @param   [in]  pv          Manipulative Variable
 * @return  ΔMV (Manipulative Variable)
 * @note    e = PV - SV
 * <br>     MV = Kp { e + Tc/Ti*Σe + Td/Tc*(e[0]-e[1]) }
 * <br>     ΔMV = MV[0] - MV[1]
 * <br>         = Kp { e[0] + Tc/Ti*Σe + Td/Tc*(e[0]-e[1]) } - Kp { e[1] + Tc/Ti*Σ[i=1,n](e[i]) + Td/Tc*(e[1]-e[2]) }
 * <br>         = Kp { e[0] - e[1] + Tc/Ti*e[0] + Td/Tc*(e[0]-2*e[1]+e[2]) }
 * <br>
 * <br>     * Replace differential terms with Inexact differential
 * <br>     ΔD[0] = Td/(Tc+ηTd) * (ΔD[1]*η + e[0]-2*e[1]+e[2])
 * <br>     ΔMV = Kp { e[0] - e[1] + Tc/Ti*e[0] + ΔD[0] }
 * <br>
 * <br>     == BLOCK DIAGRAM ==
 * <br>           -              +         MV
 * <br>     SV ----o------+-------o---[P]------[Target]---+--->PV
 * <br>            |+     |      +|+                      |
 * <br>            |      +--[I]--o                       |
 * <br>            |      |       |+                      |
 * <br>            |      +--[D]--+                       |
 * <br>            |                                      |
 * <br>            +--------------------------------------+
 */
short int PID_Direct_Acting(const PID_Parameter *param, PID_State *state, short int sv, short int pv)
{
  if (state == NULL) {
    return 0;
  }
  state->E[2] = state->E[1];
  state->E[1] = state->E[0];
  state->E[0] = Sub_Short_Int(pv, sv, &(state->error_flag));
  return PID_Common(param, state, state->E[0], state->E);
}

/**
 * @brief   PI-D Controller (Reverse Acting Mode)
 * @param   [in]  param       PID parameters
 * @param   [io]  state       PID state
 * @param   [in]  sv          Set point Variable
 * @param   [in]  pv          Manipulative Variable
 * @return  ΔMV (Manipulative Variable)
 * @note    e = SV - PV
 * <br>     MV = Kp { e + Tc/Ti*Σe + Td/Tc*(-PV[0]+PV[1]) }
 * <br>     ΔMV = MV[0] - MV[1]
 * <br>         = Kp { e[0] + Tc/Ti*Σe + Td/Tc*(-PV[0]+PV[1]) } - Kp { e[1] + Tc/Ti*Σ[i=1,n](e[i]) + Td/Tc*(-PV[1]+PV[2]) }
 * <br>         = Kp { e[0] - e[1] + Tc/Ti*e[0] + Td/Tc*(-PV[0]+2*PV[1]-PV[2]) }
 * <br>
 * <br>     * Replace differential terms with Inexact differential
 * <br>     ΔD[0] = Td/(Tc+ηTd) * (ΔD[1]*η + -PV[1]+2*PV[2]-PV[3])
 * <br>     ΔMV = Kp { e[0] - e[1] + Tc/Ti*e[0] + ΔD[0] }
 * <br>
 * <br>     == BLOCK DIAGRAM ==
 * <br>           +              +         MV
 * <br>     SV ----o------+-------o---[P]------[Target]---+--->PV
 * <br>            |-     |      +|+                      |
 * <br>            |      +--[I]--o                       |
 * <br>            |              |+                      |
 * <br>            +---------[D]--+                       |
 * <br>            |                                      |
 * <br>            +--------------------------------------+
 */
short int PI_D_Reverse_Acting(const PID_Parameter *param, PID_State *state, short int sv, short int pv)
{
  if (state == NULL) {
    return 0;
  }
  if (pv == SHRT_MIN) {
    pv = -SHRT_MAX;
    state->error_flag = PID_ERROR;
  }
  state->P[2] = state->P[1];
  state->P[1] = state->P[0];
  state->P[0] = -pv;

  state->E[2] = state->E[1];
  state->E[1] = state->E[0];
  state->E[0] = Sub_Short_Int(sv, pv, &(state->error_flag));
  return PID_Common(param, state, state->E[0], state->P);
}

/**
 * @brief   PI-D Controller (Direct Acting Mode)
 * @param   [in]  param       PID parameters
 * @param   [io]  state       PID state
 * @param   [in]  sv          Set point Variable
 * @param   [in]  pv          Manipulative Variable
 * @return  ΔMV (Manipulative Variable)
 * @note    e = PV - SV
 * <br>     MV = Kp { e + Tc/Ti*Σe + Td/Tc*(PV[0]-PV[1]) }
 * <br>     ΔMV = MV[0] - MV[1]
 * <br>         = Kp { e[0] + Tc/Ti*Σe + Td/Tc*(PV[0]-PV[1]) } - Kp { e[1] + Tc/Ti*Σ[i=1,n](e[i]) + Td/Tc*(PV[1]-PV[2]) }
 * <br>         = Kp { e[0] - e[1] + Tc/Ti*e[0] + Td/Tc*(PV[0]-2*PV[1]+PV[2]) }
 * <br>
 * <br>     * Replace differential terms with Inexact differential
 * <br>     ΔD[0] = Td/(Tc+ηTd) * (ΔD[1]*η + PV[1]-2*PV[2]+PV[3])
 * <br>     ΔMV = Kp { e[0] - e[1] + Tc/Ti*e[0] + ΔD[0] }
 * <br>
 * <br>     == BLOCK DIAGRAM ==
 * <br>           -              +         MV
 * <br>     SV ----o------+-------o---[P]------[Target]---+--->PV
 * <br>            |+     |      +|+                      |
 * <br>            |      +--[I]--o                       |
 * <br>            |              |+                      |
 * <br>            +---------[D]--+                       |
 * <br>            |                                      |
 * <br>            +--------------------------------------+
 */
short int PI_D_Direct_Acting(const PID_Parameter *param, PID_State *state, short int sv, short int pv)
{
  if (state == NULL) {
    return 0;
  }
  state->P[2] = state->P[1];
  state->P[1] = state->P[0];
  state->P[0] = pv;

  state->E[2] = state->E[1];
  state->E[1] = state->E[0];
  state->E[0] = Sub_Short_Int(pv, sv, &(state->error_flag));
  return PID_Common(param, state, state->E[0], state->P);
}

