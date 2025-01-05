/**
 * @file  PID.h
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

#define PID_ERROR   (-1)

typedef struct {
  short int   Kp_num;   // Proportional Gain (Kp = Kp_num / Kp_den)
  short int   Kp_den;
  short int   Ti;       // Integral Time. if Ti=0, disable integral control
  short int   Td;       // Derivative Time. if Td=0, disable derivative control
  short int   eta_den;  // eta (Inexact differential) (η = 1/eta_den)
  short int   Tc;       // Control cycle
} PID_Parameter;

typedef struct {
  short int   P[3];
  short int   E[3];
  int         dD[2];
  int         error_flag;
} PID_State;

void Initialize_PID_State(PID_State *state);
short int PID_Reverse_Acting(const PID_Parameter *param, PID_State *state, short int sv, short int pv);
short int PID_Direct_Acting(const PID_Parameter *param, PID_State *state, short int sv, short int pv);
short int PI_D_Reverse_Acting(const PID_Parameter *param, PID_State *state, short int sv, short int pv);
short int PI_D_Direct_Acting(const PID_Parameter *param, PID_State *state, short int sv, short int pv);

