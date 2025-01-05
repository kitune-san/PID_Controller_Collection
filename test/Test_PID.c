#include <stdio.h>
#include "./TesUT/tesut.h"
#include "./TesUT/additions/tesut-lib.h"
#include "./TesUT/additions/tesut-console.h"
#include "../PID.c"

static void Add_Int_Func_Test_01(void)
{
  int error_flag = 0;
  int ret = Add_Int(27, -78, &error_flag);

  TESU_ASSERT_EQUAL(ret, -51);
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Add_Int_Func_Test_02(void)
{
  int error_flag = 0;
  int ret = Add_Int((INT_MAX-6), 6, &error_flag);

  TESU_ASSERT_EQUAL(ret, INT_MAX);
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Add_Int_Func_Test_03(void)
{
  int error_flag = 0;
  int ret = Add_Int((INT_MAX-6), 7, &error_flag);

  TESU_ASSERT_EQUAL(ret, INT_MAX);
  TESU_ASSERT_EQUAL(error_flag, PID_ERROR);
  return;
}

static void Add_Int_Func_Test_04(void)
{
  int error_flag = 0;
  int ret = Add_Int((INT_MIN+6), -6, &error_flag);

  TESU_ASSERT_EQUAL(ret, INT_MIN);
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Add_Int_Func_Test_05(void)
{
  int error_flag = 0;
  int ret = Add_Int((INT_MIN+6), -7, &error_flag);

  TESU_ASSERT_EQUAL(ret, INT_MIN);
  TESU_ASSERT_EQUAL(error_flag, PID_ERROR);
  return;
}

static void Sub_Int_Func_Test_01(void)
{
  int error_flag = 0;
  int ret = Sub_Int(312, -534, &error_flag);

  TESU_ASSERT_EQUAL(ret, 846);
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Sub_Int_Func_Test_02(void)
{
  int error_flag = 0;
  int ret = Sub_Int((INT_MIN+5), 5, &error_flag);

  TESU_ASSERT_EQUAL(ret, INT_MIN);
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Sub_Int_Func_Test_03(void)
{
  int error_flag = 0;
  int ret = Sub_Int((INT_MIN+5), 6, &error_flag);

  TESU_ASSERT_EQUAL(ret, INT_MIN);
  TESU_ASSERT_EQUAL(error_flag, PID_ERROR);
  return;
}

static void Sub_Int_Func_Test_04(void)
{
  int error_flag = 0;
  int ret = Sub_Int((INT_MAX-5), -5, &error_flag);

  TESU_ASSERT_EQUAL(ret, INT_MAX);
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Sub_Int_Func_Test_05(void)
{
  int error_flag = 0;
  int ret = Sub_Int((INT_MAX-5), -6, &error_flag);

  TESU_ASSERT_EQUAL(ret, INT_MAX);
  TESU_ASSERT_EQUAL(error_flag, PID_ERROR);
  return;
}


static void Sub_Short_Int_Func_Test_01(void)
{
  int error_flag = 0;
  int ret = Sub_Short_Int(-56, 78, &error_flag);

  TESU_ASSERT_EQUAL(ret, -134);
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Sub_Short_Int_Func_Test_02(void)
{
  int error_flag = 0;
  int ret = Sub_Short_Int((SHRT_MIN+9), 9, &error_flag);

  TESU_ASSERT_EQUAL(ret, SHRT_MIN);
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Sub_Short_Int_Func_Test_03(void)
{
  int error_flag = 0;
  int ret = Sub_Short_Int((SHRT_MIN+9), 10, &error_flag);

  TESU_ASSERT_EQUAL(ret, SHRT_MIN);
  TESU_ASSERT_EQUAL(error_flag, PID_ERROR);
  return;
}

static void Sub_Short_Int_Func_Test_04(void)
{
  int error_flag = 0;
  int ret = Sub_Short_Int((SHRT_MAX-9), -9, &error_flag);

  TESU_ASSERT_EQUAL(ret, SHRT_MAX);
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Sub_Short_Int_Func_Test_05(void)
{
  int error_flag = 0;
  int ret = Sub_Short_Int((SHRT_MAX-9), -10, &error_flag);

  TESU_ASSERT_EQUAL(ret, SHRT_MAX);
  TESU_ASSERT_EQUAL(error_flag, PID_ERROR);
  return;
}

static void Integral_Controller_Func_Test_01(void)
{
  PID_Parameter param;
  param.Tc  = 10;
  param.Ti  = 4;
  int error_flag = 0;
  int ret = Integral_Controller(&param, 24, &error_flag);

  TESU_ASSERT_EQUAL(ret, 60);
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Integral_Controller_Func_Test_02(void)
{
  PID_Parameter param;
  param.Tc  = SHRT_MAX;
  param.Ti  = 1;
  int error_flag = 0;
  int ret = Integral_Controller(&param, SHRT_MAX, &error_flag);

  TESU_ASSERT_EQUAL(ret, ((int)SHRT_MAX * SHRT_MAX));
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Integral_Controller_Func_Test_03(void)
{
  PID_Parameter param;
  param.Tc  = SHRT_MIN;
  param.Ti  = -1;
  int error_flag = 0;
  int ret = Integral_Controller(&param, SHRT_MIN, &error_flag);

  TESU_ASSERT_EQUAL(ret, ((int)SHRT_MIN * SHRT_MIN * -1));
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Integral_Controller_Func_Test_04(void)
{
  PID_Parameter param;
  param.Tc  = 10;
  param.Ti  = 0;
  int error_flag = 0;
  int ret = Integral_Controller(&param, 24, &error_flag);

  TESU_ASSERT_EQUAL(ret, 0);
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Integral_Controller_Func_Test_05(void)
{
  int error_flag = 0;
  int ret = Integral_Controller(NULL, 24, &error_flag);
  TESU_ASSERT_EQUAL(ret, 0);
  TESU_ASSERT_EQUAL(error_flag, PID_ERROR);
  return;
}

static void Differential_Controller_Func_Test_01(void)
{
  PID_Parameter param;
  param.Td      = 50;
  param.Tc      = 100;
  param.eta_den = 10;
  short int history[3] = {234, 0, 0};
  int error_flag = 0;

  int ret = Differential_Controller(&param, history, 0, &error_flag);
  TESU_ASSERT_EQUAL(ret, 111);
  TESU_ASSERT_EQUAL(error_flag, 0);

  history[0] = 0;
  history[1] = 234;
  history[2] = 0;

  ret = Differential_Controller(&param, history, 0, &error_flag);
  TESU_ASSERT_EQUAL(ret, -222);
  TESU_ASSERT_EQUAL(error_flag, 0);

  history[0] = 0;
  history[1] = 0;
  history[2] = 234;

  ret = Differential_Controller(&param, history, 0, &error_flag);
  TESU_ASSERT_EQUAL(ret, 111);
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Differential_Controller_Func_Test_02(void)
{
  PID_Parameter param;
  param.Td      = 50;
  param.Tc      = 100;
  param.eta_den = 10;
  short int history[3] = {0, 234, 0};
  int error_flag = 0;

  int ret = Differential_Controller(&param, history, 111, &error_flag);
  TESU_ASSERT_EQUAL(ret, -217);
  TESU_ASSERT_EQUAL(error_flag, 0);

  history[0] = 0;
  history[1] = 0;
  history[2] = 234;

  ret = Differential_Controller(&param, history, -217, &error_flag);
  TESU_ASSERT_EQUAL(ret, 101);
  TESU_ASSERT_EQUAL(error_flag, 0);

  history[0] = 0;
  history[1] = 0;
  history[2] = 0;

  ret = Differential_Controller(&param, history, 101, &error_flag);
  TESU_ASSERT_EQUAL(ret, 4);
  TESU_ASSERT_EQUAL(error_flag, 0);

  ret = Differential_Controller(&param, history, 4, &error_flag);
  TESU_ASSERT_EQUAL(ret, 0);
  TESU_ASSERT_EQUAL(error_flag, 0);

  return;
}

static void Differential_Controller_Func_Test_03(void)
{
  PID_Parameter param;
  param.Td      = 0;
  param.Tc      = 100;
  param.eta_den = 10;
  short int history[3] = {0, 234, 0};
  int error_flag = 0;

  int ret = Differential_Controller(&param, history, 111, &error_flag);
  TESU_ASSERT_EQUAL(ret, 0);
  TESU_ASSERT_EQUAL(error_flag, 0);

  return;
}

static void Differential_Controller_Func_Test_04(void)
{
  PID_Parameter param;
  param.Td      = 50;
  param.Tc      = 100;
  param.eta_den = 0;
  short int history[3] = {0, 234, 0};
  int error_flag = 0;

  int ret = Differential_Controller(&param, history, 111, &error_flag);
  TESU_ASSERT_EQUAL(ret, -234);
  TESU_ASSERT_EQUAL(error_flag, 0);
  return;
}

static void Differential_Controller_Func_Test_05(void)
{
  PID_Parameter param;
  param.Td      = 50;
  param.Tc      = 100;
  param.eta_den = 10;
  short int history[3] = {0, 0, 0};
  int error_flag = 0;

  int ret = Differential_Controller(&param, history, ((int)SHRT_MIN-1)*10, &error_flag);
  TESU_ASSERT_EQUAL(ret, (int)SHRT_MIN*50/(100+(50/10)));
  TESU_ASSERT_EQUAL(error_flag, PID_ERROR);
  return;
}

static void Differential_Controller_Func_Test_06(void)
{
  PID_Parameter param;
  param.Td      = 50;
  param.Tc      = 100;
  param.eta_den = 10;
  short int history[3] = {0, 0, 0};
  int error_flag = 0;

  int ret = Differential_Controller(&param, history, ((int)SHRT_MAX+1)*10, &error_flag);
  TESU_ASSERT_EQUAL(ret, (int)SHRT_MAX*50/(100+(50/10)));
  TESU_ASSERT_EQUAL(error_flag, PID_ERROR);
  return;
}

static void Differential_Controller_Func_Test_07(void)
{
  PID_Parameter param;
  param.Td      = 50;
  param.Tc      = 0;
  param.eta_den = 0;
  short int history[3] = {300, 0, 0};
  int error_flag = 0;

  int ret = Differential_Controller(&param, history, 0, &error_flag);
  TESU_ASSERT_EQUAL(ret, INT_MAX);
  TESU_ASSERT_EQUAL(error_flag, PID_ERROR);
  return;
}

static void Differential_Controller_Func_Test_08(void)
{
  PID_Parameter param;
  param.Td      = 50;
  param.Tc      = 0;
  param.eta_den = 0;
  short int history[3] = {0, 300, 0};
  int error_flag = 0;

  int ret = Differential_Controller(&param, history, 0, &error_flag);
  TESU_ASSERT_EQUAL(ret, INT_MIN);
  TESU_ASSERT_EQUAL(error_flag, PID_ERROR);
  return;
}

static void Differential_Controller_Func_Test_09(void)
{
  PID_Parameter param;
  param.Td      = 50;
  param.Tc      = 0;
  param.eta_den = 0;
  short int history[3] = {0, 0, 0};
  int error_flag = 0;

  int ret = Differential_Controller(&param, history, 0, &error_flag);
  TESU_ASSERT_EQUAL(ret, 0);
  TESU_ASSERT_EQUAL(error_flag, PID_ERROR);
  return;
}

static void Differential_Controller_Func_Test_10(void)
{
  short int history[3] = {0, 234, 0};
  int error_flag = 0;

  int ret = Differential_Controller(NULL, history, 111, &error_flag);
  TESU_ASSERT_EQUAL(ret, 0);
  TESU_ASSERT_EQUAL(error_flag, PID_ERROR);
  return;
}

static void Differential_Controller_Func_Test_11(void)
{
  PID_Parameter param;
  param.Td      = 50;
  param.Tc      = 100;
  param.eta_den = 10;
  int error_flag = 0;

  int ret = Differential_Controller(&param, NULL, 111, &error_flag);
  TESU_ASSERT_EQUAL(ret, 0);
  TESU_ASSERT_EQUAL(error_flag, PID_ERROR);
  return;
}

static void PID_Common_Func_Test_01(void)
{
  PID_Parameter param = {
    .Kp_num   = 1,
    .Kp_den   = 100,
    .Ti       = 50,
    .Td       = 50,
    .eta_den  = 10,
    .Tc       = 100,
  };
  PID_State state = {0};
  state.E[0] = 300;

  int ret = PID_Common(&param, &state, 300, state.E);
  /*
   * P = 300 - 0 = 300
   * I = 300 * 100 / 50 = 600
   * D = (0/10 + 300 - 2*0 + 0) * 50 / (100 + (50/10)) = 142
   * (P+I+D)Kp = (300 + 600 + 142) / 100 = 10
   */
  TESU_ASSERT_EQUAL(ret, 10);
  TESU_ASSERT_EQUAL(state.dD[0], 142);
  TESU_ASSERT_EQUAL(state.dD[1], 0);
  TESU_ASSERT_EQUAL(state.error_flag, 0);

  state.E[2] = 0;
  state.E[1] = 300;
  state.E[0] = 150;
  ret = PID_Common(&param, &state, 150, state.E);
  /*
   * P = 150 - 300 = -150
   * I = 150 * 100 / 50 = 300
   * D = (142/10 + 150 - 2*300 + 0) * 50 / (100 + (50/10)) = -207
   * (P+I+D)Kp = (-150 + 300 + -207) / 100 = 0
   */
  TESU_ASSERT_EQUAL(ret, 0);
  TESU_ASSERT_EQUAL(state.dD[0], -207);
  TESU_ASSERT_EQUAL(state.dD[1], 142);
  TESU_ASSERT_EQUAL(state.error_flag, 0);

  state.E[2] = 300;
  state.E[1] = 150;
  state.E[0] = 150;
  ret = PID_Common(&param, &state, 150, state.E);
  /*
   * P = 150 - 150 = 0
   * I = 150 * 100 / 50 = 300
   * D = (-207/10 + 150 - 2*150 + 300) * 50 / (100 + (50/10)) = 61
   * (P+I+D)Kp = (0 + 300 + 61) / 100 = 3
   */
  TESU_ASSERT_EQUAL(ret, 3);
  TESU_ASSERT_EQUAL(state.dD[0], 61);
  TESU_ASSERT_EQUAL(state.dD[1], -207);
  TESU_ASSERT_EQUAL(state.error_flag, 0);

  return;
}

static void PID_Reverse_Acting_Test_01(void)
{
  PID_Parameter param = {
    .Kp_num   = 1,
    .Kp_den   = 100,
    .Ti       = 50,
    .Td       = 50,
    .eta_den  = 10,
    .Tc       = 100,
  };
  PID_State state = {0};

  short int ret = PID_Reverse_Acting(&param, &state, 300, 0);
  /*
   * e = 300 - 0 = 300
   * P = 300 - 0 = 300
   * I = 300 * 100 / 50 = 600
   * D = (0/10 + 300 - 2*0 + 0) * 50 / (100 + (50/10)) = 142
   * (P+I+D)Kp = (300 + 600 + 142) / 100 = 10
   */
  TESU_ASSERT_EQUAL(ret,              10);
  TESU_ASSERT_EQUAL(state.E[0],       300);
  TESU_ASSERT_EQUAL(state.E[1],       0);
  TESU_ASSERT_EQUAL(state.E[2],       0);
  TESU_ASSERT_EQUAL(state.dD[0],      142);
  TESU_ASSERT_EQUAL(state.dD[1],      0);
  TESU_ASSERT_EQUAL(state.error_flag, 0);

  ret = PID_Reverse_Acting(&param, &state, 300, 160);
  /*
   * e = 300 - 160 = 140
   * P = 140 - 300 = -160
   * I = 140 * 100 / 50 = 280
   * D = (142/10 + 140 - 2*300 + 0) * 50 / (100 + (50/10)) = -212
   * (P+I+D)Kp = (-160 + 280 + -202) / 100 = 0
   */
  TESU_ASSERT_EQUAL(ret,              0);
  TESU_ASSERT_EQUAL(state.E[0],       140);
  TESU_ASSERT_EQUAL(state.E[1],       300);
  TESU_ASSERT_EQUAL(state.E[2],       0);
  TESU_ASSERT_EQUAL(state.dD[0],      -212);
  TESU_ASSERT_EQUAL(state.dD[1],      142);
  TESU_ASSERT_EQUAL(state.error_flag, 0);

  ret = PID_Reverse_Acting(&param, &state, 300, 160);
  /*
   * e = 300 - 160 = 140
   * P = 140 - 140 = 0
   * I = 140 * 100 / 50 = 280
   * D = (-212/10 + 140 - 2*140 + 300) * 50 / (100 + (50/10)) = 66
   * (P+I+D)Kp = (0 + 280 + 66) / 100 = 3
   */
  TESU_ASSERT_EQUAL(ret,              3);
  TESU_ASSERT_EQUAL(state.E[0],       140);
  TESU_ASSERT_EQUAL(state.E[1],       140);
  TESU_ASSERT_EQUAL(state.E[2],       300);
  TESU_ASSERT_EQUAL(state.dD[0],      66);
  TESU_ASSERT_EQUAL(state.dD[1],      -212);
  TESU_ASSERT_EQUAL(state.error_flag, 0);
  return;
}

static void PID_Direct_Acting_Test_01(void)
{
  PID_Parameter param = {
    .Kp_num   = 1,
    .Kp_den   = 100,
    .Ti       = 50,
    .Td       = 50,
    .eta_den  = 10,
    .Tc       = 100,
  };
  PID_State state = {0};

  short int ret = PID_Direct_Acting(&param, &state, 300, 0);
  /*
   * e = 0 - 300 = -300
   * P = -300 - 0 = -300
   * I = -300 * 100 / 50 = -600
   * D = (0/10 + -300 - 2*0 + 0) * 50 / (100 + (50/10)) = -142
   * (P+I+D)Kp = (-300 + -600 + -142) / 100 = -10
   */
  TESU_ASSERT_EQUAL(ret,              -10);
  TESU_ASSERT_EQUAL(state.E[0],       -300);
  TESU_ASSERT_EQUAL(state.E[1],       0);
  TESU_ASSERT_EQUAL(state.E[2],       0);
  TESU_ASSERT_EQUAL(state.dD[0],      -142);
  TESU_ASSERT_EQUAL(state.dD[1],      0);
  TESU_ASSERT_EQUAL(state.error_flag, 0);

  ret = PID_Direct_Acting(&param, &state, 300, 160);
  /*
   * e = 160 - 300 = -140
   * P = -140 - (-300) = 160
   * I = -140 * 100 / 50 = -280
   * D = (-142/10 + -140 - 2*(-300) + 0) * 50 / (100 + (50/10)) = 212
   * (P+I+D)Kp = (160 + -280 + 212) / 100 = 0
   */
  TESU_ASSERT_EQUAL(ret,              0);
  TESU_ASSERT_EQUAL(state.E[0],       -140);
  TESU_ASSERT_EQUAL(state.E[1],       -300);
  TESU_ASSERT_EQUAL(state.E[2],       0);
  TESU_ASSERT_EQUAL(state.dD[0],      212);
  TESU_ASSERT_EQUAL(state.dD[1],      -142);
  TESU_ASSERT_EQUAL(state.error_flag, 0);

  ret = PID_Direct_Acting(&param, &state, 300, 160);
  /*
   * e = 160 - 300 = -140
   * P = -140 - (-140) = 0
   * I = -140 * 100 / 50 = -280
   * D = (212/10 + -140 - 2*(-140) + (-300)) * 50 / (100 + (50/10)) = -66
   * (P+I+D)Kp = (0 + -280 + -66) / 100 = -3
   */
  TESU_ASSERT_EQUAL(ret,              -3);
  TESU_ASSERT_EQUAL(state.E[0],       -140);
  TESU_ASSERT_EQUAL(state.E[1],       -140);
  TESU_ASSERT_EQUAL(state.E[2],       -300);
  TESU_ASSERT_EQUAL(state.dD[0],      -66);
  TESU_ASSERT_EQUAL(state.dD[1],      212);
  TESU_ASSERT_EQUAL(state.error_flag, 0);
  return;
}

static void PI_D_Reverse_Acting_Test_01(void)
{
  PID_Parameter param = {
    .Kp_num   = 1,
    .Kp_den   = 100,
    .Ti       = 50,
    .Td       = 50,
    .eta_den  = 10,
    .Tc       = 100,
  };
  PID_State state = {0};

  short int ret = PI_D_Reverse_Acting(&param, &state, 300, 10);
  /*
   * e = 300 - 10 = 290
   * P = 290 - 0 = 290
   * I = 290 * 100 / 50 = 580
   * D = (0/10 + (-10) - 2*0 + 0) * 50 / (100 + (50/10)) = -4
   * (P+I+D)Kp = (290 + 580 + -4) / 100 = 8
   */
  TESU_ASSERT_EQUAL(ret,              8);
  TESU_ASSERT_EQUAL(state.P[0],       -10);
  TESU_ASSERT_EQUAL(state.P[1],       0);
  TESU_ASSERT_EQUAL(state.P[2],       0);
  TESU_ASSERT_EQUAL(state.E[0],       290);
  TESU_ASSERT_EQUAL(state.E[1],       0);
  TESU_ASSERT_EQUAL(state.E[2],       0);
  TESU_ASSERT_EQUAL(state.dD[0],      -4);
  TESU_ASSERT_EQUAL(state.dD[1],      0);
  TESU_ASSERT_EQUAL(state.error_flag, 0);

  ret = PI_D_Reverse_Acting(&param, &state, 300, 160);
  /*
   * e = 300 - 160 = 140
   * P = 140 - 290 = -150
   * I = 140 * 100 / 50 = 280
   * D = (-4/10 + (-160) - 2*(-10) + 0) * 50 / (100 + (50/10)) = -66
   * (P+I+D)Kp = (-150 + 280 + -66) / 100 = 0
   */
  TESU_ASSERT_EQUAL(ret,              0);
  TESU_ASSERT_EQUAL(state.P[0],       -160);
  TESU_ASSERT_EQUAL(state.P[1],       -10);
  TESU_ASSERT_EQUAL(state.P[2],       0);
  TESU_ASSERT_EQUAL(state.E[0],       140);
  TESU_ASSERT_EQUAL(state.E[1],       290);
  TESU_ASSERT_EQUAL(state.E[2],       0);
  TESU_ASSERT_EQUAL(state.dD[0],      -66);
  TESU_ASSERT_EQUAL(state.dD[1],      -4);
  TESU_ASSERT_EQUAL(state.error_flag, 0);

  ret = PI_D_Reverse_Acting(&param, &state, 300, 160);
  /*
   * e = 300 - 160 = 140
   * P = 140 - 140 = 0
   * I = 140 * 100 / 50 = 280
   * D = (-66/10 + (-160) - 2*(-160) + (-10)) * 50 / (100 + (50/10)) = 68
   * (P+I+D)Kp = (0 + 280 + 68) / 100 = 3
   */
  TESU_ASSERT_EQUAL(ret,              3);
  TESU_ASSERT_EQUAL(state.P[0],       -160);
  TESU_ASSERT_EQUAL(state.P[1],       -160);
  TESU_ASSERT_EQUAL(state.P[2],       -10);
  TESU_ASSERT_EQUAL(state.E[0],       140);
  TESU_ASSERT_EQUAL(state.E[1],       140);
  TESU_ASSERT_EQUAL(state.E[2],       290);
  TESU_ASSERT_EQUAL(state.dD[0],      68);
  TESU_ASSERT_EQUAL(state.dD[1],      -66);
  TESU_ASSERT_EQUAL(state.error_flag, 0);
  return;
}

static void PI_D_Direct_Acting_Test_01(void)
{
  PID_Parameter param = {
    .Kp_num   = 1,
    .Kp_den   = 100,
    .Ti       = 50,
    .Td       = 50,
    .eta_den  = 10,
    .Tc       = 100,
  };
  PID_State state = {0};

  short int ret = PI_D_Direct_Acting(&param, &state, 300, 10);
  /*
   * e = 10 - 300 = -290
   * P = -290 - 0 = -290
   * I = -290 * 100 / 50 = -580
   * D = (0/10 + 10 - 2*0 + 0) * 50 / (100 + (50/10)) = 4
   * (P+I+D)Kp = (-290 + -580 + 4) / 100 = -8
   */
  TESU_ASSERT_EQUAL(ret,              -8);
  TESU_ASSERT_EQUAL(state.P[0],       10);
  TESU_ASSERT_EQUAL(state.P[1],       0);
  TESU_ASSERT_EQUAL(state.P[2],       0);
  TESU_ASSERT_EQUAL(state.E[0],       -290);
  TESU_ASSERT_EQUAL(state.E[1],       0);
  TESU_ASSERT_EQUAL(state.E[2],       0);
  TESU_ASSERT_EQUAL(state.dD[0],      4);
  TESU_ASSERT_EQUAL(state.dD[1],      0);
  TESU_ASSERT_EQUAL(state.error_flag, 0);

  ret = PI_D_Direct_Acting(&param, &state, 300, 160);
  /*
   * e = 160 - 300 = -140
   * P = -140 - (-290) = 150
   * I = -140 * 100 / 50 = -280
   * D = (4/10 + 160 - 2*10 + 0) * 50 / (100 + (50/10)) = 66
   * (P+I+D)Kp = (150 + -280 + 66) / 100 = 0
   */
  TESU_ASSERT_EQUAL(ret,              0);
  TESU_ASSERT_EQUAL(state.P[0],       160);
  TESU_ASSERT_EQUAL(state.P[1],       10);
  TESU_ASSERT_EQUAL(state.P[2],       0);
  TESU_ASSERT_EQUAL(state.E[0],       -140);
  TESU_ASSERT_EQUAL(state.E[1],       -290);
  TESU_ASSERT_EQUAL(state.E[2],       0);
  TESU_ASSERT_EQUAL(state.dD[0],      66);
  TESU_ASSERT_EQUAL(state.dD[1],      4);
  TESU_ASSERT_EQUAL(state.error_flag, 0);

  ret = PI_D_Direct_Acting(&param, &state, 300, 160);
  /*
   * e = 160 - 300 = -140
   * P = -140 - (-140) = 0
   * I = -140 * 100 / 50 = -280
   * D = (66/10 + 160 - 2*160 + 10) * 50 / (100 + (50/10)) = -68
   * (P+I+D)Kp = (0 + -280 + -68) / 100 = -3
   */
  TESU_ASSERT_EQUAL(ret,              -3);
  TESU_ASSERT_EQUAL(state.P[0],       160);
  TESU_ASSERT_EQUAL(state.P[1],       160);
  TESU_ASSERT_EQUAL(state.P[2],       10);
  TESU_ASSERT_EQUAL(state.E[0],       -140);
  TESU_ASSERT_EQUAL(state.E[1],       -140);
  TESU_ASSERT_EQUAL(state.E[2],       -290);
  TESU_ASSERT_EQUAL(state.dD[0],      -68);
  TESU_ASSERT_EQUAL(state.dD[1],      66);
  TESU_ASSERT_EQUAL(state.error_flag, 0);
  return;
}

static void user_func_putc(const char c)
{
  putchar(c);
  return;
}

int main(void)
{
  TESU_Initialize();
  Set_TESU_Putchar(user_func_putc);
  TESU_Set_Console_Handler();

  TESU_TEST Suite[] = {
    { Add_Int_Func_Test_01,                 "Add_Int_Func_Test_01",                 TESU_NO_FATAL },
    { Add_Int_Func_Test_02,                 "Add_Int_Func_Test_02",                 TESU_NO_FATAL },
    { Add_Int_Func_Test_03,                 "Add_Int_Func_Test_03",                 TESU_NO_FATAL },
    { Add_Int_Func_Test_04,                 "Add_Int_Func_Test_04",                 TESU_NO_FATAL },
    { Add_Int_Func_Test_05,                 "Add_Int_Func_Test_05",                 TESU_NO_FATAL },
    { Sub_Int_Func_Test_01,                 "Sub_Int_Func_Test_01",                 TESU_NO_FATAL },
    { Sub_Int_Func_Test_02,                 "Sub_Int_Func_Test_02",                 TESU_NO_FATAL },
    { Sub_Int_Func_Test_03,                 "Sub_Int_Func_Test_03",                 TESU_NO_FATAL },
    { Sub_Int_Func_Test_04,                 "Sub_Int_Func_Test_04",                 TESU_NO_FATAL },
    { Sub_Int_Func_Test_05,                 "Sub_Int_Func_Test_05",                 TESU_NO_FATAL },
    { Sub_Short_Int_Func_Test_01,           "Sub_Short_Int_Func_Test_01",           TESU_NO_FATAL },
    { Sub_Short_Int_Func_Test_02,           "Sub_Short_Int_Func_Test_02",           TESU_NO_FATAL },
    { Sub_Short_Int_Func_Test_03,           "Sub_Short_Int_Func_Test_03",           TESU_NO_FATAL },
    { Sub_Short_Int_Func_Test_04,           "Sub_Short_Int_Func_Test_04",           TESU_NO_FATAL },
    { Sub_Short_Int_Func_Test_05,           "Sub_Short_Int_Func_Test_05",           TESU_NO_FATAL },
    { Integral_Controller_Func_Test_01,     "Integral_Controller_Func_Test_01",     TESU_NO_FATAL },
    { Integral_Controller_Func_Test_02,     "Integral_Controller_Func_Test_02",     TESU_NO_FATAL },
    { Integral_Controller_Func_Test_03,     "Integral_Controller_Func_Test_03",     TESU_NO_FATAL },
    { Integral_Controller_Func_Test_04,     "Integral_Controller_Func_Test_04",     TESU_NO_FATAL },
    { Integral_Controller_Func_Test_05,     "Integral_Controller_Func_Test_05",     TESU_NO_FATAL },
    { Differential_Controller_Func_Test_01, "Differential_Controller_Func_Test_01", TESU_NO_FATAL },
    { Differential_Controller_Func_Test_02, "Differential_Controller_Func_Test_02", TESU_NO_FATAL },
    { Differential_Controller_Func_Test_03, "Differential_Controller_Func_Test_03", TESU_NO_FATAL },
    { Differential_Controller_Func_Test_04, "Differential_Controller_Func_Test_04", TESU_NO_FATAL },
    { Differential_Controller_Func_Test_05, "Differential_Controller_Func_Test_05", TESU_NO_FATAL },
    { Differential_Controller_Func_Test_06, "Differential_Controller_Func_Test_06", TESU_NO_FATAL },
    { Differential_Controller_Func_Test_07, "Differential_Controller_Func_Test_07", TESU_NO_FATAL },
    { Differential_Controller_Func_Test_08, "Differential_Controller_Func_Test_08", TESU_NO_FATAL },
    { Differential_Controller_Func_Test_09, "Differential_Controller_Func_Test_09", TESU_NO_FATAL },
    { Differential_Controller_Func_Test_10, "Differential_Controller_Func_Test_10", TESU_NO_FATAL },
    { Differential_Controller_Func_Test_11, "Differential_Controller_Func_Test_11", TESU_NO_FATAL },
    { PID_Common_Func_Test_01,              "PID_Common_Func_Test_01",              TESU_NO_FATAL },
    { PID_Reverse_Acting_Test_01,           "PID_Reverse_Acting_Test_01",           TESU_NO_FATAL },
    { PID_Direct_Acting_Test_01,            "PID_Direct_Acting_Test_01",            TESU_NO_FATAL },
    { PI_D_Reverse_Acting_Test_01,          "PI-D_Reverse_Acting_Test_01",          TESU_NO_FATAL },
    { PI_D_Direct_Acting_Test_01,           "PI_D_Direct_Acting_Test_01",           TESU_NO_FATAL },
    TESU_TEST_INFO_NULL
  };
  TESU_Start_Suite(Suite, NULL, NULL, "PID Test");
  return 0;
}
