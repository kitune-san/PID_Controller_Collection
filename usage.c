#include <stdio.h>
#include "PID.h"


short int control_target(short int mv)
{
  static int pv = 0;
  pv += ((mv * 33) - pv) / 10;
  return pv;
}


int main(void)
{
  const int time_unit = 10;   // Time-Unit(10) == 0.1ms (EXAMPLE)
  int ms = 0;
  short int SV = 0;
  short int PV = 0;
  short int MV = 0;

  PID_Parameter param = {
    .Kp_num   = 1,              // Kp = 1/100
    .Kp_den   = 100,
    .Ti       = 5*time_unit,    // Ti = 0.5ms (Unit: 0.1ms)
    .Td       = 5*time_unit,    // Td = 0.5ms (Unit: 0.1ms)
    //.Td       = 0,              // Td = disable
    .eta_den  = 10,             // η=1/10=0.1
    .Tc       = 10*time_unit,   // 10ms (Unit: 0.1ms)
  };

  PID_State state;
  Initialize_PID_State(&state);

  printf("elapsed (ms),\tSV,\tPV,\tMV,\tΔD\r\n");

  PV = control_target(MV);
  printf("%d,\t%d,\t%d,\t%d,\t%d\r\n", ms, SV, PV, MV, state.dD[0]);

  for (ms = 1; ms < 1000; ms++) {
    /* Setting SV */
    if (ms < 500) {
      SV = 10000;
    } else {
      SV = 5000;
    }

    /* PID Controller */
    if ((ms % (param.Tc/time_unit)) == 0) {
#if 1
      MV += PID_Reverse_Acting(&param, &state, SV, PV);
#else
      MV += PI_D_Reverse_Acting(&param, &state, SV, PV);
#endif
    }

    /* Clipper */
    if (MV >= 1000) {
      MV = 1000;
    }
    if (MV <= -1000) {
      MV = -1000;
    }

    /* Output MV & Sampling PV */
    PV = control_target(MV);
    printf("%d,\t%d,\t%d,\t%d,\t%d\r\n", ms, SV, PV, MV, state.dD[0]);
  }
  return 0;
}

