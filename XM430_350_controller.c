/*
 * File: XM430_350_controller.c
 */
#include "XM430_350_controller.h"
 ExI rtU;
 ExO rtY;
 Register rtRegister;
/* Constant parameters */
const ConstP rtConstP = {
  /* Expression: A */
  { 0.0, 0.0, 0.0, 1.0, 0.0, -54.348, 0.0, 1.0, -13.13 },
  /* Expression: B */
  { 0.0, 0.0, 1.0 },
  /* Expression: C */
  { 869.6, 0.0, 0.0 },
  /* Expression: L */
  { 0.0370, 0.2384, -1.1812 },
  /* Expression: Ke */
  { -37.4636 },
  /* Expression: K */
  { 11391.0, 1302.4, 49.98 }
};

Timing rtTiming = {
  0, 0.0001,  0.03, 0
};

/* Model step function */
void XM430_350_Controller(void)
{
  /* Register for operator */
  double rtb_Sum1_g;                   
  double Sum2[3] = {0};
  int i;

  /* tracking_error = present_position - reference_position */
  rtb_Sum1_g = rtU.ref_pos - rtU.present_position;

  /* Integrator */
  rtY.Integrator_STATE += rtb_Sum1_g * rtTiming.stepSize0;

  /* Outport: velocity_command */
  rtY.velocity_command = abs(rtConstP.Ke_Gain[0] * rtY.Integrator_STATE - ((rtConstP.K_Gain[0] *
    rtU.state_hat[0] + rtConstP.K_Gain[1] * rtU.state_hat[1]) +
    rtConstP.K_Gain[2] * rtU.state_hat[2]));

  /* Sum2 is the differential states */
  rtb_Sum1_g = (rtU.present_position - rtU.initial_position) - ((rtConstP.C_Gain[0] *
    rtU.state_hat[0] + rtConstP.C_Gain[1] * rtU.state_hat[1]) + rtConstP.C_Gain[2] * rtU.state_hat[2]);
  for (i = 0; i < 3; i++) {
    Sum2[i] = (((rtConstP.A_Gain[i + 3] * rtU.state_hat[1] +
                      rtConstP.A_Gain[i] * rtU.state_hat[0]) + rtConstP.A_Gain[i
                     + 6] * rtU.state_hat[2]) + rtConstP.B_Gain[i] * rtY.velocity_command) +
      rtConstP.L_Gain[i] * rtb_Sum1_g;
  }

  /* position_command */
  rtY.position_command = rtU.ref_pos;

  /* state x */
  rtY.state[0] += Sum2[0] * rtTiming.stepSize0;
  rtY.state[1] += Sum2[1] * rtTiming.stepSize0;
  rtY.state[2] += Sum2[2] * rtTiming.stepSize0;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
  */
  if (rtTiming.clockTick < rtTiming.Iteration_f)
  {
    rtTiming.clockTick++;
    rtU.state_hat[0] = rtY.state[0];
    rtU.state_hat[1] = rtY.state[1];
    rtU.state_hat[2] = rtY.state[2];
    XM430_350_Controller();
  }
  else
  {
    rtTiming.clockTick = 0;
  }
}



/* Model initialize function */
void XM430_350_Controller_initialize(void)
{
  rtTiming.clockTick = 0;
  rtU.ref_pos = 0;                      /* referenece_position */
  rtU.present_position = 0;             /* present_position */
  rtU.initial_position = 0;             /* initial_position */
  memset(rtU.state_hat, 0, sizeof(rtU.state_hat));
  rtY.position_command = 0;             /* position_command */
  rtY.velocity_command = 0;             /* velocity_command */
  memset(rtY.state, 0, sizeof(rtY.state));
  memset(rtRegister.last_output_angle_, 0, sizeof(rtRegister.last_output_angle_));
  memset(rtRegister.state_last, 0, sizeof(rtRegister.state_last));
  memset(rtRegister.last_Integrator_STATE, 0, sizeof(rtRegister.last_Integrator_STATE));
  rtY.Integrator_STATE = 0;
  rtTiming.Iteration_f = rtTiming.simpleTime/rtTiming.stepSize0;
}