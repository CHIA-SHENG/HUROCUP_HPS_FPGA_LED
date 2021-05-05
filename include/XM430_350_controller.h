/*
 * File: XM430_350_controller.h
 */
/*=======================================================================*
 * Target hardware information
 *   Device type: ARM Compatible->ARM 9
 *   Number of bits:     char:   8    short:   16    int:  32
 *                       long:  32    long long:  64
 *                       native word size:  32
 *=======================================================================*/

/*=======================================================================*
 * Fixed width word size data types:                                     *
 *   int8_T, int16_T, int32_T     - signed 8, 16, or 32 bit integers     *
 *   uint8_T, uint16_T, uint32_T  - unsigned 8, 16, or 32 bit integers   *
 *   real32_T, real64_T           - 32 and 64 bit floating point numbers *
 *=======================================================================*/
#include "string.h"
#include "stdlib.h"
typedef unsigned int uint32;

/* External inputs struct */
typedef struct {
  double ref_pos;                      /* referenece_position */
  double present_position;             /* present_position */
  double initial_position;             /* initial_position */
  double state_hat[3];                 /* state_hat */
} ExI;

/* External outputs struct */
typedef struct {
  double position_command;             /* position_command */
  double velocity_command;             /* velocity_command */
  double state[3];                     /* state */
  double Integrator_STATE;
} ExO;

/* Constant parameters */
typedef struct {
  double A_Gain[9];                    /* system matrix A */
  double B_Gain[3];                    /* system matrix B */
  double C_Gain[3];                    /* output matrix C */
  double L_Gain[3];                    /* observer matrix L */
  double Ke_Gain[1];                   /* gain matrix Ke */
  double K_Gain[3];                    /* gain matrix K */
} ConstP;

/*
* Timing:
* The following substructure contains information regarding
* the timing information for the model.
*/
typedef struct {
  uint32 clockTick;                   /* counter */
  double stepSize0;                   /* fixed-step */
  double simpleTime;                  /* simpling time */
  int Iteration_f;                    /* frequency of iteration */
} Timing;

/* The struct to save the states at last moment */
typedef struct {
  double last_output_angle_[21];
  double state_last[63];
  double last_Integrator_STATE[21];
} Register;

/* Constant parameters */
extern const ConstP rtConstP;

/* External inputs */
extern ExI rtU;

/* External outputs */
extern ExO rtY;

/* Continuous states */
extern Timing rtTiming;

/* Continuous states */
extern Register rtRegister;

/* Model entry point functions */
extern void XM430_350_Controller_initialize(void);
extern void XM430_350_Controller(void);

