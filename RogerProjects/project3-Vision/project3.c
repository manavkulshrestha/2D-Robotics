/*************************************************************************/
/* File:        project3.c                                               */
/* Description: User project #3 - empty project directory for project    */
/*              development                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "RLToolKit.h"

/***********************************************************************/
/* default parameters for the state-action value table used to compose */
/* skills as sequential decision to engage primative actions           */
/*    all actions/skills return status values in {NOREF, !CONV, COV}   */
/*    where NOREF = 0, !CONV=1, CONV=2                                 */
#define NACTIONS 1
#define NSTATES  3    // 3^(NACTIONS)

// only used if using the RL toolkit
#define Q_TABLE_FILE ""
#define CONVG_ACT 0
// end RLToolKit parameters

double proj_three_q_table[NSTATES][NACTIONS] = {0.0};
/**************************************************************/

int pixel_is_red(int *pixel) {
  return pixel[0] == 255 && !pixel[1] && !pixel[2];
}

int average_red_pixel(Robot *roger, int *u)
{
  int foundRed = FALSE;

  for (int i=0; i<NEYES; i++) {
    int j = 0;

    while (j < NPIXELS && !pixel_is_red(roger->image[i][j]))
      j++;

    int startIdx = j, endIdx = j;

    while(j < NPIXELS && pixel_is_red(roger->image[i][j])) {
      endIdx = j++;
      foundRed = TRUE;
    }

    u[i] = (int) (startIdx+endIdx)/2;
  }

  return foundRed;
}

/* 
* Primitive Skill prototype function.
*/
int track3(roger, errors, time)
Robot* roger;
double errors[NDOF];
double time;
{
  static int return_status = NO_REFERENCE;

  // first, do no harm
  for (int i=0; i<NDOF; ++i) {
    errors[i] = 0.0;
  }

  int u[NEYES];

  if (!average_red_pixel(roger, u))
    return return_status;

  for (int i=0; i<NEYES; i++)
    errors[2+i] = -atan2(NPIXELS/2 - u[i], FOCAL_LENGTH);

  return TRANSIENT;
}

void project3_control(roger, time)
Robot* roger;
double time;
{
	// starter code for testing primitive actions
	double errors[NDOF];

	track3(roger, errors, time);
	submit_errors(roger, errors);
}

// void project3_control(roger, time)
// Robot* roger;
// double time;
// { 
//   int u[NEYES];

//   if (!average_red_pixel(roger, u))
//     return;

//   double theta_error[NEYES];
//   for (int i=0; i<NEYES; i++)
//     theta_error[i] = atan2(NPIXELS/2 - u[i], FOCAL_LENGTH);

//   for (int i=0; i<NEYES; i++)
//     roger->eyes_setpoint[i] = roger->eye_theta[i] - theta_error[i];
// }

/************************************************************************/
void project3_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project3_enter_params()
{
  printf("Project 3 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project3_visualize(roger)
Robot* roger;
{ }
