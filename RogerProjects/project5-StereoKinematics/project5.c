/*************************************************************************/
/* File:        project5.c                                               */
/* Description: User project #5 - empty project directory for project    */
/*              developement                                             */
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
#define NACTIONS 2
#define NSTATES  3    // 3^(NACTIONS)

// only used if using the RL toolkit
#define Q_TABLE_FILE "q_tables/....q_table_%d.txt"
#define CONVG_ACT 0
// end RLToolKit parameters

double proj_five_q_table[NSTATES][NACTIONS] = {0.0};
/**************************************************************/

#define DRAW_BUFFER_SIZE 1
#define DRAW_SKIP_SIZE -1
Observation obuf[DRAW_BUFFER_SIZE];

int stereo_observation(Robot *roger, double time, Observation *obs) {
	int u[NEYES];
	double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4], Jw[2][2], wRb[2][2], JwT[2][2];

	if (!average_red_pixel(roger, u))
		return;

	double gL = roger->eye_theta[LEFT]-atan2(NPIXELS/2 - u[LEFT], FOCAL_LENGTH);
	double gR = roger->eye_theta[RIGHT]-atan2(NPIXELS/2 - u[RIGHT], FOCAL_LENGTH);
	double lL = 2*BASELINE*cos(gR)/sin(gR-gL);

	ref_b[X] = lL*cos(gL);
	ref_b[Y] = BASELINE + lL*sin(gL);
	ref_b[2] = 0;
	ref_b[3] = 1;

	double jc = 2*BASELINE/SQR(sin(gR-gL));
	double Jb[2][2] = {
		{jc*SQR(cos(gR)), -jc*SQR(cos(gL))},
		{jc*sin(gR)*cos(gR), -jc*sin(gL)*cos(gL)}
	};

	construct_wTb(roger->base_position, wTb);
  matrix_mult(4, 4, wTb, 1, ref_b, ref_w);

  for (int i=X; i<=Y; i++)
  	for (int j=X; j<=Y; j++)
  		wRb[i][j] = wTb[i][j];

  matrix_mult(2, 2, wRb, 2, Jb, Jw);

  obs->pos[X] = ref_w[X];
  obs->pos[Y] = ref_w[Y];

  matrix_transpose(2, 2, Jw, JwT);
  matrix_mult(2, 2, Jw, 2, JwT, obs->cov);

  obs->time = time;
}

void project5_control(roger, time)
Robot* roger;
double time;
{
	double errors[NDOF];
	static int obs_counter = 0;
	static int skip_counter = 0;
	Observation obs;

	search_track(roger, errors, time);
	submit_errors(roger, errors);

	stereo_observation(roger, time, &obs);
  matrix_scale(2, 2, obs.cov, 0.01, obs.cov);

  if (skip_counter > DRAW_SKIP_SIZE) {
  	obuf[obs_counter] = obs;
  	obs_counter = (obs_counter+1)%DRAW_BUFFER_SIZE;
  	skip_counter = 0;
  } else
  	skip_counter++;
}

/************************************************************************/
void project5_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project5_enter_params() 
{
  printf("Project 5 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project5_visualize(roger)
Robot* roger;
{
	for (int i=0; i<DRAW_BUFFER_SIZE; i++)
		draw_observation(roger, obuf[i]);
}
