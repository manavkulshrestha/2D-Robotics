/*************************************************************************/
/* File:        project6.c                                               */
/* Description: User project #6 - observe, filter, and visualize         */
/*                                (no motor control)                     */
/* Date:        6-04-2013                                                */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "matrix_math.h"
#include "RLToolKit.h"

/***********************************************************************/
/* default parameters for the state-action value table used to compose */
/* skills as sequential decision to engage primative actions           */
/*    all actions/skills return status values in {NOREF, !CONV, COV}   */
/*    where NOREF = 0, !CONV=1, CONV=2                                 */
#define NACTIONS 1
#define NSTATES  3    // 3^(NACTIONS)

// only used if using the RL toolkit
#define Q_TABLE_FILE "q_tables/....q_table_%d.txt"
#define CONVG_ACT 0
// end RLToolKit parameters

double proj_six_q_table[NSTATES][NACTIONS] = {0.0};
/**************************************************************/

void project6_control(roger, time)
Robot* roger;
double time;
{ }

/************************************************************************/
void project6_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project6_enter_params() 
{
  printf("Project 6 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project6_visualize(roger)
Robot* roger;
{ }
