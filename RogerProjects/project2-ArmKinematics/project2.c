/*************************************************************************/
/* File: project2.c                                                      */
/* Description: Kinematic function for Arms                              */
/*                 fwd_arm_kinematics() - maps joint angles in the arm   */
/*                    into endpoint coordinates in the base frame        */
/*                 inv_arm_kinematics() - maps endpoint position for the */
/*                    specified arm in base frame into two possible joint*/
/*                    angle configurations and selects one of the two    */
/*                    writes arm cspace setpoints                        */
/* Date:        1-2015                                                 */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
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
#define Q_TABLE_FILE ""
#define CONVG_ACT 2
// end RLToolKit parameters

double proj_two_q_table[NSTATES][NACTIONS] = {0.0};
/**************************************************************/


/*************************************************************************/
/*** PROJECT #2 - FORWARD KINEMATICS: MAP (THETA1, THETA2) TO (X,Y)_B  ***/
/***              INVERSE KINEMATICS: MAP (X,Y)_B TO (THETA1, THETA2)  ***/
/*************************************************************************/
void fwd_arm_kinematics(roger, limb, x, y)
Robot * roger;
int limb;
double x, y;
{ }

// double sqr(double x) {
//   return x*x;
// }

// submit calculated angles directly to robot-?setpoints structure
int inv_arm_kinematics(roger, limb, x, y)
Robot * roger;
int limb;
double x, y;
{
  double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4];

  double r2, c2, s2_plus, s2_minus, theta2_plus, theta2_minus;
  double k1, k2_plus, k2_minus, alpha_plus, alpha_minus;
  double theta1_plus, theta1_minus;

  // input (x,y) is in world frame coordinates - map it into the base frame
  construct_wTb(roger->base_position, wTb);
  HT_invert(wTb,bTw);

  ref_w[0] = x;
  ref_w[1] = y;
  ref_w[2] = 0.0;
  ref_w[3] = 1.0;

  matrix_mult(4, 4, bTw, 1, ref_w, ref_b);
  if (limb==LEFT) ref_b[Y] -= ARM_OFFSET;
  else ref_b[Y] += ARM_OFFSET;

  r2 = SQR(ref_b[X]) + SQR(ref_b[Y]);
  c2 = (r2-SQR(L_ARM1)-SQR(L_ARM2))/(2*L_ARM1*L_ARM2);

  if (c2 < -1 || c2 > 1)
    return FALSE;

  s2_plus = sqrt(1-SQR(c2));
  theta2_plus = atan2(s2_plus, c2);
  k1 = L_ARM1+L_ARM2*c2;
  k2_plus = L_ARM2*s2_plus;
  alpha_plus = atan2(k2_plus, k1);
  theta1_plus = atan2(ref_b[Y], ref_b[X])-alpha_plus;

  s2_minus = -s2_plus;
  theta2_minus = atan2(s2_minus, c2);
  k2_minus = L_ARM2*s2_minus;
  alpha_minus = atan2(k2_minus, k1);
  theta1_minus = atan2(ref_b[Y], ref_b[X])-alpha_minus;

  double theta1_pos = roger->arm_theta[limb][0];
  double theta2_pos = roger->arm_theta[limb][1];

  double plus_dist = (theta1_plus-theta1_pos)+(theta2_plus-theta2_pos);
  double minus_dist = (theta1_minus-theta1_pos)+(theta2_minus-theta2_pos);

  if (plus_dist < minus_dist) {
    roger->arm_setpoint[limb][0] = theta1_plus;
    roger->arm_setpoint[limb][1] = theta2_plus;
  } else {
    roger->arm_setpoint[limb][0] = theta1_minus;
    roger->arm_setpoint[limb][1] = theta2_minus;
  }

  // printf("x = %6.4lf  y = %6.4lf OUT OF REACH\n", ref_b[X], ref_b[Y]);
  return TRUE;
}


// exact same function as above, but submit differential setpionts via the errors array
//     create the setpoint via errors[x] = target_theta - roger->arm_theta[limb][0 or 1]
//     useful for later hierarchical skill development
int inv_arm_kinematics_errors(roger, limb, x, y, errors)
Robot * roger;
int limb;
double x, y;
double errors[NDOF];
{
  double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4];

  double r2, c2, s2_plus, s2_minus, theta2_plus, theta2_minus;
  double k1, k2_plus, k2_minus, alpha_plus, alpha_minus;
  double theta1_plus, theta1_minus;
  int i;

  // first, do no harm
  	for (i=0; i<NDOF; ++i) {
  		errors[i] = 0.0;
  	}

  // input (x,y) is in world frame coordinates - map it into the base frame
  construct_wTb(roger->base_position, wTb);
  HT_invert(wTb,bTw);

  ref_w[0] = x;
  ref_w[1] = y;
  ref_w[2] = 0.0;
  ref_w[3] = 1.0;

  matrix_mult(4, 4, bTw, 1, ref_w, ref_b);
  if (limb==LEFT) ref_b[Y] -= ARM_OFFSET;
  else ref_b[Y] += ARM_OFFSET;

  // printf("x = %6.4lf  y = %6.4lf OUT OF REACH\n", ref_b[X], ref_b[Y]);
  return FALSE;
}

void add_error_arrays(arr_1, arr_2, out)
double arr_1[NDOF];
double arr_2[NDOF];
double out[NDOF];
{
	int i;
	for(i=0;i<NDOF;++i) {
    	out[i] = arr_1[i] + arr_2[i];    
  	}

}

// LEGEND for delta errors: errors[i]
//          i=0   Base Translate error
//          i=1   Base Rotate error
//          i=2   Left Eye error
//          i=3   Right Eye error
//          i=4   Left Shoulder error
//          i=5   Left Elbow error
//          i=6   Right Shoulder error
//          i=7   Right Elbow error
void submit_errors(roger, errors)
Robot *roger;
double errors[NDOF];
{
  // add the base components
  roger->base_setpoint[0] = roger->base_position[X] + errors[0]*cos(roger->base_position[THETA]);
  roger->base_setpoint[1] = roger->base_position[Y] + errors[0]*sin(roger->base_position[THETA]);
  roger->base_setpoint[THETA] = roger->base_position[THETA] + errors[1];
  
  // add the eye information
  roger->eyes_setpoint[LEFT] = roger->eye_theta[LEFT] + errors[2];
     if (roger->eyes_setpoint[LEFT] > (M_PI/2.0))
       roger->eyes_setpoint[LEFT]=(M_PI/2.0);
     if (roger->eyes_setpoint[LEFT] < (-M_PI/2.0))
       roger->eyes_setpoint[LEFT]=(-M_PI/2.0);
  roger->eyes_setpoint[RIGHT] = roger->eye_theta[RIGHT] + errors[3];
     if (roger->eyes_setpoint[RIGHT] > (M_PI/2.0))
       roger->eyes_setpoint[RIGHT]=(M_PI/2.0);
     if (roger->eyes_setpoint[RIGHT] < (-M_PI/2.0))
       roger->eyes_setpoint[RIGHT]=(-M_PI/2.0);
  
   // arms
  roger->arm_setpoint[LEFT][0] = roger->arm_theta[LEFT][0] + errors[4];
  roger->arm_setpoint[LEFT][1] = roger->arm_theta[LEFT][1] + errors[5];
  roger->arm_setpoint[RIGHT][0] = roger->arm_theta[RIGHT][0] + errors[6];
  roger->arm_setpoint[RIGHT][1] = roger->arm_theta[RIGHT][1] + errors[7];
}

void copy_errors(from, to)
double from[NDOF], to[NDOF];
{
  int i;
  for(i=0;i<NDOF;++i) {
    to[i] = from[i];    
  }
}

void home_arms(roger, errors, time) 
Robot* roger;
double errors[NDOF];
double time;
{
	int i;

	// printf("inside home arms\n");

	errors[4] = 3.05 - roger->arm_theta[0][0];
	errors[5] = (-2.8) - roger->arm_theta[0][1];


  	errors[6] = (-3.05) - roger->arm_theta[1][0];
  	errors[7] = 2.8 - roger->arm_theta[1][1];
}

/*************************************************************************/
/* project development interface - called by GUI                         */
/*************************************************************************/
/* executed automatically when                                           */
/* control mode = PROJECT2; input mode = BALL INPUTS                     */
void project2_control(roger, time)
Robot * roger;
double time;
{
  // two work flows
  // 1) debug the inv_kinematics directly
  //     inv_arm_kinematics(roger, limb, x, y)

  // 2) debug inv_kinematic with errors array
  // double errors[NDOF] = {0.0}
  // inv_arm_kinematics_errors(roger, limb, x, y, errors)
  // submit_errors(roger, errors) 

}


void project2_reset(roger)
Robot* roger;
{ }

void project2_enter_params()
{
  printf("Project 2 enter_params called. \n");
}

void project2_visualize(roger)
Robot* roger;
{ }
