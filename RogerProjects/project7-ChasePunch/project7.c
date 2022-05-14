/*************************************************************************/
/* File:        project7.c                                               */
/* Description: User project #7 - empty project directory for project    */
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
#define NACTIONS 2
#define NSTATES  9    // 3^(NACTIONS)

// only used if using the RL toolkit
#define Q_TABLE_FILE "q_tables/....q_table_%d.txt"
#define CONVG_ACT 0
// end RLToolKit parameters

double proj_seven_q_table[NSTATES][NACTIONS] = {0.0};
double proj_seven_q_table2[NSTATES][NACTIONS] = {0.0};
/**************************************************************/

#define BASETRA_ERROR_OFFSET 0
#define BASEROT_ERROR_OFFSET 1

#define LEFTSHO_ERROR_OFFSET 4
#define RIGHTSH_ERROR_OFFSET 6

#define DIST2(x1, y1, x2, y2) sqrt(SQR((x1)-(x2))+SQR((y1)-(y2)))

int stereo_observation(), fwd_arm_kinematics(), inv_arm_kinematics(), inv_arm_kinematics_errors();
void construct_wTb(), matrix_mult(), copy_errors();
void submit_errors(), add_error_arrays();
int search_track();


// double distance2(double x1, double y1, double x2, double y2) {
// 	return sqrt(SQR(x1-x2)+SQR(y1-y2));
// }

int subarreq(double *arr1, double *arr2, int start, int end, double tol) {
	for (int i=start; i<=end; i++)
		if (fabs(arr1[i]-arr2[i]) > tol)
			return FALSE;

	return TRUE;
}

int chase(roger, errors, time)
Robot* roger;
double errors[NDOF];
double time;
{
	static int return_status = NO_REFERENCE;
	// first, do no harm
	for (int i=0; i<NDOF; ++i) {
		errors[i] = 0.0;
	}

	Observation obs;
	if (!stereo_observation(roger, time, &obs))
		return (return_status = NO_REFERENCE);

	// double d_errors[NDOF];
	// if (search_track(roger, d_errors, time) != CONVERGED)
	// 	return (return_status = NO_REFERENCE);

	// double aerrors[2];
	// aerrors[X] = obs.pos[X] - roger->base_position[X];
	// aerrors[Y] = obs.pos[Y] - roger->base_position[Y];

	return_status = TRANSIENT;
	errors[0] = DIST2(obs.pos[X], obs.pos[Y], roger->base_position[X], roger->base_position[Y]);
	// errors[1] = atan2(aerrors[Y], aerrors[X]);

	if (errors[0] < 0.8) {
		errors[0] = 0;
		return_status = CONVERGED;
	}

	return return_status;
}

int touch(roger, errors, time)
Robot* roger;
double errors[NDOF];
double time;
{
	static int return_status = NO_REFERENCE;
	// first, do no harm
	for (int i=0; i<NDOF; ++i) {
		errors[i] = 0.0;
	}

	Observation obs;
	if (!stereo_observation(roger, time, &obs))
		return (return_status = NO_REFERENCE);
 
	double aerrors[NARMS][NDOF] = {0};

	int can_reach = FALSE;
	for (int limb=0; limb<NARMS; limb++)
		if (can_reach = (inv_arm_kinematics_errors(roger, limb, obs.pos[X], obs.pos[Y], aerrors[limb]) | can_reach))
			return_status = TRANSIENT;

	if (!can_reach)
		return_status = NO_REFERENCE;

	add_error_arrays(aerrors[0], aerrors[1], errors);

	for (int limb=0; limb<NARMS; limb++) {
		for (int fi=X; fi<=Y; fi++) {

			double arms_b[NARMS][4], arms_w[NARMS][4], wTb[4][4];
			arms_b[LEFT][2] = 0;
			arms_b[LEFT][3] = 1;
			arms_b[RIGHT][2] = 0;
			arms_b[RIGHT][3] = 1; 
			construct_wTb(roger->base_position, wTb);

			fwd_arm_kinematics(roger, limb, &arms_b[limb][X], &arms_b[limb][Y]);
			matrix_mult(4, 4, wTb, 1, arms_b[limb], arms_w[limb]);

			double hand_ball_dist = DIST2(arms_w[limb][X], arms_w[limb][Y], obs.pos[X], obs.pos[Y]);

			if (roger->ext_force[limb][fi] > 0 && hand_ball_dist < 2*(R_BALL+R_JOINT)) {//experimentally determined
				return (return_status = CONVERGED);
			}
		}
	}

	return return_status;
}

int chase_touch(roger, errors, time)
Robot* roger;
double errors[NDOF];
double time;
{
	static int return_status = NO_REFERENCE;

	for (int i=0; i<NDOF; i++)
		errors[i] = 0;

	double chase_errors[NDOF], touch_errors[NDOF], st_errors[NDOF];

	double st_status = search_track(roger, st_errors, time);
	double chase_status = chase(roger, chase_errors, time);
	double touch_status = touch(roger, touch_errors, time);

	add_error_arrays(st_errors, touch_errors, errors);

// a = 0.001,0.2. k = 0.8,.999

	// printf("[%f]\nST: %d\n, C :%d\n, T :%d\n\n", time, st_status, chase_status, touch_status)
	if (st_status == CONVERGED && touch_status != TRANSIENT)
		add_error_arrays(chase_errors, errors, errors);

	if (touch_status == NO_REFERENCE) {
		// printf("[%f] Homing arms\n", time);
		double home_arm_errors[NDOF];
		home_arms(roger, home_arm_errors, time);
		for (int i=4; i<=7; i++)
			errors[i] = home_arm_errors[i];
	}

	// printf("ball placed x=%f, y=%f\n", roger->button_reference[X], roger->button_reference[Y]);

	return return_status = (touch_status == CONVERGED ? CONVERGED : TRANSIENT);
}

/********************************************************************************************************/

void project7_control(roger, time)
Robot* roger;
double time;
{
	/******** Code outline for testing primitive actions ********/
	// double errors[NDOF];

	// Action_X(roger, errors, time);
	// submit_errors(roger, errors);
	/************************************************************/


	/******** Code outline for testing HAND-DESIGNED composite actions ********/
	double errors[NDOF];

	chase_touch(roger, errors, time);
	// printf("%d\n", chase_touch(roger, errors, time));
	// printf("[%f,%f,%f,%f,%f,%f,%f,%f]\n", errors[0], errors[1], errors[2], errors[3], errors[4], errors[5], errors[6], errors[7]);
	submit_errors(roger, errors);

	// LOGGING FOR GRAPH
	Observation obs;
	double base_ball_dist = 0, hand_ball_dist[NARMS] = {0};
	if (stereo_observation(roger, time, &obs)) {
		base_ball_dist = DIST2(roger->base_position[X], roger->base_position[Y], obs.pos[X], obs.pos[Y]);

		double arms_b[NARMS][4], arms_w[NARMS][4], wTb[4][4];
		arms_b[LEFT][2] = 0;
		arms_b[LEFT][3] = 1;
		arms_b[RIGHT][2] = 0;
		arms_b[RIGHT][3] = 1; 
		construct_wTb(roger->base_position, wTb);

		for (int limb=0; limb<NARMS; limb++) {
			fwd_arm_kinematics(roger, limb, &arms_b[limb][X], &arms_b[limb][Y]);
  		matrix_mult(4, 4, wTb, 1, arms_b[limb], arms_w[limb]);
  		hand_ball_dist[limb] = DIST2(arms_w[limb][X], arms_w[limb][Y], obs.pos[X], obs.pos[Y]);
		}
	}

	// FILE *fptr = fopen("p7.txt", "a");
	// fprintf(fptr, "%f %f %f %f\n", roger->base_position[THETA], base_ball_dist, hand_ball_dist[0], hand_ball_dist[1]);

	// fclose(fptr);

	/************************************************************/

	/******** Code outline for LEARNING/TESTING composite actions ********/
	// NOTE: See programming exercises document, section 7, for detailed explanation of below function and
	//     example reward anf reset functions
	// int (*actions[NACTIONS])(Robot* roger, double errors[NDOF], double time);
    // double (*rewards[...])(int state, int previous_state, int previous_action, int internal_state[NSTATES]);
    // double alpha=..., gamma=...;
    // int reward_num = ...;
    // double default_reward = ...;
	// double conv_reward = 100;
	// double conv_penalty = -100;             

    // // important note, final eps values is the SUM of eps_min and eps_max
	// //     i.e. eps_min = 0.2 and eps_max = 0.6 results in a final eps value of 0.8
	// //     eps is the probability of a greedy action
    // double eps_min = ..., eps_max = ...;
    // int time_per_episode = ; // ms
    // int num_episodes = ; 

    // // learn a policy
	// // NOTE, DOES NOT construct empty folder inside of q_tables/. If you want to put data in a sub-folder
	// //     create the sub-folder first!	
	// Q_Learning(roger, time, alpha, gamma, actions, rewards, default_reward, NACTIONS, NSTATES, 
	// 	reward_num, proj_X_q_table, time_per_episode, num_episodes, eps_min, eps_max, CONVG_ACT, conv_reward, conv_penalty, Q_TABLE_FILE,
	// 	..reset_func..);
	/************************************************************/
	
    /******** Code outline for loading and testing previously learned composite actions ********/
  	// static int policy_loaded = 0;
	// int q_table_num = ...;
	
    // // load a learned policy
    // if (policy_loaded == 0) {
	// 	LoadLearnedQTable(q_table_num, proj_X_q_table, NSTATES, NACTIONS, Q_TABLE_FILE);
	// 	policy_loaded = 1;
	// }

    // // exploit the learned policy
	// Run_Policy(roger, time, actions, rewards, default_reward, NACTIONS, NSTATES, reward_num, proj_X_q_table);

	// alternatively, you can load the learned file into the above CompositeAction_X template, create the approperiatre action/reward arrays
	//     and run the action is displayed above for HAND-DESIGNED composite actions
	/************************************************************/

	/******** Code outline for collecting performance data for learned composite actions ********/
	// static int action_calls[NACTIONS];

	// // load a learned policy
    // if (policy_loaded == 0) {
	// 	LoadLearnedQTable(q_table_num, proj_X_q_table, NSTATES, NACTIONS, Q_TABLE_FILE);
	// 	policy_loaded = 1;
	// }

	// // NOTE, DOES NOT construct empty folder inside of perf_data. If you want to pput data in a sub-folder
	// //     create the sub-folder first!
	// generatePerformanceData(roger, time, actions, rewards, default_reward, NACTIONS, NSTATES, reward_num, proj_X_q_table,
	// "perf_data/TODO/TODO_%s", action_calls, 100, CONVG_ACT, -100, 100, time_per_episode, reset_X); 

	/************************************************************/
}

/************************************************************************/
void project7_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project7_enter_params() 
{
  printf("Project 7 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project7_visualize(roger)
Robot* roger;
{ }