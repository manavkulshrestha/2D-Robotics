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

#define DIST(X,Y) sqrt(SQR(X)+SQR(Y))

int subarreq(double *arr1, double *arr2, int start, int end, double tol) {
	for (int i=start; i<=end; i++)
		if (fabs(arr1[i]-arr2[i]) > tol)
			return FALSE;

	return TRUE;
}

int search_track2(Robot *roger, double errors[NDOF], double time) {
	return search_track(roger, time, errors);
}

int approach(roger, errors, time)
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

	double aerrors[2];
	aerrors[X] = obs.pos[X] - roger->base_position[X];
	aerrors[Y] = obs.pos[Y] - roger->base_position[Y];

	return_status = TRANSIENT;
	errors[0] = DIST(aerrors[X], aerrors[Y]);
	errors[1] = atan2(aerrors[Y], aerrors[X]);

	if (subarreq(roger->base_position, obs.pos, X, Y, 0.01))
		return_status = CONVERGED;

	return return_status;
}

int chase(roger, errors, time)
Robot* roger;
double errors[NDOF];
double time;
{
	int state, return_state, internal_state[NACTIONS]; 
	// double search_errors[NDOF], track_errors[NDOF];
	double action_errors[NACTIONS][NDOF];
	static int initialized = 0;
	int selected_action;

	// arrays to hold the action and reward functions (rewards are still tracked for debugging)
	static int (*actions[NACTIONS])(Robot* roger, double errors[NDOF], double time);

	if (initialized == 0) {
		actions[0] = search_track2;
		actions[1] = approach;
     
    proj_seven_q_table[0][0] = 1;
    proj_seven_q_table[1][0] = 1;
    proj_seven_q_table[2][0] = 1;
    proj_seven_q_table[3][0] = 1;
    proj_seven_q_table[4][0] = 1;
    proj_seven_q_table[5][1] = 1;
    proj_seven_q_table[6][1] = 1;
    proj_seven_q_table[7][1] = 1;
    proj_seven_q_table[8][1] = 1;
		initialized = 1;
	}
	
	// first do no harm
	for (int i=0; i<NDOF; ++i) {
		errors[i] = 0.0;
	}

	// calculate the current state based on the avaliable actions
	state = 0;
	for (int i=0; i<NACTIONS; ++i) {
		internal_state[i] = actions[i](roger, action_errors[i], time);
		state += pow(3, i)*internal_state[i];
	}

	// get the greedy (largest q-value) action to perform based on the q-table
	selected_action = GetActionGreedy(state, proj_seven_q_table, NACTIONS);
	copy_errors(action_errors[selected_action], errors);

	// handle setting the return state
	if (state == 8)
		return CONVERGED;

	return state ? TRANSIENT : NO_REFERENCE;
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
		return;

	int limb;
	for (limb=0; limb<NARMS; limb++)
		if (inv_arm_kinematics_errors(roger, limb, obs.pos[X], obs.pos[Y], errors))
			break;

	return_status = (limb == NARMS) ? NO_REFERENCE : TRANSIENT;

	if (roger->ext_force[limb][0] > 0 || roger->ext_force[limb][1] > 0)
		return_status = CONVERGED;

	return return_status;
}

int chase_touch(roger, errors, time)
Robot* roger;
double errors[NDOF];
double time;
{
	int state, return_state, internal_state[NACTIONS];
	// double search_errors[NDOF], track_errors[NDOF];
	double action_errors[NACTIONS][NDOF];
	static int initialized = 0;
	int selected_action;

	// arrays to hold the action and reward functions (rewards are still tracked for debugging)
	static int (*actions[NACTIONS])(Robot* roger, double errors[NDOF], double time);

	if (initialized == 0) {
		// need to assign actions to the array indicies 
		//    NOTE: order matters w.r.t. how state is calculated
		actions[0] = chase;
		actions[1] = touch;

		// ... OR - define the q-table by hand
   	proj_seven_q_table2[0][0] = 1;
		proj_seven_q_table2[1][0] = 1;
		proj_seven_q_table2[2][0] = 1;
		proj_seven_q_table2[3][0] = 1;
		proj_seven_q_table2[4][1] = 1;
		proj_seven_q_table2[5][1] = 1;
		proj_seven_q_table2[6][1] = 1;
		proj_seven_q_table2[7][1] = 1;
		proj_seven_q_table2[8][1] = 1;
		initialized = 1;
	}
	
	// first do no harm
	for (int i=0; i<NDOF; ++i) {
		errors[i] = 0.0;
	}

	// calculate the current state based on the avaliable actions
	state = 0;
    // create a defualt set of action parameters
	for (int i=0; i<NACTIONS; ++i) {
		internal_state[i] = actions[i](roger, action_errors[i], time);
		state += pow(3, i)*internal_state[i];
	}

	printf("internal state: [%d, %d]\n", internal_state[0], internal_state[1]);
	printf("state: %d\n", state);


	// get the greedy (largest q-value) action to perform based on the q-table
	selected_action = GetActionGreedy(state, proj_seven_q_table2, NACTIONS);
	copy_errors(action_errors[selected_action], errors);

	// handle setting the return state
  if (state == 8)
		return CONVERGED;

	return state ? TRANSIENT : NO_REFERENCE;
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
	// printf("[%f,%f,%f,%f,%f,%f,%f,%f]\n", errors[0], errors[1], errors[2], errors[3], errors[4], errors[5], errors[6], errors[7]);
	submit_errors(roger, errors);
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