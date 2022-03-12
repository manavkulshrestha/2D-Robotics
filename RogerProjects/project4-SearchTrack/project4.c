/*************************************************************************/
/* File:        project4.c                                               */
/* Description: User project #4 - empty project directory for project    */
/*              developement                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "../include/Xkw/Xkw.h"
#include "../include/roger.h"
#include "../include/simulate.h"
#include "../include/control.h"
#include "../include/modes.h"
#include "../include/RLToolKit.h"

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

// double proj_four_q_table[NSTATES][NACTIONS] = {0.0};
double proj_four_q_table[NSTATES][NACTIONS] = {
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
};
/**************************************************************/

/* 
* Primitive Skill prototype function.
*/
int search(roger, errors, time)
Robot* roger;
double errors[NDOF];
double time;
{
	static int return_status = NO_REFERENCE;
	static double heading = NULL;

	// // first, do no harm
	for (int i=0; i<NDOF; ++i) {
		errors[i] = 0.0;
	}

	for (int i=0; i<NEYES; i++) {
		errors[EYE_ERROR_OFFSET+i] = heading-roger->eye_theta[i];

		if (heading == NULL || abs(errors[EYE_ERROR_OFFSET+i]) < 0.01) {

		}
	}

	return return_status;
}

/* 
* Primitive Skill prototype function.
*/
int track(roger, errors, time)
Robot* roger;
double errors[NDOF];
double time;
{
	static int return_status = NO_REFERENCE;
	// first, do no harm

	for (int i=0; i<NDOF; ++i) {
		errors[i] = 0.0;
	}

	// DO STUFF

	return(return_status);
}


/********************************** Behavorial-Build File Function Prototypes ****************************/
// /* 
// * Learned/Hand-Designed Policy Execution Action Function Prototype
// *     use your learned or hand-designed policy array in this function
// *     for future functions to call.
// */
// int CompositeAction_X(roger, errors, time)
// Robot* roger;
// double errors[NDOF];
// double time;
// {
// 	int i, state, return_state, internal_state[NACTIONS]; 
// 	// double search_errors[NDOF], track_errors[NDOF];
// 	double action_errors[NACTIONS][NDOF];
// 	static int initialized = 0;
// 	int selected_action;

// 	// arrays to hold the action and reward functions (rewards are still tracked for debugging)
// 	static int (*actions[NACTIONS])(Robot* roger, double errors[NDOF], double time);

// 	if (initialized == 0) {
// 		// need to assign actions to the array indicies 
// 		//    NOTE: order matters w.r.t. how state is calculated
// 		actions[0] = Action_X;
// 		actions[1] = ...;
//   		...
//   		// Load the learned q-table - if skill was learned through RL....
// 		LoadLearnedQTable(..., proj_X_q_table, NSTATES, NACTIONS, Q_TABLE_FILE);
     
//   		// ... OR - define the q-table by hand
//      	proj_X_q_table[0][0] = 1.0;
// 		proj_X_q_table[1][1] = 1.0;
// 		proj_X_q_table[2][0] = 1.0;
//      	...
//      	total_reward_exploit = 0.0;
// 		initialized = 1;
// 	}
	
// 	// first do no harm
// 	for (i=0; i<NDOF; ++i) {
// 		errors[i] = 0.0;
// 	}

// 	// calculate the current state based on the avaliable actions
// 	state = 0;
//     // create a defualt set of action parameters
// 	for (i=0; i<NACTIONS; ++i) {
// 		internal_state[i] = actions[i](roger, action_errors[i], time);
// 		state += pow(3, i)*internal_state[i];
// 	}

// 	// get the greedy (largest q-value) action to perform based on the q-table
// 	selected_action = GetActionGreedy(state, proj_..._q_table, NACTIONS);

// 	copy_errors(action_errors[selected_action], errors);

// 	// handle setting the return state
//     // TODO: define your own skill's return status based on the state
// 	if (state >= 1 && ...) {
// 		return(TRANSIENT); 
// 	} else if (...) {
// 		return(CONVERGED);
// 	}
// 	... 
// 	return(NO_REFERENCE);
// }
/********************************************************************************************************/


// /* 
// * Primitive Skill prototype function.
// */
// int Action_X(roger, errors, time)
// Robot* roger;
// double errors[NDOF];
// double time;
// {
// 	int i;
// 	static int return_status = NO_REFERENCE;
// 	// first, do no harm
//   	for (i=0; i<NDOF; ++i) {
//   		errors[i] = 0.0;
//   	}
// 	// DO STUFF
// 	return(return_status);
// }

void project4_control(roger, time)
Robot* roger;
double time;
{ 
	/******** Code outline for testing primitive actions ********/
	// double errors[NDOF];

	// Action_X(roger, errors, time);
	// submit_errors(roger, errors);
	/************************************************************/


	/******** Code outline for testing HAND-DESIGNED composite actions ********/
	// double errors[NDOF];

	// CompositeAction_X(roger, errors, time);
	// submit_errors(roger, errors);
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
void project4_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project4_enter_params() 
{
  printf("Project 4 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project4_visualize(roger)
Robot* roger;
{ }