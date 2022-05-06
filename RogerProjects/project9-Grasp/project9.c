/*************************************************************************/
/* File:        project9.c                                               */
/* Description: User project #9 - GRASP                                  */
/* Date:        12-2014                                                  */
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
#define NACTIONS 3
#define NSTATES  27    // 3^(NACTIONS)

// only used if using the RL toolkit
//   IMPORTANT TIP!!!! consider making sub-folders for different experiments, 
//                     but be sure to MAKE the folder in the directory first 
//   MUST end with %d.txt
#define Q_TABLE_FILE "q_tables/q_table_%d.txt"
#define CONVG_ACT 0
// end RLToolKit parameters

double proj_nine_q_table[NSTATES][NACTIONS] = {0.0};
/**************************************************************/

// declare necessary functions
// !!!! TODO !!!!! declare any functions you use/define here!!!
int stereo_observation(), fwd_arm_kinematics(), inv_arm_kinematics(), inv_arm_kinematics_errors();
void construct_wTb(), matrix_mult(), copy_errors();
void submit_errors(), add_error_arrays();
void LoadLearnedPolicy();
void Q_Learning(), Run_Policy();
double drand();
void random_location();
int reset_ball_training();
void LoadLearnedQTable();

int search_track(), track(), chase_touch();

int VFClosure(), TFClosure();

void pseudoinverse();

// variable for FClosure procedures
//    declare at global level to use in reward functions
double phi_wt;
double phi_wv;


// Observation used by FClosure functions
Observation obs;

// use uniquely named global variables from OTHER c files like this
//     (external variables can be used for reward functions if you want)
// extern (var type) var name;


/********************************** Behavioral-Build File Function Prototypes ****************************/
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
//     // create a default set of action parameters
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

/* 
* Function to Grasp ball with three frictionless contact points
*
*/
int TFClosure(roger, errors, time)
Robot * roger;
double errors[NDOF];
double time;
{
	int return_state = NO_REFERENCE;
	double left[2], right[2], base[2];
    double st_errors[NDOF], chase_errors[NDOF];
    double larm_errors[NDOF], rarm_errors[NDOF];
	int obs_result = 0;
	int i;
	double wTb[4][4] = {0.0}, ref_bl[4] = {0.0}, ref_wl[4] = {0.0}, ref_br[4] = {0.0}, ref_wr[4] = {0.0};
	double lx = 0, ly = 0, rx = 0, ry = 0;
	double left_arm_forces[2], right_arm_forces[2], base_forces[2], estimated_theta[3];

	// first do no harm
	for (i=0; i<NDOF; ++i) {
		errors[i] = 0.0;
	}

	// get the estimated position of the ball (in world coordinates)
	//    if your implementation doesn't work, try figuring it out on your own first
	//        and if you can't email Oscar
	obs_result = stereo_observation(roger, &obs);

	// if we see the ball and all contacts are touching the ball and trying to form a FClosure Grasp on the ball
	if ((fabs(left_arm_forces[0]) > 0 || fabs(left_arm_forces[1]) > 0)
			&& (fabs(right_arm_forces[0]) > 0 || fabs(right_arm_forces[1]) > 0)
			&& (fabs(base_forces[0]) > 0 || fabs(base_forces[1]) > 0) 
			&& obs_result) {
		
		// set the return state to transient if we see the ball and are making contact
		return_state = TRANSIENT;

		// assign the moving average to the value used in the fclosure calculation
		left_arm_forces[0] = roger->ext_force[LEFT][0];
		left_arm_forces[1] = roger->ext_force[LEFT][1];
		right_arm_forces[0] = roger->ext_force[RIGHT][0];
		right_arm_forces[1] = roger->ext_force[RIGHT][1];
		base_forces[0] = roger->ext_force_body[0];
		base_forces[1] = roger->ext_force_body[1];

		// Dr. Grupen mentioned in class that you can do all these calculation in base-frame
		//     bu tmy solution has always just used world, you can do whatever
		
		// construct_wTb(roger->base_position, wTb);
		construct_wTb(roger->base_position, wTb);
		fwd_arm_kinematics(roger, LEFT, &lx, &ly);
		ly += ARM_OFFSET;
		ref_bl[0] = lx;
		ref_bl[1] = ly;
		ref_bl[2] = 0.0;
		ref_bl[3] = 1.0;
		matrix_mult(4, 4, wTb, 1, ref_bl, ref_wl);

		// construct_wTb(roger->base_position, wTb);
		fwd_arm_kinematics(roger, RIGHT, &rx, &ry);
		ry -= ARM_OFFSET;
		ref_br[0] = rx;
		ref_br[1] = ry;
		ref_br[2] = 0.0;
		ref_br[3] = 1.0;
		matrix_mult(4, 4, wTb, 1, ref_br, ref_wr);

		// estimate theta_0, add pi to rotate from sensor to world frame
		// estimate based on contact forces
		estimated_theta[0] = M_PI + atan2(base_forces[1], base_forces[0]);

		// estimate theta_1, add pi to rotate from sensor to world frame
		// estimate based on contact forces
		estimated_theta[1] =  M_PI + atan2(left_arm_forces[1], left_arm_forces[0]);

		// estimate theta_2, add pi to rotate from sensor to world frame
		// estimate based on contact forces
		estimated_theta[2] =  M_PI + atan2(right_arm_forces[1], right_arm_forces[0]);

		// variables needed for navigation function/error calculation
		double j[2], del_theta[2], kappa;
		double new_theta[2], l_pos[2], r_pos[2];
		double j_p_inv[2] = {0.0};
		double j_fm[2] = {0.0};

		//using diff temp for thetas
		double t[3];
		for (int i=0; i<2; i++)
			t[i] = estimated_theta[i];

		// set te kappa, gradient descent step-size
		kappa = 0.8;

		// the closed form solutions to the navigation function used
		//     to control arm contacts
		phi_wt = 2*cos(t[0]-t[1])+2*cos(t[0]-t[2])+2*cos(t[1]-t[2])+3;

		// calculate the control jacobian used in calculations
		j[0] = 2*sin(t[0]-t[1])-2*sin(t[1]-t[2]);
		j[1] = 2*sin(t[0]-t[2])+2*sin(t[1]-t[2]);

		// get the Penrose pusedo-inverse of the jacobian
		double jpinv[2][1];
		pseudoinverse(1, 2, j, jpinv);
		j_p_inv[0] = jpinv[0][0];
		j_p_inv[1] = jpinv[1][0];
		
		// calculate the change in angle value needed for control (perform gradient descent)
		del_theta[0] = -kappa*j_p_inv[0]*phi_wt;
		del_theta[1] = -kappa*j_p_inv[1]*phi_wt;

		// get the new value of the desired angle based on the del_theta update
		new_theta[0] = estimated_theta[1] + del_theta[0];
		new_theta[1] = estimated_theta[2] + del_theta[1];

		// Radius of the ball, slightly smaller so that we try to "squeeze" the ball some
		double R = 0.18;

		// updates to the left hand's desired location
		left[0] = obs.pos[0] + R*cos(new_theta[0]);
		left[1] = obs.pos[1] + R*sin(new_theta[0]);
		right[0] = obs.pos[0] + R*cos(new_theta[1]);
		right[1] = obs.pos[1] + R*sin(new_theta[1]);

		// state will be converged if all three coSntact points are touching AND
		//     the navigation function is below a threshold.
		//     you can play with the threshold value below, 0.1 is arbirtary 
		if (phi_wt < 0.1) { // you can mess with this threshold value
			return_state = CONVERGED;
		}

	} // end if (observation == True) block, i.e. roger does not see the ball 
	else {
		return_state = NO_REFERENCE;
	}

	// check if the state is not no_reference
	if (return_state != NO_REFERENCE) {
		double dummy_errors[NDOF];

		// set arms
		//    if your implementation doesn't work, try figuring it out on your own first
		//        and if you can't email Oscar
		int l_in_range = inv_arm_kinematics_errors(roger, LEFT, left[0], left[1], larm_errors);
		int r_in_range = inv_arm_kinematics_errors(roger, RIGHT, right[0], right[1], rarm_errors);

		// if the arms are out of range don't try to move them
		//    you can remove this check if you want to try that
		if (l_in_range == FALSE || r_in_range == FALSE) {
			return_state = NO_REFERENCE;
		} else {
			// add the desired arm setpoints to errors array
			add_error_arrays(larm_errors, rarm_errors, dummy_errors);
			copy_errors(dummy_errors, errors);
		}

    	// neither of these rely on additional params
		//    neither does this function, so pass the params
		//    struct we are given, assuming it is empty
		//    IMPORTANT!! BE sure to declare you function at the top of the file
		// TODO-ADD your search-track function here SearchTrack(roger, st_errors, time, params);
		search_track(roger, st_errors, time);


		// ST update to keep Roger's gaze centered on the ball
		// adjust base theta and eye thetas
		errors[1] = st_errors[1];
		errors[2] = st_errors[2];
		errors[3] = st_errors[3];
		
	} // end of transient/converged if statement
	else {
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!! TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// home the arms if not transient or converged
		// TODO put code to home arms here (I used a standalone function to submit
		//    arm errors for a predefine "home" position)
		double home_arms_errors[NDOF];
		home_arms(roger, home_arms_errors, time);
		for (int t=4; i<=7; i++)
			errors[i] = home_arms_errors[i];

	}

	return return_state;
}


/* 
* Function to Grasp ball with three frictionless contact points
*
*/
int VFClosure(roger, errors, time)
Robot * roger;
double errors[NDOF];
double time;
{
	int return_state = NO_REFERENCE;
	double left[2], right[2], base[2];
    double st_errors[NDOF], chase_errors[NDOF];
    double larm_errors[NDOF], rarm_errors[NDOF];
	int obs_result = 0;
	int i;
	double wTb[4][4] = {0.0}, ref_bl[4] = {0.0}, ref_wl[4] = {0.0}, ref_br[4] = {0.0}, ref_wr[4] = {0.0};
	double lx = 0, ly = 0, rx = 0, ry = 0;
	double estimated_theta[3];

	// first do no harm
	for (i=0; i<NDOF; ++i) {
		errors[i] = 0.0;
	}

	// get the estimated position of the ball (in world coordinates)
	//    if your implementation doesn't work, try figuring it out on your own first
	//        and if you can't email Oscar
	obs_result = stereo_observation(roger, &obs);

	// if we see the ball try to form a FClosure Grasp on the ball
	if (obs_result) {
		return_state = TRANSIENT;

		// Dr. Grupen mentioned in class that you can do all these calculation in base-frame
		//     bu tmy solution has always just used world, you can do whatever

		// estimate theta_1
		construct_wTb(roger->base_position, wTb);
		fwd_arm_kinematics(roger, LEFT, &lx, &ly);
		ly += ARM_OFFSET;
		ref_bl[0] = lx;
		ref_bl[1] = ly;
		ref_bl[2] = 0.0;
		ref_bl[3] = 1.0;
		matrix_mult(4, 4, wTb, 1, ref_bl, ref_wl);

		// estimate theta_2
		construct_wTb(roger->base_position, wTb);
		fwd_arm_kinematics(roger, RIGHT, &rx, &ry);
		ry -= ARM_OFFSET;
		ref_br[0] = rx;
		ref_br[1] = ry;
		ref_br[2] = 0.0;
		ref_br[3] = 1.0;
		matrix_mult(4, 4, wTb, 1, ref_br, ref_wr);

		// estimate theta 0 visually based on Roger's position in the world frame
		estimated_theta[0] =  M_PI + atan2(obs.pos[1] - roger->base_position[1], obs.pos[0] - roger->base_position[0]);

		// estimate theta 1 visually based on Roger's position in the world frame
		estimated_theta[1] =  M_PI + atan2(obs.pos[1]-ref_wl[1], obs.pos[0]-ref_wl[0]);

		// estimate theta 2 visually based on Roger's position in the world frame
		estimated_theta[2] =  M_PI + atan2(obs.pos[1]-ref_wr[1], obs.pos[0]-ref_wr[0]);

		// variables needed for navigation function/error calculation
		double j[2], del_theta[2], kappa;
		double new_theta[2], l_pos[2], r_pos[2];
		double j_p_inv[2] = {0.0};
		double j_fm[2] = {0.0};
		
		// set te kappa, gradient descent step-size
		kappa = 0.8;

		//using diff temp for thetas
		double t[3];
		for (int i=0; i<2; i++)
			t[i] = estimated_theta[i];

		// the closed form solutions to the navigation function used
		//     to control arm contacts
		phi_wv = 2*cos(t[0]-t[1])+2*cos(t[0]-t[2])+2*cos(t[1]-t[2])+3;

		// calculate the control jacobian used in calculations
		j[0] = 2*sin(t[0]-t[1])-2*sin(t[1]-t[2]);
		j[1] = 2*sin(t[0]-t[2])+2*sin(t[1]-t[2]);

		// get the Penrose pusedo-inverse of the jacobian
		double jpinv[2][1];
		pseudoinverse(1, 2, j, jpinv);
		j_p_inv[0] = jpinv[0][0];
		j_p_inv[1] = jpinv[1][0];
		
		// calculate the change in angle value needed for control (perform gradient descent)
		del_theta[0] = -kappa*j_p_inv[0]*phi_wv;
		del_theta[1] = -kappa*j_p_inv[1]*phi_wv;
		
		// get the new value of the desired angle based on the del_theta update
		new_theta[0] = estimated_theta[1] + del_theta[0];
		new_theta[1] = estimated_theta[2] + del_theta[1];

		// Radius of the ball, slightly larger than the ball + rogers hand to avoid hitting it 
		//    while pre-shaping
		double R = R_BALL + R_TACTILE + 0.1;

		// updates to the left hand's desired location
		left[0] = obs.pos[0] + R*cos(new_theta[0]);
		left[1] = obs.pos[1] + R*sin(new_theta[0]);

		// updates to the right hand's desired location
		right[0] = obs.pos[0] + R*cos(new_theta[1]);
		right[1] = obs.pos[1] + R*sin(new_theta[1]);

		// state will be converged if all three contact points are touching AND
		//     the navigation function is below a threshold.
		//     you can play with the threshold value below, 1.0 is arbirtary
		//         will need to be much larger than in the tactile version  
		if (phi_wv < 1.0) {
			return_state = CONVERGED;
		}

	} // end if (observation == True) block 

	// check if the state is not no_reference
	if (return_state != NO_REFERENCE) {
		double dummy_errors[NDOF];
		
		// TODO --- add your own eye tracking/foveation function here!!!
		//    IMPORTANT!! BE sure to declare you function at the top of the file
		int track_status = track(roger, dummy_errors, time);

		// set arms
		//    if your implementation doesn't work, try figuring it out on your own first
		//        and if you can't email Oscar
		int l_in_range = inv_arm_kinematics_errors(roger, LEFT, left[0], left[1], larm_errors);
		int r_in_range = inv_arm_kinematics_errors(roger, RIGHT, right[0], right[1], rarm_errors);


		// if the arms are out of range don't try to move them
		//    you can remove this check if you want to try that
		if (l_in_range == FALSE || r_in_range == FALSE) {
			return_state = NO_REFERENCE;
		} else {
			// add the desired arm setpoints to errors array
			if (track_status == CONVERGED) {
				add_error_arrays(larm_errors, rarm_errors, dummy_errors);
				copy_errors(dummy_errors, errors);
			}
		}
	} // end of transient/converged if statement
	else {
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!! TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// home the arms if not transient or converged
		// TODO put code to home arms here (I used a standalone function to submit
		//    arm errors for a predefine "home" position)
		double home_arms_errors[NDOF];
		home_arms(roger, home_arms_errors, time);
		for (int t=4; i<=7; i++)
			errors[i] = home_arms_errors[i];
	}
	return return_state;
	
}

/****************** Standardized Reset Function Definition Outline ******************/

/* 
*  Automatic Env. Reset Function: Randomly places a red ball in
*       the world when CONVG_ACT - or - the episode timer maxes out.
*     	Called at every time-step
*/
int reset_ball_training(roger, time, state, internal_state,
	previous_action, counter, episode_len)
Robot* roger;
double time;
int state;                              // combinatorial state of used actions
int internal_state[NACTIONS];           // state of action in RL problem
int counter;                            // current OVERALL timestep in training (ms)
int previous_action;                    // action called in previous time-step
int episode_len;                        // max episode length (ms)
{
	double roger_x, roger_y;            // current x, y position of Rogers base
	double new_x, new_y;                // new x, y location of the red-ball
	double ball_distance = 0.0;         // distance between Roger and ball
	static int reset_flag = 0;          // reset book-keeping variables per episode
	static int converged_counter = 0;   // count how long CONVG_ACT is converged
	

	// can include more complex behavior here, depending on your task
	if (reset_flag == TRUE) {
		reset_flag = FALSE;
		converged_counter = 0;
	}

	// get Roger's current locations
	roger_x = roger->base_position[X];
	roger_y = roger->base_position[Y];
	
	// check if the internal state of CONVG_ACT is converged OR
	//     we have reached the max number of actions per episode
	if ((internal_state[CONVG_ACT] == CONVERGED && previous_action ==
		CONVG_ACT) || (counter % episode_len == 0 && counter != 0)) {
		
		converged_counter += 1;      // increment counter

		// if converged long enough or we reach the max time per episode
		if (converged_counter > 500 || (counter % episode_len == 0 && counter != 0)) {
			random_location((R_BALL + 0.75), &new_x, &new_y); // get a random location in the enviornment
			// you can play with the wall-offset value above
			
			// calculate distance from new ball center to Roger center
			ball_distance = sqrt(SQR(new_x - roger_x)+SQR(new_y - roger_y));

			// keep selecting new locations until we pick one that is not going to hit Roger
			while (ball_distance < (R_BALL + 1.2)) {     // you can play with this value
				random_location((R_BALL + 0.75), &new_x, &new_y);			
				ball_distance = sqrt(SQR(new_x - roger_x)+SQR(new_y - roger_y));
			}

			// we have got the new location, now to set the Roger data-structure accordingly
			roger->button_event = 1;               // say some GUI input happened
			roger->input_mode = 3;                 // we want the ball input mode
			roger->key_event = 49;                 // say we want a ball (pressed the "1" key)
			roger->button_reference[X] = new_x;    // pass the actual new center location (x)
			roger->button_reference[Y] = new_y;    // pass the actual new center location (y)
			reset_flag = 1;                        // set the reset_flag
			return 1;                              // return true (time to reset)
		}
	} else {
		converged_counter = 0;                     // not converged this time-step, so reset
	}
	return 0;                                      // return false (not reseting)
}

/* 
*  Helper function for Auto Reset functions that automatically generates a
*     new location for an object that is wall_offset away from any wall.
*     
*/
void random_location(wall_offset, x, y)
double wall_offset;
double *x;
double *y;
{
	double x_min, x_max, y_min, y_max;
	x_min = MIN_X_DEV + wall_offset;
	x_max = MAX_X_DEV - wall_offset;
	y_min = MIN_Y_DEV + wall_offset;
	y_max = MAX_X_DEV - wall_offset;

	*x = drand(x_min, x_max);
	*y = drand(y_min, y_max);
}

/****************** Standardized Reward Definition ******************/     
// double ...Reward(state, previous_state, previous_action, internal_state)
// int state;                        // state is the combinatorial state
// int previous_state;               // previous combinatorial state
// int previous_action;              // action called in prev. time-step
// int internal_state[NACTIONS];     // internal state of actions in RL prob.
// {
// 	// define one (or more) reward values
// 	double reward = 0.0; 

// 	// keep track of the previous internal state of one,
// 	//    or more, actions
// 	static int previous_internal_state = NO_REFERENCE;
	
// 	// assign reward based one state transitions, time spent in a certian state, 
// 	//    transitions based on previous actions (and their internal states),
// 	//    etc...
// 	if (internal_state[...] == ... && previous_internal_state == ...
// 		&& previous_action == ...) {
// 		// TODO: fill in your own reward logic
// 	}

// 	previous_internal_state = internal_state[...];
// 	return(reward);
// }


/* 
* RL ToolKit requires AT LEAST ONE reward function.
*     So, if nothing else use this one with always rewards 0.0*     
*/
double NothingReward(state, previous_state, previous_action, internal_state)
int state;
int previous_state;
int previous_action;
int internal_state[NACTIONS];
{
  double reward = 0.0; 
  return(reward);
}

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





/************************************************************************/
void project9_control(roger, time)
Robot *roger;
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
	//     example reward and reset functions
	int (*actions[NACTIONS])(Robot* roger, double errors[NDOF], double time);
    double (*rewards[1])(int state, int previous_state, int previous_action, int internal_state[NSTATES]);
// a = 0.001,0.2. k = 0.8,.999
    double alpha=0.001, gamma=0.99;         // learning rate, reward discounting factor
    int reward_num = 1;                   // total number of rewards used (MUST BE AT LEAST ONE)
    double default_reward = -0.01;         // reward applied for every decision 
	double end_episode_reward = 100;     // reward given for successful episode termination
	double end_episode_penalty = -100;   // penalty for not ending the episode in success

	actions[0] = chase_touch;
	actions[1] = TFClosure;
	actions[2] = VFClosure;
	// as many as you are planning to use ...

	rewards[0] = NothingReward; // or define our own...


    // important note, final eps values is the SUM of eps_min and eps_max
	//     i.e. eps_min = 0.2 and eps_max = 0.6 results in a final eps value of 0.8
	//     eps is the probability of a greedy action
	double e_max = 0.8;
    double eps_min = 0.2, eps_max = e_max-eps_min;
    int max_time_per_episode = 5000;                 // maximum number of time-steps per episode
    int num_episodes = 1000000;                         // number of training episodes

    // learn a policy, saves off a q-table file after every episode

	// NOTE, DOES NOT construct empty folder inside of q_tables/. If you want to put data in a sub-folder
	//     create the sub-folder first!	
	Q_Learning(roger, time, alpha, gamma, actions, rewards, default_reward, NACTIONS, NSTATES, 
		reward_num, proj_nine_q_table, max_time_per_episode, num_episodes, eps_min, eps_max,
		CONVG_ACT, end_episode_reward, end_episode_penalty, Q_TABLE_FILE,
		reset_ball_training);
	/************************************************************/
	
    
	
	/******** Code outline for loading and testing previously learned composite actions ********/
  	// static int policy_loaded = 0;     // track if the policy has already been loaded
	// int q_table_num = ...;            // q_table episode number to load
	
    // // load a learned policy
    // if (policy_loaded == 0) {
	// 	LoadLearnedQTable(q_table_num, proj_X_q_table, NSTATES, NACTIONS, Q_TABLE_FILE);
	// 	policy_loaded = 1;
	// }

    // // exploit the learned policy
	// Run_Policy(roger, time, actions, rewards, default_reward, NACTIONS, NSTATES, reward_num, proj_X_q_table);
	/************************************************************/

	/******** Code outline for collecting performance data for learned composite actions ********/
	// static int action_calls[NACTIONS]; // a static array for tracking action call counts
	// char* performance_data_file_path = "perf_data/.../TODO_%s";  // path to folder to save performance data CSV
	// int num_trails = 50                // number of data collection trials to run

	// // load a learned policy
	// //   q_table_num is the same as used in Run_Policy
    // if (policy_loaded == 0) {
	// 	LoadLearnedQTable(q_table_num, proj_X_q_table, NSTATES, NACTIONS, Q_TABLE_FILE);
	// 	policy_loaded = 1;
	// }

	// // NOTE, DOES NOT construct empty folder inside of perf_data. If you want to put data in a sub-folder
	// //     create the sub-folder first!
	// generatePerformanceData(roger, time, actions, rewards, default_reward, NACTIONS, NSTATES, reward_num, proj_X_q_table,
	// performance_data_file_path, action_calls, num_trails, CONVG_ACT, end_episode_penalty, end_episode_reward, time_per_episode, reset_X); 
	/************************************************************/
}

/************************************************************************/
void project9_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project9_enter_params() 
{
  printf("Project 9 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project9_visualize(roger)
Robot* roger;
{ }

