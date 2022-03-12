/**************************************************************************/
/* File:        RLToolKit.h                                               */
/* Description: all the compile time constants that define Roger          */
/* Author:      Oscar Youngquist                                          */
/* Date:        06-2021                                                   */
/**************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>

/*
*  Runs a given action from array of actions
*/
void RunAction();

/*
*  Implmentation of one-step tabular q-learning
*/
void Q_Learning();

/* 
* Greedy Action Selection
*/
int GetActionGreedy(int state, double * q_table, int num_actions);

/* 
* Select the next action using epsilon-greedy acction selection algorithm
*/
int GetActionEpsilonGreedy(int state, int counter, double * q_table, double eps_min, double eps_max, int itermax, int num_actions);

int GetActionEpsilonGreedy_TimeBased(int state, int counter, double * q_table, double eps_min, double eps_max, int num_episodes, int time_per_eps, int num_actions);

/* 
* Helper function to check if a file exists before opening it
*/
int file_exists(const char *filename);
/* 
* Helper function to saves the current q-table to
*     QTABLE_PATH.txt
*/
void SaveQTable(int fileNum, double *q_table, int numStates, int num_actions, char *fileName);
void SavePolicy(int fileNum, int policy[], int numStates, char *fileName);

/* 
* Helper function to load the specified policy to
*     learned_policy
*/
void LoadLearnedPolicy(int fileNum, int policy[], int numStates, char *fileName);
void LoadLearnedQTable(int fileNum, double *q_table, int numStates, int num_actions, char *fileName);

/* 
* Helper function to generate a random integer between [0, max_num]
*/
int RandomInt(int max_num);

/* 
* Helper function to initialize Q_function to all zero
*/
void InitQFunc(double *q_table, int num_states, int num_actions);

/* 
* Helper function to initialize learned_policy to all zero (first action)
*/
void InitPolicy(int policy[], int num_states);

/* 
* Helper function to generate a learned greedy policy from Q_function
*/
void GeneratePolicy(int policy[], double *q_table, int num_states, int num_actions);

int generatePerformanceData();