/*************************************************************************/
/* File:        project1.c                                               */
/* Description: PD control analogs of the biological motor units for     */
/*              every degree of freedom: eyes, arms, base rotate and     */
/*              base translate --- motor units execute every simulated   */
/*              millisecond and are never disengaged. Higher-level       */
/*              control applications submit sequences of setpoints to    */
/*              combinations of motorunits.                              */
/* Date:        1-2015                                                   */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "matrix_math.h"

#define EPSILON 0.1

void update_setpoints();

/*************************************************************************/
/* PROJECT #1 - COMPLETE THE FOLLOWING CONTROL PROCEDURES                */
// gains for the PD controllers EYES
double Kp_eye = 1;
double Kd_eye = 0.02;
double passive_Kd_eye = 0.001;

// ARMS 
double Kp_arm =  100;
double Kd_arm =  6.5;
// double Kp_arm =  30;
// double Kd_arm =  2.6;
double passive_Kd_arm = 1.0;

// BASE TRANSLATION
// double Kp_base_trans = 200;
// double Kd_base_trans = 15; 
double Kp_base_trans = 125;
double Kd_base_trans = 26;

double passive_Kd_base_trans = 2.0;

// BASE ROTATION
double Kp_base_rot =  50;
double Kd_base_rot =  8.2;
double passive_Kd_base_rot = 1.0;
/*************************************************************************/

int flip_flag = 0;

/* PROJECT #1.1 - PD CONTRSmOLLER FOR THE EYES                             */
/* setpoints are joint angle values in radians for the eyes              */
void PDController_eyes(roger, time)
Robot * roger;
double time;
{
  int i;
  double theta_error, theta_dot_error;

  // FILE *eye_file = fopen("eyes.txt", "a");

  for (i = 0; i < NEYES; i++) {
    theta_error = roger->eyes_setpoint[i] - roger->eye_theta[i];
    theta_dot_error = 0.0 - roger->eye_theta_dot[i];
    if (ACTUATE_EYES) {
      // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE EYE
      // USING YOUR GAINS
      // roger->eye_torque[i] = passive_Kd_eye*theta_dot_error;
      roger->eye_torque[i] = Kp_eye*theta_error+Kd_eye*theta_dot_error;
      // if (i == 0)
      //   fprintf(eye_file, "%f\n", theta_error);
    } 
    else roger->eye_torque[i] = passive_Kd_eye*theta_dot_error;
  }

  // fclose(eye_file);
}

/* PROJECT #1.2 - PD CONTROLLER FOR THE ARMS                             */
/* setpoints - joint angles in radians for the shoulders and elbows      */
void PDController_arms(roger, time)
Robot * roger;
double time;
{
  int i, j;
  double theta_error, theta_dot_error;

  // FILE *arm_file = fopen("arms.txt", "a");

  // setting setpoint goal to pi/2
  // for (j=0; j<NARM_JOINTS; ++j)
  //     roger->arm_setpoint[0][j] = flip_flag ? 1.57079632679 : 0;

  for (i=LEFT; i<=RIGHT; ++i) {
    for (j=0; j<NARM_JOINTS; ++j) {

      theta_error = roger->arm_setpoint[i][j] - roger->arm_theta[i][j];
      theta_dot_error = 0.0 - roger->arm_theta_dot[i][j];

      while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
      while (theta_error < -M_PI) theta_error += 2.0 * M_PI;

      // tune kp_arm and kd_arm by changing their value using enter_params()
      if (ACTUATE_ARMS) {
        // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE ARM
        // USING YOUR GAINS
        // roger->arm_torque[i][j] = passive_Kd_arm * theta_dot_error;
      	roger->arm_torque[i][j] = Kp_arm*theta_error+Kd_arm*theta_dot_error;

        // if (i == 0 && flip_flag)
        //   fprintf(arm_file, "%d %f\n", j, theta_error);
      }
      else {
        roger->arm_torque[i][j] = passive_Kd_arm * theta_dot_error;
      }
    }
  }

  // fclose(arm_file);
}

/* Base PD controller, Cartesian reference */
double PDBase_translate(roger, time) 
Robot * roger;
double time;
{ 
  double Fx, error[2], trans_error, trans_vel;

  error[X] = roger->base_setpoint[X] - roger->base_position[X];
  error[Y] = roger->base_setpoint[Y] - roger->base_position[Y];

  trans_error = error[X]*cos(roger->base_position[THETA]) +
    error[Y]*sin(roger->base_position[THETA]);

  trans_vel = roger->base_velocity[X]*cos(roger->base_position[THETA]) +
    roger->base_velocity[Y]*sin(roger->base_position[THETA]);

  if (ACTUATE_BASE) {
    // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE BASE
    // USING YOUR GAINS
    Fx = Kp_base_trans*trans_error-Kd_base_trans*trans_vel;
  }
  else {
    Fx = - passive_Kd_base_trans * trans_vel;
  }

  return(Fx);
}

/* Base PD controller, Cartesian reference */
double PDBase_rotate(roger, time) 
Robot * roger;
double time;
{
  double Mz, theta_error, theta_dot_error;

  FILE *fout = fopen("base_b.txt", "a");

  double error[2];
  error[X] = roger->base_setpoint[X] - roger->base_position[X];
  error[Y] = roger->base_setpoint[Y] - roger->base_position[Y];

  double pos_error = fabs(error[X])+fabs(error[Y]);
  if (pos_error > EPSILON) {
    roger->base_setpoint[THETA] = atan2(error[Y], error[X]);
    // printf("error is %f+%f=%f\n", error[X], error[Y], pos_error);
  }

  if (flip_flag) {
    roger->base_setpoint[X] = .75;
    // roger->base_setpoint[X] = 0;
    // roger->base_setpoint[Y] = -.75;
    // roger->base_setpoint[Y] = 0;
    // roger->base_setpoint[THETA] = 0;
  }

  theta_error = roger->base_setpoint[THETA] - roger->base_position[THETA];
  theta_dot_error = 0.0 - roger->base_velocity[THETA];
  while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
  while (theta_error < -M_PI) theta_error += 2.0 * M_PI;

  if (flip_flag) {
    fprintf(fout, "%f %f %f\n", error[X], error[Y], theta_error);
    // flip_flag = !flip_flag;
  }

  if (ACTUATE_BASE) {
    // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE BASE
    // USING YOUR GAINS
    Mz = Kp_base_rot*theta_error+Kd_base_rot*theta_dot_error;
  }
  else {
    Mz = passive_Kd_base_rot * theta_dot_error;
  }

  fclose(fout);
  
  return(Mz);
}

/* PROJECT #1.3 - PD CONTROLLER FOR THE BASE                             */
/* setpoints - (xy) location for translation heading in radians          */

/*   the base differential drive Jacobian:                               */
/*    |tau_l|     |Fx|      |  x_dot  |     |theta_dot_left |            */
/*    |     |= JT |  |      |         | = J |               |            */
/*    |tau_r|     |Mz|      |theta_dot|     |theta_dot_right|            */

double baseJT[2][2] = 
  {{(1.0/2.0), -(1.0/(2.0*R_AXLE))}, {(1.0/2.0), (1.0/(2.0*R_AXLE))}};

void PDController_base(roger, time)
Robot * roger;
double time;
{ 
  double Fx, Mz, PDBase_translate(), PDBase_rotate();

  // Fx = PDBase_translate(roger,time); // translate along current heading
  // Mz = 0.0;

  // Fx = 0.0;                          // rotate in current footprint
  // Mz = PDBase_rotate(roger,time);

  Fx = PDBase_translate(roger,time);     // translate and rotate 
  Mz = PDBase_rotate(roger,time);

  // integrated wheel torque control
  roger->wheel_torque[LEFT] = baseJT[0][0]*Fx + baseJT[0][1]*Mz;
  roger->wheel_torque[RIGHT] = baseJT[1][0]*Fx + baseJT[1][1]*Mz;
}

/*************************************************************************/
/*       THE SIMULATOR EXECUTES control_roger() EVERY CONTROL CYCLE      */
/*                        *** DO NOT ALTER ***                           */
/*************************************************************************/
void control_roger(roger, time)
Robot * roger;
double time;
{
  update_setpoints(roger); // check_GUI_inputs(roger)

  // turn setpoint references into torques
  PDController_eyes(roger, time);
  PDController_arms(roger, time);
  PDController_base(roger,time);

}

/*************************************************************************/
void project1_reset(roger)
Robot* roger;
{ }

/*************************************************************************/
/* prompt for and read user customized input values                      */
/*************************************************************************/
void project1_enter_params()
{
  // put anything in here that you would like to change at run-time
  // without re-compiling user project codes

  // printf("ARM: K=%6.4lf  B=%6.4lf\n", Kp_arm, Kd_arm);
  // printf("ARM: enter 'K B'\n"); fflush(stdout);
  // scanf("%lf %lf", &Kp_arm, &Kd_arm);
  // printf("ARM: K=%6.4lf  B=%6.4lf\n", Kp_arm, Kd_arm);

  printf("FLIP FLAG WAS %d AND IS NOW ", flip_flag);
  flip_flag = !flip_flag;
  printf("%d\n", flip_flag);

  // printf("BASE T: K=%6.4lf  B=%6.4lf\n", Kp_base_trans, Kd_base_trans);
  // printf("BASE T: enter 'K B'\n"); fflush(stdout);
  // scanf("%lf %lf", &Kp_base_trans, &Kd_base_trans);
  // printf("BASE T: K=%6.4lf  B=%6.4lf\n", Kp_base_trans, Kd_base_trans);
}

/*************************************************************************/
// function called when the 'visualize' button on the gui is pressed            
void project1_visualize(roger)
Robot* roger;
{ }

