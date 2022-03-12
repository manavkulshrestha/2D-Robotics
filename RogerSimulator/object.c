/*************************************************************************/
/* File:        object.c                                                 */
/* Description: all structures and dynamics specific to the objects      */
/* Author:      Rod Grupen                                               */
/* Date:        11-1-2009                                                */
/*************************************************************************/
#include <math.h>
#include "include/roger.h"
#include "include/simulate.h"
#include "include/control.h"

#define SCALE 1.25

// Obj object; // initialized in xrobot.c

ToyObject toy_home_circle = {
                   CIRCLE,    // object id (CIRCLE||TRIANGLE)
                        1,    // number of vertices
  {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
   {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
   {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
   {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}, // radial offsets/theta values
  {R_BALL, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0}, // the radii of the balls
                     RED,
                     R_BALL,
                   M_BALL,    // mass
                   I_BALL,    // mass moment of inertia
{ -2.3, -2.9, 0.0 },    // position in world coordinates
                              // (initially outside/north of drawable canvas
        { 0.0, 0.0, 0.0 },    // velocity in world coordinates
        { 0.0, 0.0, 0.0 },  // default external forces
        1,
        {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
        {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}}};


ToyObject toy_home_triangle = {
                  TRIANGLE,    // object id (CIRCLE||TRIANGLE)
                        34,    // number of vertices
  {{0.0*1.4, 0.0}, {0.154*1.4, M_PI/2}, {0.154*1.4, 7*M_PI/6}, 
  {0.154*1.4, 11*M_PI/6}, {0.130*1.4, 7*M_PI/18},{0.130*1.4, 11*M_PI/18}, 
  {0.130*1.4, 19*M_PI/18}, {0.130*1.4, 23*M_PI/18}, {0.130*1.4, 31*M_PI/18}, 
  {0.130*1.4, 35*M_PI/18}, {0.206*1.4, M_PI/2}, {0.206*1.4, 7*M_PI/6}, 
  {0.206*1.4, 11*M_PI/6}, {0.061*1.4, 2*M_PI/3}, {0.061*1.4, M_PI/3}, 
  {0.075*1.4, 107*M_PI/180}, {0.075*1.4, 73*M_PI/180},{0.097*1.4, 43*M_PI/90},
  { 0.097*1.4, 47*M_PI/90}, { 0.111*1.4, M_PI/2}, {0.061*1.4, M_PI},
  {0.061*1.4, 4*M_PI/3},{0.075*1.4, 193*M_PI/180}, {0.075*1.4, 227*M_PI/180},
  {0.097*1.4, 103*M_PI/90},{0.097*1.4, 107*M_PI/90},{ 0.111*1.4, 7*M_PI/6}, 
  {0.061*1.4, 0},{0.061 *1.4, 5*M_PI/3}, {0.075*1.4, 347*M_PI/180},
  {0.075*1.4, 313*M_PI/180}, {0.097*1.4, 334*M_PI/180}, {0.097 *1.4, 326*M_PI/180}, 
  {0.111*1.4, 11*M_PI/6}, {0.0*1.4, 0.0}, {0.0*1.4, 0.0},
  {0.0*1.4, 0.0}, {0.0*1.4, 0.0}, {0.0*1.4, 0.0},
  {0.0*1.4, 0.0}, {0.0*1.4, 0.0}, {0.0*1.4, 0.0},
  {0.0*1.4, 0.0}, {0.0*1.4, 0.0}, {0.0*1.4, 0.0},
  {0.0*1.4, 0.0}, {0.0*1.4, 0.0}, {0.0*1.4, 0.0},
  {0.0*1.4, 0.0}, {0.0*1.4, 0.0}}, // radial offsets/theta values
  
  {0.12*1.4,0.04*1.4, 0.04*1.4, 0.04*1.4, 0.02*1.4,
  0.02*1.4, 0.02*1.4, 0.02*1.4, 0.02*1.4, 0.02*1.4,
  0.01*1.4, 0.01*1.4, 0.01*1.4, 0.008*1.4, 0.008*1.4,
  0.006*1.4, 0.006*1.4, 0.005*1.4, 0.005*1.4, 0.004*1.4, 
  0.008*1.4, 0.008*1.4, 0.006*1.4, 0.006*1.4, 0.005*1.4, 
  0.005*1.4, 0.004*1.4, 0.008*1.4, 0.008*1.4, 0.006*1.4, 
  0.006*1.4, 0.005*1.4, 0.005*1.4, 0.004*1.4, 0.0*1.4, 
  0.0*1.4, 0.0*1.4, 0.0*1.4, 0.0*1.4, 0.0*1.4,
  0.0*1.4, 0.0*1.4, 0.0*1.4, 0.0*1.4, 0.0*1.4,
  0.0*1.4, 0.0*1.4, 0.0*1.4, 0.0*1.4, 0.0*1.4}, // the radii of the balls 
                     RED,
                     R_BALL,
                  M_TRIANGLE,    // mass
                  I_TRIANGLE,    // mass moment of inertia
      { -1.15, -2.983, 0.0 },    // position in world coordinates
                              // (initially outside/north of drawable canvas
            { 0.0, 0.0, 0.0 },    // velocity in world coordinates
            { 0.0, 0.0, 0.0 },   // default external forces
                            3,
        {{0.23*1.4, M_PI/2}, {0.23*1.4, 7*M_PI/6}, {0.23*1.4, 11*M_PI/6}, {0, 0}, {0, 0}, 
         {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}} };


// sqaure four big circles in the center
ToyObject toy_home_square = {
                  SQUARE,    // object id (CIRCLE||TRIANGLE)
                        13,    // number of vertices
  {{0.0*1.0, 0.0}, 
  {0.141*1.0, M_PI/4}, {0.141*1.0, 3*M_PI_4}, {0.141*1.0, 5*M_PI/4}, {0.141*1.0, 7*M_PI/4},
  {0.175*1.0, 0.0}, {0.175*1.0, M_PI/2}, {0.175*1.0, M_PI}, {0.175*1.0, 3*M_PI/2}, 
  {0.258*1.0, M_PI/4}, {0.258*1.0, 3*M_PI/4}, {0.258*1.0, 5*M_PI/4}, {0.258*1.0, 7*M_PI/4}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}}, // radial offsets/theta values
  
  {0.041*1.0, 
   0.1*1.0, 0.1*1.0, 0.1*1.0, 0.1*1.0,
   0.025*1.0, 0.025*1.0, 0.025*1.0, 0.025*1.0, 
   0.017*1.0, 0.017*1.0, 0.017*1.0, 0.017*1.0, 
   0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0}, // the radii of the balls 
                     RED,
                     R_BALL,
                  M_TRIANGLE,    // mass
                  I_TRIANGLE,    // mass moment of inertia
  { 0.0, -2.9, 0.0 },    // position in world coordinates
                              // (initially outside/north of drawable canvas
        { 0.0, 0.0, 0.0 },    // velocity in world coordinates
        { 0.0, 0.0, 0.0 },  // default external forces
        4,
        {{0.282* 1.0, M_PI/4},{0.282* 1.0, 3*M_PI/4},{0.282* 1.0, 5*M_PI/4},{0.282* 1.0, 7*M_PI/4}, {0, 0},
        {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}}};

// bean
ToyObject toy_home_iso_triangle = {
                   ISO_TRIANGLE,    // object id (CIRCLE||TRIANGLE)
                      7,    // number of vertices
  {{0.051*1.75, 0}, {0.051*1.75, M_PI}, 
  {0.149*1.75, 201*M_PI/180}, {0.149*1.75, 339*M_PI/180}, 
  {0.035*1.75, M_PI/2},
  {0.117*1.75, 2*M_PI/180}, {0.117*1.75, 178*M_PI/180}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}, // radial offsets/theta values
  
  {0.051*1.75, 0.051*1.75, 0.051*1.75, 0.051*1.75, 0.015*1.75,
   0.0115*1.75, 0.015*1.75, 
   0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0}, // the radii of the balls 
                     RED,
                     R_BALL,
                  M_TRIANGLE,    // mass
                  I_TRIANGLE,    // mass moment of inertia
    { 1.15, -3.15, 0.0 },    // position in world coordinates
                              // (initially outside/north of drawable canvas
        { 0.0, 0.0, 0.0 },    // velocity in world coordinates
        { 0.0, 0.0, 0.0 },  // default external forces
        22, 
        {{0.051*1.75, 3*M_PI/2}, {0.068*1.75, (270-34)*M_PI/180},
         {0.093*1.75, (270-44)*M_PI/180}, {0.140*1.75, (270-47)*M_PI/180},
         {0.166*1.75, (270-49)*M_PI/180},{0.193*1.75, (270-57)*M_PI/180},
         {0.203*1.75, (270-65)*M_PI/180},{0.195*1.75, (270-76)*M_PI/180},
         {0.178*1.75, (180+8)*M_PI/180}, {0.126*1.75, (180-8)*M_PI/180},
         {0.077*1.75, (180-33)*M_PI/180},{0.051*1.75, M_PI/2},
        {0.077*1.75, 33*M_PI/180},{0.126*1.75, 8*M_PI/180},
        {0.178*1.75, (360-8)*M_PI/180},{0.195*1.75, (270+76)*M_PI/180},
        {0.203*1.75, (270+65)*M_PI/180},{0.193*1.75, (270+57)*M_PI/180},
        {0.166*1.75, (270+49)*M_PI/180},{0.140*1.75, (270+47)*M_PI/180},
        {0.093*1.75, (270+44)*M_PI/180},{0.068*1.75, (270+34)*M_PI/180}}};

// pentagon
ToyObject toy_home_pentagon = {
                  PENTAGON,    // object id (CIRCLE||TRIANGLE)
                        11,    // number of vertices
  {{0.0*SCALE, 0.0}, 
  {0.116*SCALE, M_PI/10}, {0.116*SCALE, M_PI/2}, {0.116*SCALE, 9*M_PI/10}, {0.116*SCALE, 13*M_PI/10},{0.116*SCALE, 17*M_PI/10}, 
  {0.145*SCALE, 11*M_PI/10}, {0.145*SCALE, 3*M_PI/2}, {0.145*SCALE, 19*M_PI/10}, {0.145*SCALE, 3*M_PI/10}, {0.145*SCALE, 7*M_PI/10}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 
  {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}, // radial offsets/theta values
  
  {0.047*SCALE, 
   0.068*SCALE, 0.068*SCALE, 0.068*SCALE, 0.068*SCALE,0.068*SCALE, 
   0.017*SCALE, 0.017*SCALE, 0.017*SCALE,0.017*SCALE, 0.017*SCALE, 
   0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0,
   0.0, 0.0, 0.0, 0.0, 0.0}, // the radii of the balls 
                     RED,
                     R_BALL,
                  M_TRIANGLE,    // mass
                  I_TRIANGLE,    // mass moment of inertia
{ 2.3, -3.15, 0.0 },    // position in world coordinates
                              // (initially outside/north of drawable canvas
        { 0.0, 0.0, 0.0 },    // velocity in world coordinates
        { 0.0, 0.0, 0.0 },  // default external forces
        5,
        {{0.2*SCALE, M_PI/10},{0.2*SCALE, M_PI/2},{0.2*SCALE, 9*M_PI/10},{0.2*SCALE, 13*M_PI/10},{0.2*SCALE, 17*M_PI/10}}};
void simulate_object(obj)
Obj * obj;
{
  int i;
  double acc[3];
  double mag, fnorm[2];
/*
  printf("net force on object = %6.4lf %6.4lf\n",
	 obj->extForce[0], obj->extForce[1]);*/
  mag = sqrt(SQR(obj->extForce[0])+SQR(obj->extForce[1]));
  fnorm[0] = fnorm[1] = 0.0;

  if (mag < STATIC_FORCE_THRESHOLD) mag=0.0;
  else {
    fnorm[0] = obj->extForce[0]/mag; fnorm[1] = obj->extForce[1]/mag;
    mag -= STATIC_FORCE_THRESHOLD;
  }

  acc[X] = (mag*fnorm[0] - VISCOSITY*obj->velocity[X])/obj->mass;
  acc[Y] = (mag*fnorm[1] - VISCOSITY*obj->velocity[Y])/obj->mass;
  acc[THETA] = (obj->extForce[THETA])/I_BALL;

  //    acc[0] = (obj->extForce[0] - VISCOSITY*obj->vel[0])/obj->mass;
  //    acc[1] = (obj->extForce[1] - VISCOSITY*obj->vel[1])/obj->mass;

  //acc[0] = 0.0;
  //acc[1] = 0.0;

  obj->velocity[X] += acc[X] * DT;
  obj->velocity[Y] += acc[Y] * DT;
  obj->velocity[THETA] += acc[THETA] * DT;

  obj->position[X] += 0.5*acc[X]*SQR(DT) + obj->velocity[X]*DT;
  obj->position[Y] += 0.5*acc[Y]*SQR(DT) + obj->velocity[Y]*DT;
  obj->position[THETA] += 0.5*acc[THETA]*SQR(DT) + obj->velocity[THETA]*DT;

  //  if ((obj->position[X] < MIN_X + R_OBJ) && (obj->velocity[X] < 0.0))
  //    obj->velocity[X] *= -1.0;
  //  if ((obj->position[X] > MAX_X - R_OBJ) && (obj->velocity[X] > 0.0))
  //    obj->velocity[X] *= -1.0;
  //  if ((obj->position[Y] < MIN_Y + R_OBJ) && (obj->velocity[Y] < 0.0))
  //    obj->velocity[Y] *= -1.0;
  //  if ((obj->position[Y] > MAX_Y - R_OBJ) && (obj->velocity[Y] > 0.0))
  //    obj->velocity[Y] *= -1.0;
}


// void simulate_object_polyball(obj)
// PolyBall * obj;
// {
//   int i;
//   double acc[3];
//   double mag, vmag, fnorm[2];

//   //  printf("net force on object = %6.4lf %6.4lf\n",
//   //   obj->net_extForce[X], obj->net_extForce[Y]);

//   mag = sqrt(SQR(obj->net_extForce[X])+SQR(obj->net_extForce[Y]));
//   fnorm[X] = fnorm[Y] = 0.0;

//   if (mag < STATIC_FORCE_THRESHOLD) mag=0.0;
//   else {
//     fnorm[X] = obj->net_extForce[X]/mag; fnorm[Y] = obj->net_extForce[Y]/mag;
//     mag -= STATIC_FORCE_THRESHOLD;
//   }

//   //  printf("\t force mag=%6.4lf  fhat=[%6.4lf %6.4lf]\n", 
//   //   mag, fnorm[X], fnorm[Y]);

//   acc[X] = (mag*fnorm[X] - VISCOSITY*obj->velocity[X])/obj->mass;
//   acc[Y] = (mag*fnorm[Y] - VISCOSITY*obj->velocity[Y])/obj->mass;
//   acc[THETA] = (obj->net_extForce[THETA])/obj->moi;

//   // experimental velocity governor to make collisions behave better
//   obj->velocity[X] += acc[X] * DT;
//   obj->velocity[Y] += acc[Y] * DT;
//   obj->velocity[THETA] += acc[THETA] * DT;


//   // printf("velocity=%lf\n",sqrt(SQR(obj->velocity[X]) +
//   //                              SQR(obj->velocity[Y])));

//   obj->position[X] += 0.5*acc[X]*SQR(DT) + obj->velocity[X]*DT;
//   obj->position[Y] += 0.5*acc[Y]*SQR(DT) + obj->velocity[Y]*DT;
//   obj->position[THETA] += 0.5*acc[THETA]*SQR(DT) + obj->velocity[THETA]*DT;
//   //  if ((obj->position[X] < MIN_X + R_OBJ) && (obj->velocity[X] < 0.0))
//   //    obj->velocity[X] *= -1.0;
//   //  if ((obj->position[X] > MAX_X - R_OBJ) && (obj->velocity[X] > 0.0))
//   //    obj->velocity[X] *= -1.0;
//   //  if ((obj->position[Y] < MIN_Y + R_OBJ) && (obj->velocity[Y] < 0.0))
//   //    obj->velocity[Y] *= -1.0;
//   //  if ((obj->position[Y] > MAX_Y - R_OBJ) && (obj->velocity[Y] > 0.0))
//   //    obj->velocity[Y] *= -1.0;

//   //  printf("\t\t X: acc=%lf vel=%lf pos=%lf\n", 
//   //   acc[X], obj->velocity[X], obj->position[X]);
//   //  printf("\t\t Y: acc=%lf vel=%lf pos=%lf\n", 
//   //   acc[Y], obj->velocity[Y], obj->position[Y]);
//   //  printf("\t\t THETA: acc=%lf vel=%lf pos=%lf\n", 
//   //   acc[THETA], obj->velocity[THETA], obj->position[THETA]);
// }


void simulate_object_polyball(obj)
ToyObject * obj;
{
  int i;
  double acc[3];
  double mag, vmag, fnorm[2];

  //  printf("net force on object = %6.4lf %6.4lf\n",
  //   obj->net_extForce[X], obj->net_extForce[Y]);

  mag = sqrt(SQR(obj->net_extForce[X])+SQR(obj->net_extForce[Y]));
  fnorm[X] = fnorm[Y] = 0.0;

  if (mag < STATIC_FORCE_THRESHOLD) mag=0.0;
  else {
    fnorm[X] = obj->net_extForce[X]/mag; fnorm[Y] = obj->net_extForce[Y]/mag;
    mag -= STATIC_FORCE_THRESHOLD;
  }

  //  printf("\t force mag=%6.4lf  fhat=[%6.4lf %6.4lf]\n", 
  //   mag, fnorm[X], fnorm[Y]);

  acc[X] = (mag*fnorm[X] - VISCOSITY*obj->velocity[X])/obj->mass;
  acc[Y] = (mag*fnorm[Y] - VISCOSITY*obj->velocity[Y])/obj->mass;
  acc[THETA] = (obj->net_extForce[THETA])/obj->moi;

  // experimental velocity governor to make collisions behave better
  obj->velocity[X] += acc[X] * DT;
  obj->velocity[Y] += acc[Y] * DT;
  obj->velocity[THETA] += acc[THETA] * DT;


  // printf("velocity=%lf\n",sqrt(SQR(obj->velocity[X]) +
  //                              SQR(obj->velocity[Y])));

  obj->position[X] += 0.5*acc[X]*SQR(DT) + obj->velocity[X]*DT;
  obj->position[Y] += 0.5*acc[Y]*SQR(DT) + obj->velocity[Y]*DT;
  obj->position[THETA] += 0.5*acc[THETA]*SQR(DT) + obj->velocity[THETA]*DT;
  //  if ((obj->position[X] < MIN_X + R_OBJ) && (obj->velocity[X] < 0.0))
  //    obj->velocity[X] *= -1.0;
  //  if ((obj->position[X] > MAX_X - R_OBJ) && (obj->velocity[X] > 0.0))
  //    obj->velocity[X] *= -1.0;
  //  if ((obj->position[Y] < MIN_Y + R_OBJ) && (obj->velocity[Y] < 0.0))
  //    obj->velocity[Y] *= -1.0;
  //  if ((obj->position[Y] > MAX_Y - R_OBJ) && (obj->velocity[Y] > 0.0))
  //    obj->velocity[Y] *= -1.0;

  //  printf("\t\t X: acc=%lf vel=%lf pos=%lf\n", 
  //   acc[X], obj->velocity[X], obj->position[X]);
  //  printf("\t\t Y: acc=%lf vel=%lf pos=%lf\n", 
  //   acc[Y], obj->velocity[Y], obj->position[Y]);
  //  printf("\t\t THETA: acc=%lf vel=%lf pos=%lf\n", 
  //   acc[THETA], obj->velocity[THETA], obj->position[THETA]);
}



