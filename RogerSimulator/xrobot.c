/****************************************************************/
/** xrobot.c: simulates and renders mobile manipulator         **/
/**           version of Roger-the-Crab                        **/
/** author:   Grupen                                           **/
/** date:     April, 2020                                      **/
/****************************************************************/
#include <stdio.h>
#include <X11/Xlib.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "Xkw/Xkw.h"

#include "include/roger.h"
#include "include/simulate.h"
#include "include/control.h"
#include "include/modes.h"
#define NXBINS            160   // why is this here?
#define NYBINS             64

// global variables
int ServerPorts[MaxRobotNum] = {8000, 8001, 8002};

Robot Rogers[MaxRobotNum];
RogerBody RogersBodys[MaxRobotNum];


// objects here are ONLY robot body parts and poly_ball toys
// TODO update this to be something other than PolyBall - OR - rename polyball?
ToyObject objects_total[MaxInertialObjectsNum]; // polyball is obsolete
                                               // MaxInternalObjectsNum -> simulate.h4

History history[MaxRobotNum][MAX_HISTORY]; // to illustrate a trajectory
int history_ptr[MaxRobotNum];

int numRoger = 0;
int numObjects = 0; // number of inertial objects

int numToy = N_TOY_TYPES;     // will likely have to modify
                    // currently only used in the main function to initialize the number of objects
                    // maybe could remove, maybe will need to increment as toys are added.
                    // UPDATE numToy is the number of toys in the toybox

int socknew[MaxRobotNum];

// simulator data structures that comprise Roger
Base mobile_base_home; // there are 4 robots declared in this section
Eye eyes_home[NEYES];
Arm arms_home[NARMS][NARM_FRAMES];

// individual home positions for all robots - only if ARENA
Base mobile_base_home_1;
Eye eyes_home_1[NEYES];
Arm arms_home_1[NARMS][NARM_FRAMES];
Base mobile_base_home_2;
Eye eyes_home_2[NEYES];
Arm arms_home_2[NARMS][NARM_FRAMES];
Base mobile_base_home_3;
Arm arms_home_3[NARMS][NARM_FRAMES];


// A SET OF OBJECTS (simulate.h) (MOBILE/STATIONARY) FOR PROGRAMMING EXERCISES:
// CIRCLE, TRIANGLE, SQUARE, TRIANGULAR receptical, SQUARE receptical
//    N_TOY_TYPES = 5   // number of unique toys in the toybox
//       MAX_TOYS = 10  // maximum number of simulated toys
// N_TOY_TYPES -> simulate.h
// MAX_TOYS    -> simulate.h 
ToyObject toy; 
extern ToyObject toy_home_circle; // this doesn't seem to actually need to be extern
extern ToyObject toy_home_triangle;
extern ToyObject toy_home_square;
extern ToyObject toy_home_iso_triangle;
extern ToyObject toy_home_pentagon;
// PolyBall toy_home;

ToyObject toybox[N_TOY_TYPES]; // currently 2
ToyObject active_toys[MaxToyNum]; // currently 2


int input_toy_type = 0; // 0<=input_toy_type<NTOY_TYPES, initialized 0 (CIRCLE)
int n_active_toys = 0;  // the current number of user instantiated toys
                        // do we need this and numToy variable above? numToy variable only
                        // give the max number of numToy available


SimColor world_colors[117];
int draw_visual = FALSE;     // what does draw_visual mean?

int init_control_flag = TRUE;

// global Boolean reset flag - eliminates user defined obstacles and goals
// reconfigures Roger and single object to starting configuration
int reset;

// global Boolean motor saturation flag for robot 0...set in read_interface()
int saturation_flag[NDOF];  // [LW,RW,LE,RE,LAS,LAE,RAS,RAE]

// global environment variable (Arena or Development)
int kEnvironment;

// global simulator clock
double simtime = 0.0;

void x_canvas_proc(), x_start_proc(), x_params_proc(),
     x_input_mode_arena_proc(), x_input_mode_dev_proc(),
     x_control_mode_arena_proc(), x_input_mode_2_proc(),
     x_control_mode_2_proc();
void x_quit_proc(), x_timer_proc(), x_visualize_proc();
void x_sock_1_proc(), x_sock_2_proc();
void initialize_control_mode();
void SIMmatXvec(), SIMmatXmat(), SIMinv_transform(), SIMcopy_matrix();
void simulate_object(), simulate_arm(), simulate_base(), simulate_eyes();
void SocketCommunicate(), SocketInit();
void usleep();
void InitArenaEnv();
void InitDevelopmentEnv();
void x_control_mode_proc(), x_room_proc();
void simulate_object_polyball(), simulate_roger(), HandleDrawingRequests();
void simulate_object_toys();
void draw_object_toys();
void matrix_mult();
void initialize_toybox();
void copy_object();
void draw_toy_object();
void initialize_room_overlay();

// --------------------------

Display          *display;
Window           window;
Window           overlay;
Pixmap           pixmap;
XtAppContext     app_con;
GC               gc;
GC               gcoverlay;
int              screen;
Widget           canvas_w, input_mode_1_w, control_mode_1_w, params_w;
Widget           start_w, popup_w = NULL;
Widget       input_mode_2_w, control_mode_2_w, pause_w = NULL;
Widget       input_mode_w, control_mode_w, room_w;
Widget           intensity_w, stream_w;
XtIntervalId     timer = 0;
int              width = (int)(1.0* WIDTH), height = (int)(1.0*HEIGHT), depth;
unsigned long    foreground, background;

int width_pixmap = (int)((float)(WIDTH)* PIX_MAP_SIZE_RATIO);
int height_pixmap = (int)((float)(HEIGHT)* PIX_MAP_SIZE_RATIO);
float zoom = 1;


// Helper functions for converting world coordinates to pixmap
int ConvertWorld2PixmapX(float scale, float num, int environment) {
  if (environment == ARENA) return W2DX(scale, num);
  else return W2DX_DEV(scale, num);
}

int ConvertWorld2PixmapY(float scale, float num, int environment) {
  if (environment == ARENA) return W2DY(scale, num);
  else return W2DY_DEV(scale, num);
}

int ConvertWorld2PixmapR(float scale, float num, int environment) {
  if (environment == ARENA) return W2DR(scale, num);
  else return W2DR_DEV(scale, num);
}

// Helper functions for converting pixmap to world coordinates
double ConvertPixmap2WorldX(float scale, int num, int environment) {
  if (environment == ARENA) return D2WX(scale, num);
  else return D2WX_DEV(scale, num);
}

double ConvertPixmap2WorldY(float scale, int num, int environment) {
  if (environment == ARENA) return D2WY(scale, num);
  else return D2WY_DEV(scale, num);
}

double ConvertPixmap2WorldR(float scale, int num, int environment) {
  if (environment == ARENA) return D2WR(scale, num);
  else return D2WR_DEV(scale, num);
}

void x_init_colors()
{
  int i;

  //  printf("initializing grey scale colors..."); fflush(stdout);
  for (i = 0; i <= 100; i++) { // 0 => black; 100 => white
    sprintf(world_colors[i].name, "grey%d", i);
    world_colors[i].display_color =
      XkwParseColor(display, world_colors[i].name);
    world_colors[i].red = (int)i*2.55;
    world_colors[i].green = (int)i*2.55;
    world_colors[i].blue = (int)i*2.55;
  }
  strcpy(world_colors[101].name, "dark red");
  world_colors[101].display_color = XkwParseColor(display, "dark red");
  world_colors[101].red = 139;
  world_colors[101].green = 0;
  world_colors[101].blue = 0;

  strcpy(world_colors[102].name, "red");
  world_colors[102].display_color = XkwParseColor(display, "red");
  world_colors[102].red = 255;
  world_colors[102].green = 0;
  world_colors[102].blue = 0;

  strcpy(world_colors[103].name, "hot pink");
  world_colors[103].display_color = XkwParseColor(display, "hot pink");
  world_colors[103].red = 255;
  world_colors[103].green = 105;
  world_colors[103].blue = 180;

  strcpy(world_colors[104].name, "navy");
  world_colors[104].display_color = XkwParseColor(display, "navy");
  world_colors[104].red = 65;
  world_colors[104].green = 105;
  world_colors[104].blue = 225;

  strcpy(world_colors[105].name, "blue");
  world_colors[105].display_color = XkwParseColor(display, "blue");
  world_colors[105].red = 0;
  world_colors[105].green = 0;
  world_colors[105].blue = 255;

  strcpy(world_colors[106].name, "light sky blue");
  world_colors[106].display_color = XkwParseColor(display, "light sky blue");
  world_colors[106].red = 250;
  world_colors[106].green = 128;
  world_colors[106].blue = 114;

  strcpy(world_colors[107].name, "dark green");
  world_colors[107].display_color = XkwParseColor(display, "dark green");
  world_colors[107].red = 244;
  world_colors[107].green = 164;
  world_colors[107].blue = 96;

  strcpy(world_colors[108].name, "green");
  world_colors[108].display_color = XkwParseColor(display, "green");
  world_colors[108].red = 0;
  world_colors[108].green = 255;
  world_colors[108].blue = 0;

  strcpy(world_colors[109].name, "light green");
  world_colors[109].display_color = XkwParseColor(display, "light green");
  world_colors[109].red = 46;
  world_colors[109].green = 139;
  world_colors[109].blue = 87;

  strcpy(world_colors[110].name, "gold");
  world_colors[110].display_color = XkwParseColor(display, "gold");
  world_colors[110].red = 160;
  world_colors[110].green = 82;
  world_colors[110].blue = 45;

  strcpy(world_colors[111].name, "yellow");
  world_colors[111].display_color = XkwParseColor(display, "yellow");
  world_colors[111].red = 255;
  world_colors[111].green = 255;
  world_colors[111].blue = 0;

  strcpy(world_colors[112].name, "light goldenrod");
  world_colors[112].display_color = XkwParseColor(display, "light goldenrod");
  world_colors[112].red = 192;
  world_colors[112].green = 192;
  world_colors[112].blue = 192;

  strcpy(world_colors[113].name, "purple");
  world_colors[113].display_color = XkwParseColor(display, "purple");
  world_colors[113].red = 141;
  world_colors[113].green = 9;
  world_colors[113].blue = 230;

  strcpy(world_colors[114].name, "magenta");
  world_colors[114].display_color = XkwParseColor(display, "magenta");
  world_colors[114].red = 196;
  world_colors[114].green = 5;
  world_colors[114].blue = 230;

  strcpy(world_colors[115].name, "orange");
  world_colors[115].display_color = XkwParseColor(display, "orange");
  world_colors[115].red = 235;
  world_colors[115].green = 126;
  world_colors[115].blue = 16;

  strcpy(world_colors[116].name, "aqua");
  world_colors[116].display_color = XkwParseColor(display, "aqua");
  world_colors[116].red = 16;
  world_colors[116].green = 235;
  world_colors[116].blue = 198;
}

double motor_model(tau, omega, tau_s, omega_0)
double tau, omega, tau_s, omega_0;
{
  int i;
  double tau_max, tau_min;

  if (omega >= 0) { // motor velocity positive
    tau_min = -tau_s;
    tau_max = tau_s - (tau_s / omega_0)*omega;
  }
  else { // motor velocity negative
    tau_min = -tau_s - (tau_s / omega_0)*omega;
    tau_max = tau_s;
  }
  if (tau < tau_min) tau = tau_min;
  if (tau > tau_max) tau = tau_max;

  return(tau);
}

// used in the GUI handling loop to place a ball in the enviornment
void place_object(x,y,id)
double x,y;
int id;
{
  printf("\trequest to place object %d\n", id);
  if (n_active_toys < MaxToyNum) {
    active_toys[n_active_toys].id = toybox[id].id;
    active_toys[n_active_toys].N = toybox[id].N;
    for (int i = 0; i < NUM_VERTICES; ++i) {
      active_toys[n_active_toys].vertices[i][0] = toybox[id].vertices[i][0];
      active_toys[n_active_toys].vertices[i][1] = toybox[id].vertices[i][1];
      active_toys[n_active_toys].radii[i] = toybox[id].radii[i];
    }
    active_toys[n_active_toys].N_draw = toybox[id].N_draw;

    for (int i = 0; i < NUM_DRAW_VERTS; ++i){
      active_toys[n_active_toys].draw_verts[i][0] = toybox[id].draw_verts[i][0];
      active_toys[n_active_toys].draw_verts[i][1] = toybox[id].draw_verts[i][1];
    }

    active_toys[n_active_toys].color = toybox[id].color;
    active_toys[n_active_toys].image_radius = toybox[id].image_radius;
    active_toys[n_active_toys].mass = M_BALL;
    active_toys[n_active_toys].moi = I_BALL;
    active_toys[n_active_toys].position[X] = x;
    active_toys[n_active_toys].position[Y] = y;
    active_toys[n_active_toys].position[THETA] = 0.0;
    active_toys[n_active_toys].velocity[X] = active_toys[n_active_toys].velocity[Y] = active_toys[n_active_toys].velocity[THETA] = 0.0;
    active_toys[n_active_toys].net_extForce[X] = active_toys[n_active_toys].net_extForce[Y] = active_toys[n_active_toys].net_extForce[THETA] = 0.0;
    n_active_toys++;    // increment number of active toys
    numObjects++;       // increment number of inertial objects
    printf("\tn_active_toys: %d, numObjects: %d\n", n_active_toys, numObjects);
  } else { // otherwise replace the first object specific to the click
    printf("\t inside of else block for place object!\n");
    for (int q = 0; q < n_active_toys; q++) {
      if (active_toys[q].id == id) {
        active_toys[q].id = toybox[id].id;
        active_toys[q].N = toybox[id].N;
        for (int i = 0; i < NUM_VERTICES; ++i) {
          active_toys[q].vertices[i][0] = toybox[id].vertices[i][0];
          active_toys[q].vertices[i][1] = toybox[id].vertices[i][1];
          active_toys[q].radii[i] = toybox[id].radii[i];
        }

        active_toys[n_active_toys].N_draw = toybox[id].N_draw;

        for (int i = 0; i < NUM_DRAW_VERTS; ++i){
          active_toys[n_active_toys].draw_verts[i][0] = toybox[id].draw_verts[i][0];
          active_toys[n_active_toys].draw_verts[i][1] = toybox[id].draw_verts[i][1];
        }

        active_toys[q].color = toybox[id].color;
        active_toys[q].image_radius = toybox[id].image_radius;
        active_toys[q].mass = M_BALL;
        active_toys[q].moi = I_BALL;
        active_toys[q].position[X] = x;
        active_toys[q].position[Y] = y;
        active_toys[q].position[THETA] = 0.0;
        active_toys[q].velocity[X] = active_toys[q].velocity[Y] = active_toys[q].velocity[THETA] = 0.0;
        active_toys[q].net_extForce[X] = active_toys[q].net_extForce[Y] = active_toys[q].net_extForce[THETA] = 0.0;
        return;
      }
    }
  }
}

void remove_object(x, y)
double x, y;
{
  double obj_distance;
  int id;
  double obj_x, obj_y;
  int removed_object = 0;

  if (n_active_toys > 0) {

    for (int q = 0; q < n_active_toys; ++q) {
      id = active_toys[q].id;
      obj_distance = sqrt(SQR(active_toys[q].position[X] - x) + SQR(active_toys[q].position[Y] - y));
      
      if (removed_object == 0) {
        if (obj_distance <= REMOVE_DIST) {
          removed_object = 1;
          printf("n_active_toys: %d, numObjects: %d\n", n_active_toys, numObjects);
        }
      } else { // shift all the other objects over apperoperiately.
        copy_object(q, &active_toys[q-1], active_toys);
        copy_object(q+N_BODY_ROBOT, &objects_total[q-1+N_BODY_ROBOT], objects_total);
      }
      
    }
    if (removed_object == 1) {
      copy_object(id, &active_toys[n_active_toys-1], toybox);
      copy_object(id, &objects_total[numObjects-1], toybox);
      // active_toys[n_active_toys-1].position[X] = objects_total[numObjects-1].position[X] = toybox[id].position[X];
      // active_toys[n_active_toys-1].position[Y] = objects_total[numObjects-1].position[Y] = toybox[id].position[Y];
      // active_toys[n_active_toys-1].position[THETA] = objects_total[numObjects-1].position[THETA] = 0.0;

      // active_toys[n_active_toys-1].velocity[X] = objects_total[numObjects-1].velocity[X] = 0.0;
      // active_toys[n_active_toys-1].velocity[Y] = objects_total[numObjects-1].velocity[Y] = 0.0;
      // active_toys[q].velocity[THETA] = objects_total[numObjects-1].velocity[THETA] = 0.0;

      // active_toys[n_active_toys-1].net_extForce[X] = objects_total[numObjects-1].net_extForce[X] = 0.0;
      // active_toys[n_active_toys-1].net_extForce[Y] = objects_total[q + N_BODY_ROBOT].net_extForce[Y] = 0.0;
      // active_toys[n_active_toys-1].net_extForce[THETA] = objects_total[q + N_BODY_ROBOT].net_extForce[THETA] = 0.0;
      
      n_active_toys -= 1;
      numObjects -= 1;
    }
  }
}

void x_params_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  Rogers[0].enter_param_event = TRUE;
}


int change_input_mode_dev()
{
  static int input_mode;

  input_mode = (input_mode + 1) % N_INPUT_MODES_DEV;
  //init_input_flag = TRUE;
  return (input_mode);
}

int change_input_mode_arena()
{
  static int input_mode;

  input_mode = (input_mode + 1) % N_INPUT_MODES_ARENA;
  //init_input_flag = TRUE;
  return (input_mode);
}


void x_input_mode_dev_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  Rogers[0].input_mode = change_input_mode_dev();

  switch (Rogers[0].input_mode) {
  case JOINT_ANGLE_INPUT:
    XkwSetWidgetLabel(input_mode_w, "Input: Joint angles"); break;
  case BASE_GOAL_INPUT:
    XkwSetWidgetLabel(input_mode_w, "Input: Base goal"); break;
  case ARM_GOAL_INPUT:
    XkwSetWidgetLabel(input_mode_w, "Input: Arm goals"); break;
  case BALL_INPUT:
    XkwSetWidgetLabel(input_mode_w, "Input: Ball position"); break;
  case MAP_INPUT:
    XkwSetWidgetLabel(input_mode_w, "Input: Map Editor"); break;
  default: break;
  }
}

void x_input_mode_arena_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  int i;

  int current_input_mode = change_input_mode_arena();
  for (i = 0; i < numRoger; ++i) {
    Rogers[i].input_mode = current_input_mode;
  }

  switch (Rogers[0].input_mode) {
  case BALL_INPUT_ARENA:
    XkwSetWidgetLabel(input_mode_1_w, "Input: Ball position"); break;
  case MAP_INPUT_ARENA:
    XkwSetWidgetLabel(input_mode_1_w, "Input: Map Editor"); break;
  default: break;
  }
}

int change_control_mode_dev()
{
  static int control_mode;

  control_mode = (control_mode + 1) % N_CONTROL_MODES;
  init_control_flag = TRUE;
  return (control_mode);
}

int change_control_mode_arena()
{
  static int control_mode;

  control_mode = (control_mode + 1) % N_CONTROL_MODES;
  init_control_flag = TRUE;
  return (control_mode);
}


void x_control_mode_arena_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  int i, current_control_mode;

  current_control_mode = change_control_mode_arena();

  for (i = 0; i < numRoger; ++i) {
    Rogers[i].control_mode = current_control_mode;
  }


  switch (Rogers[0].control_mode) {
     case PROJECT1:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 1-MotorUnits"); break;
     case PROJECT2:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 2-ArmKinematics"); break;
     case PROJECT3:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 3-Vision"); break;
     case PROJECT4:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 4-SearchTrack"); break;
     case PROJECT5:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 5-StereoKinematics");
       break;
     case PROJECT6:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 6-Kalman"); break;
     case PROJECT7:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 7-ChasePunch"); break;
     case PROJECT8:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 8-PathPlanning"); break;
     case PROJECT9:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 9-Grasp"); break;
     case PROJECT10:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 10-Transport"); break;
     case PROJECT11:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 11-Belief"); break;
     case PROJECT12:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 12-PONG"); break;
     default: break;
  }
  //call init here makes it independent of timer running
  for (i = 0; i < numRoger; ++i) {
    initialize_control_mode(&Rogers[i]);
  }
}

void x_quit_proc(w, client_data, call_data)
Widget w;
XtPointer call_data, client_data;
{
  XFreePixmap(display, pixmap);
  XFreeGC(display, gc);
  XtDestroyApplicationContext(app_con);
  exit(0);
}


void assignKeyPress(key_id)
char key_id;
{
  for (int k = 0; k < numRoger; ++k) {
    Rogers[k].key_event = key_id;
  }
}

// GUI button info is added to roger struct here!!!!!
void x_canvas_proc(w, client_data, call_data)
Widget w;
XtPointer call_data, client_data;
{
  XEvent *event = ((XEvent *)call_data);
  char text[10];
  KeySym key;
  double x, y, theta1, theta2;
  int i, j, xbin, ybin, k;
  int c;
  static Dimension nwidth = WIDTH, nheight = HEIGHT;

  // Original width and height of current environemtn window
  double env_width, env_height;
  if (kEnvironment == ARENA) {
    // nwidth = WIDTH;
    // nheight = HEIGHT;
    env_width = (double)WIDTH;
    env_height = (double)HEIGHT;
  } else {
    // nwidth = WIDTH_DEV;
    // nheight = HEIGHT_DEV;
    env_width = (double)WIDTH_DEV;
    env_height = (double)HEIGHT_DEV;
  }
  void x_expose(), x_clear();

  switch (event->type) {
  case ConfigureNotify:
    nwidth = event->xconfigure.width;
    nheight = event->xconfigure.height;
    // Calculate the zoom scale for scaling the windows and graphics
    float zoom_width = (double)nwidth / env_width;
    float zoom_height = (double)nheight / env_height;
    zoom = zoom_width > zoom_height ? zoom_height : zoom_width;

    // Limit the zoom scale between 1.0 and PIX_MAP_SIZE_RATIO
    zoom = zoom > 1.0 ? zoom : 1.0;
    zoom = zoom > (float)(PIX_MAP_SIZE_RATIO) ? 
      (float)(PIX_MAP_SIZE_RATIO) : zoom;

    for (k = 0; k < numRoger; ++k) {
      Rogers[k].graphics.zoom = zoom;
    }

    break;
  case Expose:
    if (nwidth == width && nheight == height) x_expose();
    else {
      width = nwidth; height = nheight;
      x_expose();
    }
    break;

  case ButtonPress:
    for (k = 0; k < numRoger; ++k) {
      Rogers[k].button_reference[X] = 
	ConvertPixmap2WorldX(zoom, event->xbutton.x, kEnvironment);
      Rogers[k].button_reference[Y] = 
	ConvertPixmap2WorldY(zoom, event->xbutton.y, kEnvironment);
      Rogers[k].button_event = event->xbutton.button;
    }
    break;

  case ButtonRelease:
    break;

  case KeyPress:
    c = XLookupString((XKeyEvent *)event, text, 10, &key, 0);
    if (c == 1)
      switch (text[0]) {
      case 'h': break;
      case 'c': x_clear(); x_expose(); break;
      case 'q': x_quit_proc(w, client_data, call_data); break;
      case '1':
        // printf("\tpressed key number 1!\n");
      case '2':
        // printf("\tpressed key number 2!\n");
      case '3':
        // printf("\tpressed key number 3!\n");
      case '4':
        // printf("\tpressed key number 4!\n");
      case '5':
        assignKeyPress(text[0]);
        break;
      }
  }
}

void x_draw_line(color, start_x, start_y, end_x, end_y)
int color;
double start_x, start_y, end_x, end_y;
{

  // printf("start_x: %.4f, stop_x: %.4f, start_y: %.4f, stop_y: %.4f\n", start_x, end_x, start_y, end_y);

  XSetForeground(display, gc, world_colors[color].display_color);
  XDrawLine(display, pixmap, gc, 
	    ConvertWorld2PixmapX(zoom, start_x, kEnvironment), 
	    ConvertWorld2PixmapY(zoom, start_y, kEnvironment),
	    ConvertWorld2PixmapX(zoom, end_x, kEnvironment),
	    ConvertWorld2PixmapY(zoom, end_y, kEnvironment));
}

void x_draw_string(color, x, y, string)
int color;
double x, y;
char string[100];
{
  XSetForeground(display, gc, world_colors[color].display_color);
  XDrawString(display, pixmap, gc,
        ConvertWorld2PixmapX(zoom, x, kEnvironment),
        ConvertWorld2PixmapY(zoom, y, kEnvironment), string, strlen(string));
}

void x_draw_circle(color, center_x, center_y, radius, fill)
int color;
double center_x;
double center_y;
double radius;
int fill;
{
  void draw_circle();
  XSetForeground(display, gc, world_colors[color].display_color);
  draw_circle(ConvertWorld2PixmapX(zoom,center_x, kEnvironment),
	      ConvertWorld2PixmapY(zoom,center_y, kEnvironment),
	      ConvertWorld2PixmapR(zoom,radius, kEnvironment),fill);
}

void x_draw_pixel(color, x, y) 
int color;
double x;
double y;
{
  // XSetForeground(display, gc, world_colors[color].display_color);
  // printf("%f, %f\n", x, y);
  XSetForeground(display, gc, world_colors[color].display_color);
  XFillRectangle(display, pixmap, gc, 
    ConvertWorld2PixmapX(zoom, x, kEnvironment),
    ConvertWorld2PixmapY(zoom, y, kEnvironment),
    XDELTA_DEV * 64,
    YDELTA_DEV * 64
  );
}

void x_expose()
{
  XCopyArea(display, pixmap, window, gc, 0, 0, width, height, 0, 0);
}

void x_clear()
{
  XSetForeground(display, gc, background);
  XFillRectangle(display, pixmap, gc, 0, 0, width_pixmap, height_pixmap);
}

// shouldn't be here
#define STEP         0.01
#define STREAM_SPACE 2


// what does draw_visual mean?
void x_visualize_proc(w, client_data, call_data)
{
  if (draw_visual == TRUE) draw_visual=FALSE;
  else draw_visual = TRUE;
}

// what is this doing here? this is part of the special purpose 
// draw_streamline() visualization tool that is specific to the
// harmonic function path planning project
void mark_used(ii, jj, aux)
int ii, jj;
int aux[NBINS][NBINS];
{
  int j, k;
  double dist;

  for (j = -STREAM_SPACE; j <= STREAM_SPACE; ++j) {
    for (k = -STREAM_SPACE; k <= STREAM_SPACE; ++k) {
      dist = sqrt(SQR((double)j) + SQR((double)k));
      if ((dist < (2.0*STREAM_SPACE + 1.0)) &&
        ((ii + j) >= 0) && ((ii + j) < NBINS) &&
        ((jj + k) >= 0) && ((jj + k) < NBINS))
        aux[ii + j][jj + k] = TRUE;
    }
  }
}


void write_interface(reset)
int reset;
{
  int i, j, k;

  for (k = 0; k < numRoger; ++k) {
    // pass in afferents (read only)
    Rogers[k].eye_theta[0] = RogersBodys[k].eyes[0].theta;
    Rogers[k].eye_theta_dot[0] = RogersBodys[k].eyes[0].theta_dot;
    Rogers[k].eye_theta[1] = RogersBodys[k].eyes[1].theta;
    Rogers[k].eye_theta_dot[1] = RogersBodys[k].eyes[1].theta_dot;
    for (i = 0; i<NPIXELS; ++i) {
      Rogers[k].image[LEFT][i][RED_CHANNEL] =
        world_colors[RogersBodys[k].eyes[LEFT].image[i]].red;
      Rogers[k].image[LEFT][i][GREEN_CHANNEL] =
        world_colors[RogersBodys[k].eyes[LEFT].image[i]].green;
      Rogers[k].image[LEFT][i][BLUE_CHANNEL] =
        world_colors[RogersBodys[k].eyes[LEFT].image[i]].blue;
      Rogers[k].image[RIGHT][i][RED_CHANNEL] =
        world_colors[RogersBodys[k].eyes[RIGHT].image[i]].red;
      Rogers[k].image[RIGHT][i][GREEN_CHANNEL] =
        world_colors[RogersBodys[k].eyes[RIGHT].image[i]].green;
      Rogers[k].image[RIGHT][i][BLUE_CHANNEL] =
        world_colors[RogersBodys[k].eyes[RIGHT].image[i]].blue;
    }
    Rogers[k].arm_theta[0][0] = RogersBodys[k].arms[0][1].theta;
    Rogers[k].arm_theta[0][1] = RogersBodys[k].arms[0][2].theta;
    Rogers[k].arm_theta[1][0] = RogersBodys[k].arms[1][1].theta;
    Rogers[k].arm_theta[1][1] = RogersBodys[k].arms[1][2].theta;
    Rogers[k].arm_theta_dot[0][0] = RogersBodys[k].arms[0][1].theta_dot;
    Rogers[k].arm_theta_dot[0][1] = RogersBodys[k].arms[0][2].theta_dot;
    Rogers[k].arm_theta_dot[1][0] = RogersBodys[k].arms[1][1].theta_dot;
    Rogers[k].arm_theta_dot[1][1] = RogersBodys[k].arms[1][2].theta_dot;
    Rogers[k].ext_force[0][X] = 
      RogersBodys[k].arms[0][NARM_FRAMES-1].extForce[X];
    Rogers[k].ext_force[0][Y] = 
      RogersBodys[k].arms[0][NARM_FRAMES-1].extForce[Y];
    Rogers[k].ext_force[1][X] = 
      RogersBodys[k].arms[1][NARM_FRAMES-1].extForce[X];
    Rogers[k].ext_force[1][Y] = 
      RogersBodys[k].arms[1][NARM_FRAMES-1].extForce[Y];

    Rogers[k].base_position[0] = RogersBodys[k].mobile_base.x;
    Rogers[k].base_position[1] = RogersBodys[k].mobile_base.y;
    Rogers[k].base_position[2] = RogersBodys[k].mobile_base.theta;
    Rogers[k].base_velocity[0] = RogersBodys[k].mobile_base.x_dot;
    Rogers[k].base_velocity[1] = RogersBodys[k].mobile_base.y_dot;
    Rogers[k].base_velocity[2] = RogersBodys[k].mobile_base.theta_dot;
    Rogers[k].ext_force_body[X] = -RogersBodys[k].mobile_base.extForce[X];
    Rogers[k].ext_force_body[Y] = -RogersBodys[k].mobile_base.extForce[Y];

    // zero efferents (write only)
    Rogers[k].eye_torque[0] = Rogers[k].eye_torque[1] = 0.0;
    Rogers[k].arm_torque[0][0] = Rogers[k].arm_torque[0][1] = 
      Rogers[k].arm_torque[1][0] = Rogers[k].arm_torque[1][1] = 0.0;
    Rogers[k].wheel_torque[0] = Rogers[k].wheel_torque[1] = 0.0;
    Rogers[k].simtime = simtime;
  }
}

void read_interface()
{
  int i,k;

  for (i=0; i<NDOF; ++i) {     // INIT GLOBAL saturation_flag for rendering
    saturation_flag[i] = FALSE;
  }
    
  // SATURATION FLAG [NDOF];  // [LW,RW,LE,RE,LAS,LAE,RAS,RAE]
  for (k = 0; k < numRoger; ++k) {
    // pass back torques (write only)
    //
    // LEFT WHEEL
    RogersBodys[k].mobile_base.wheel_torque[0] =
      motor_model(Rogers[k].wheel_torque[0], Rogers[k].wheel_theta_dot[0],
		  WHEEL_TS, WHEEL_W0);
    if ((k==0) && (RogersBodys[k].mobile_base.wheel_torque[0] != 
		    Rogers[k].wheel_torque[0]))
      saturation_flag[0] = TRUE;

    // RIGHT WHEEL
    RogersBodys[k].mobile_base.wheel_torque[1] =
      motor_model(Rogers[k].wheel_torque[1], Rogers[k].wheel_theta_dot[1],
		  WHEEL_TS, WHEEL_W0);
    if ((k==0) && (RogersBodys[k].mobile_base.wheel_torque[1] != 
		    Rogers[k].wheel_torque[1]))
      saturation_flag[1] = TRUE;

    // LEFT EYE
    RogersBodys[k].eyes[0].torque =
      motor_model(Rogers[k].eye_torque[0], Rogers[k].eye_theta_dot[0],
		  EYE_TS, EYE_W0);
    if ((k==0) && (RogersBodys[k].eyes[0].torque != Rogers[k].eye_torque[0]))
      saturation_flag[2] = TRUE;

    // RIGHT EYE
    RogersBodys[k].eyes[1].torque =
      motor_model(Rogers[k].eye_torque[1], Rogers[k].eye_theta_dot[1],
		  EYE_TS, EYE_W0);
    if ((k==0) && (RogersBodys[k].eyes[1].torque != Rogers[k].eye_torque[1]))
      saturation_flag[3] = TRUE;

    // LEFT ARM / SHOULDER JOINT
    RogersBodys[k].arms[0][1].torque =
      motor_model(Rogers[k].arm_torque[0][0], Rogers[k].arm_theta_dot[0][0],
		  SHOULDER_TS, SHOULDER_W0);
    if ((k==0) && (RogersBodys[k].arms[0][1].torque !=
		   Rogers[k].arm_torque[0][0]))
      saturation_flag[4] = TRUE;

    // LEFT ARM / ELBOW JOINT
    RogersBodys[k].arms[0][2].torque =
      motor_model(Rogers[k].arm_torque[0][1], Rogers[k].arm_theta_dot[0][1],
		  ELBOW_TS, ELBOW_W0);
    if ((k==0) && (RogersBodys[k].arms[0][2].torque !=
		   Rogers[k].arm_torque[0][1]))
      saturation_flag[5] = TRUE;

    // RIGHT ARM / SHOULDER JOINT
    RogersBodys[k].arms[1][1].torque =
      motor_model(Rogers[k].arm_torque[1][0], Rogers[k].arm_theta_dot[1][0],
		  SHOULDER_TS, SHOULDER_W0);
    if ((k==0) && (RogersBodys[k].arms[1][1].torque != 
		   Rogers[k].arm_torque[1][0]))
      saturation_flag[6] = TRUE;

    // RIGHT ARM / ELBOW JOINT
    RogersBodys[k].arms[1][2].torque =
      motor_model(Rogers[k].arm_torque[1][1], Rogers[k].arm_theta_dot[1][1],
		  ELBOW_TS, ELBOW_W0);
    if ((k==0) && (RogersBodys[k].arms[1][2].torque != 
		   Rogers[k].arm_torque[1][1]))
      saturation_flag[7] = TRUE;
  }
}

/* forward kinematics in base frame **************************************/
void sim_fwd_kinematics(arm_id, theta1, theta2, x, y)
int arm_id;
double theta1, theta2;
double *x, *y;
{
  *x = L_ARM1 * cos(theta1) + L_ARM2 * cos(theta1 + theta2);
  *y = L_ARM1 * sin(theta1) + L_ARM2 * sin(theta1 + theta2);

  if (arm_id == LEFT) *y += ARM_OFFSET;
  else *y -= ARM_OFFSET;
}

void sim_arm_Jacobian(theta1, theta2, Jacobian)
double theta1, theta2;
double Jacobian[2][2];
{
  Jacobian[0][0] = -L_ARM1*sin(theta1) - L_ARM2*sin(theta1 + theta2);
  Jacobian[0][1] = -L_ARM2*sin(theta1 + theta2);
  Jacobian[1][0] = L_ARM1*cos(theta1) + L_ARM2*cos(theta1 + theta2);
  Jacobian[1][1] = L_ARM2*cos(theta1 + theta2);
}

//#define NBODY 4 // single ball, two hands, roger's body

// copy the contents of a ToyObject object from an array of objects
//     to the specific obj item
void copy_object(i, obj, objects)
int i;
ToyObject * obj;
ToyObject objects[NBODY];
{
  obj->id = objects[i].id;
  obj->N = objects[i].N;
  for (int q = 0; q < NUM_VERTICES; ++q) {
      obj->vertices[q][0] = objects[i].vertices[q][0];
      obj->vertices[q][1] =  objects[i].vertices[q][1];
      obj->radii[q] =  objects[i].radii[q];
      // if (objects[i].N > 1) {
      //   printf("radii[%d]: %.4f, theta: %.4f, radial-offset: %.4f\n", q, objects[i].radii[q], objects[i].vertices[q][0], objects[i].vertices[q][1]);
      //   printf("copy: radii[%d]: %.4f, theta: %.4f, radial-offset: %.4f\n", q, obj->radii[q], obj->vertices[q][0], obj->vertices[q][1]);
      // }
  }
  obj->N_draw = objects[i].N_draw;
  for (int q = 0; q < NUM_DRAW_VERTS; ++q) {
    obj->draw_verts[q][0] = objects[i].draw_verts[q][0];
    obj->draw_verts[q][1] = objects[i].draw_verts[q][1];
  }


  obj->color = objects[i].color;
  obj->image_radius = objects[i].image_radius;
  obj->mass = objects[i].mass;
  obj->moi = objects[i].moi;
  obj->position[X] = objects[i].position[X];
  obj->position[Y] = objects[i].position[Y];
  obj->position[THETA] = objects[i].position[THETA];

  obj->velocity[X] = objects[i].velocity[X];
  obj->velocity[Y] = objects[i].velocity[Y];
  obj->velocity[THETA] = objects[i].velocity[THETA];

  obj->net_extForce[X] = objects[i].net_extForce[X];
  obj->net_extForce[Y] = objects[i].net_extForce[Y];
  obj->net_extForce[THETA] = objects[i].net_extForce[THETA];
}


// We will have to update the size of the objects_total array to account for the 
void update_objects()
{
  double pb[4], pw[4]; // homogeneous position vectors in base and world coords
  double vb[4], vw[4]; // homogeneous velocity vectors in base and world coords
  double x, y, J[2][2];
  int i;

  void sim_fwd_kinematics(), sim_arm_Jacobian();

  int base_counter = 0;
  for (i = 0; i < numRoger; ++i) {
    base_counter = i * N_BODY_ROBOT;

    /*************************** 0: BASE ************************************/
    objects_total[base_counter + BASE].N = 1;
    objects_total[base_counter + base_counter + BASE].radii[0] = R_BASE;
    objects_total[base_counter + BASE].vertices[0][0] = 0.0;
    objects_total[base_counter + BASE].vertices[0][1] = 0.0;


    objects_total[base_counter + BASE].mass = M_BASE;
    objects_total[base_counter + BASE].moi = I_BASE;

    objects_total[base_counter + BASE].position[X] = 
      RogersBodys[i].mobile_base.x;
    objects_total[base_counter + BASE].position[Y] = 
      RogersBodys[i].mobile_base.y;
    objects_total[base_counter + BASE].position[THETA] = 
      RogersBodys[i].mobile_base.theta;

    objects_total[base_counter + BASE].velocity[X] = 
      RogersBodys[i].mobile_base.x_dot;
    objects_total[base_counter + BASE].velocity[Y] = 
      RogersBodys[i].mobile_base.y_dot;
    objects_total[base_counter + BASE].velocity[THETA] = 
      RogersBodys[i].mobile_base.theta_dot;

    objects_total[base_counter + BASE].net_extForce[X] = 
      objects_total[base_counter + BASE].net_extForce[Y] =
      objects_total[base_counter + BASE].net_extForce[THETA] = 0.0;

    /*********************** 1: LEFT ARM ************************************/
    // left hand position in world coordinates
    sim_fwd_kinematics(LEFT, RogersBodys[i].arms[LEFT][1].theta, 
		       RogersBodys[i].arms[LEFT][2].theta, &x, &y);
    pb[0] = x; pb[1] = y; pb[2] = 0.0; pb[3] = 1.0;
    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, pb, pw);

    // left hand velocity relative to base written in base coordinates
    //          (figure 2 in RogerDynamics document defines frames)
    // ^w(v)_10 = wRb[ ^b(v)_7 +  J_arm theta_dot_arm ]
    //    = ^w(v)_b + wRb[ -ARM_OFFSET omega_0 xhat_b + J_arm theta_dot_arm ]
    sim_arm_Jacobian(RogersBodys[i].arms[LEFT][1].theta, 
		     RogersBodys[i].arms[LEFT][2].theta, J);
    vb[X] = J[0][0] * RogersBodys[i].arms[LEFT][1].theta_dot +
      J[0][1] * RogersBodys[i].arms[LEFT][2].theta_dot - 
      ARM_OFFSET*RogersBodys[i].mobile_base.theta_dot;
    vb[Y] = J[1][0] * RogersBodys[i].arms[LEFT][1].theta_dot + 
      J[1][1] * RogersBodys[i].arms[LEFT][2].theta_dot;
    vb[2] = 0.0;
    vb[3] = 0.0; // homogeneous vector

    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, vb, vw);
    //  v[1][X] = base->x_dot + v_w[X];
    //  v[1][Y] = base->y_dot + v_w[Y];
    //  R[1] = R_TACTILE;

    objects_total[base_counter + ARM1].N = 1;
    objects_total[base_counter + ARM1].radii[0] = R_TACTILE;
    objects_total[base_counter + ARM1].vertices[0][0] = 0.0;
    objects_total[base_counter + ARM1].vertices[0][1] = 0.0;



    objects_total[base_counter + ARM1].mass = M_ARM1;
    objects_total[base_counter + ARM1].moi = I_ARM1;

    objects_total[base_counter + ARM1].position[X] = pw[X];
    objects_total[base_counter + ARM1].position[Y] = pw[Y];
    objects_total[base_counter + ARM1].position[THETA] = 0.0;
    // hand orientation is not relevant to system dynamics

    objects_total[base_counter + ARM1].velocity[X] = 
      RogersBodys[i].mobile_base.x_dot + vw[X];
    objects_total[base_counter + ARM1].velocity[Y] = 
      RogersBodys[i].mobile_base.y_dot + vw[Y];
    objects_total[base_counter + ARM1].velocity[THETA] = 0.0;
    // angular acceleration in the hand is not relevant to system dynamics

    objects_total[base_counter + ARM1].net_extForce[X] = 
      objects_total[base_counter + ARM1].net_extForce[Y] =
      objects_total[base_counter + ARM1].net_extForce[THETA] = 0.0;

    /********************** 2: RIGHT ARM ************************************/
    // right hand position in world coordinates
    sim_fwd_kinematics(RIGHT, RogersBodys[i].arms[RIGHT][1].theta, 
		       RogersBodys[i].arms[RIGHT][2].theta, &x, &y);
    pb[0] = x; pb[1] = y; pb[2] = 0.0; pb[3] = 1.0;
    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, pb, pw);

    // left hand velocity relative to base written in base coordinates
    //          (figure 2 in RogerDynamics document defines frames)
    // ^w(v)_10 = wRb[ ^b(v)_7 +  J_arm theta_dot_arm ]
    //    = ^w(v)_b + wRb[ -ARM_OFFSET omega_0 xhat_b + J_arm theta_dot_arm ]
    sim_arm_Jacobian(RogersBodys[i].arms[RIGHT][1].theta, 
		     RogersBodys[i].arms[RIGHT][2].theta, J);
    vb[X] = J[0][0] * RogersBodys[i].arms[RIGHT][1].theta_dot +
      J[0][1] * RogersBodys[i].arms[RIGHT][2].theta_dot - 
      ARM_OFFSET*RogersBodys[i].mobile_base.theta_dot;
    vb[Y] = J[1][0] * RogersBodys[i].arms[RIGHT][1].theta_dot + 
      J[1][1] * RogersBodys[i].arms[RIGHT][2].theta_dot;
    vb[2] = 0.0;
    vb[3] = 0.0; // homogeneous vector

    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, vb, vw);

    objects_total[base_counter + ARM2].N = 1;
    objects_total[base_counter + ARM2].radii[0] = R_TACTILE;
    objects_total[base_counter + ARM2].vertices[0][0] = 0.0;
    objects_total[base_counter + ARM2].vertices[0][1] = 0.0;
    objects_total[base_counter + ARM2].mass = M_ARM1;
    objects_total[base_counter + ARM2].moi = I_ARM1;

    objects_total[base_counter + ARM2].position[X] = pw[X];
    objects_total[base_counter + ARM2].position[Y] = pw[Y];
    objects_total[base_counter + ARM2].position[THETA] = 0.0;
    // hand orientation not relevant to system dynamics

    objects_total[base_counter + ARM2].velocity[X] = 
      RogersBodys[i].mobile_base.x_dot + vw[X];
    objects_total[base_counter + ARM2].velocity[Y] = 
      RogersBodys[i].mobile_base.y_dot + vw[Y];
    objects_total[base_counter + ARM2].velocity[THETA] = 0.0;
    // angular acceleration of hand is not relevant to system dynamics

    objects_total[base_counter +ARM2].net_extForce[X] = 
      objects_total[base_counter + ARM2].net_extForce[Y] =
    objects_total[base_counter + ARM2].net_extForce[THETA] = 0.0;
  }
  
  
  // updated to now be 3 (for one toy)
  base_counter = base_counter + N_BODY_ROBOT;

  /****************** 3: TOY OBJECT - CIRCLE || TRIANGLE *********************/

  // TODO loop over active toys

  for (int q = 0; q < n_active_toys; ++q) {
    int toy_counter = base_counter + q;

    objects_total[toy_counter].id = active_toys[q].id;
    objects_total[toy_counter].N = active_toys[q].N;
    for (int i = 0; i < NUM_VERTICES; ++i) {
        objects_total[toy_counter].vertices[i][0] = active_toys[q].vertices[i][0];
        objects_total[toy_counter].vertices[i][1] =  active_toys[q].vertices[i][1];
        objects_total[toy_counter].radii[i] =  active_toys[q].radii[i];
    }
    objects_total[toy_counter].mass = active_toys[q].mass;
    objects_total[toy_counter].moi = active_toys[q].moi;

    objects_total[toy_counter].color = active_toys[q].color;
    objects_total[toy_counter].image_radius = active_toys[q].image_radius;

    objects_total[toy_counter].position[X] = active_toys[q].position[X];
    objects_total[toy_counter].position[Y] = active_toys[q].position[Y];
    objects_total[toy_counter].position[THETA] = active_toys[q].position[THETA];

    objects_total[toy_counter].velocity[X] = active_toys[q].velocity[X];
    objects_total[toy_counter].velocity[Y] = active_toys[q].velocity[Y];
    objects_total[toy_counter].velocity[THETA] = active_toys[q].velocity[THETA];

    objects_total[toy_counter].net_extForce[X] = 
      objects_total[toy_counter].net_extForce[Y] =
      objects_total[toy_counter].net_extForce[THETA] = 0.0;
  }
}

void compute_external_forces()
{
  int i, j, ii, jj, row, col;
  double ri[2], rj[2];
  double dr[2], mag, theta, dij, rx, ry, fx, fy, tz, radiusi, radiusj, thetai, thetaj;
  ToyObject obji, objj;

  void copy_object(), update_objects(), SIMfwd_kinematics(), SIMarm_Jacobian();

  // Define delta_x, delta_y, and r_obstacle size given the environment
  float x_delta, y_delta, r_obstacle;
  float max_x, max_y, min_x, min_y;
  if (kEnvironment == DEVELOPMENT) {
    x_delta = XDELTA_DEV;
    y_delta = YDELTA_DEV;
    r_obstacle = R_OBSTACLE_DEV;
    max_x = MAX_X_DEV;
    min_x = MIN_X_DEV;
    max_y = MAX_Y_DEV;
    min_y = MIN_Y_DEV;
  }
  else {
    x_delta = XDELTA;
    y_delta = YDELTA;
    r_obstacle = R_OBSTACLE;
    max_x = MAX_X;
    min_x = MIN_X;
    max_y = MAX_Y;
    min_y = MIN_Y;
  }


  // copy the current pos/vel info from device (mobile_base, arms, and toy
  // structures) into inertial "PolyBall objects[NBODY]" array
  // called before comupting forces, which is why the update_objects function assings a 
  //     net force of zero to all DOFs (x, y, theta)
  update_objects();



  for (i = 0; i<(numObjects); ++i) { // compute force on body i by body j
    for (j = (i + 1); j < (numObjects+1) ; ++j) {

      copy_object(i, &obji, objects_total); 
      copy_object(j, &objj, objects_total);

      // printf("obj[%d]->N: %d\n", i, obji.N);

      // This if bloack calculates forces for the wall "objects"
      // Last body is the occupancy grid - sum compression
      if (j == numObjects) { // needs to be upgraded too
        //  printf("checking body i=%d bouncing on OBSTACLE j=%d\n", i,j);
        for (ii = 0; ii<obji.N; ++ii) {
          thetai = obji.vertices[ii][1];
          radiusi = obji.vertices[ii][0];
          double rad = obji.radii[ii];

          // if (obji.N > 1) {
          //     printf("\tjj, %d, objj->thetai: %.4f, radiusi: %.4f, radii: %.4f \n", jj, thetai, radiusi, rad, rad);
          // }

          // printf("obji[%d]->N: %d\n", i, obji.N);
          // printf("\tii, %d, obji->thetai: %.4f, radiusii: %.4f, radius: %.4f\n", ii, thetai, radiusi, obji.radii[ii]);

          // position of the first vertex
          //    theta = (double)k*(2.0*M_PI/3.0);
          //    xt = RT*cos((double)k*one_twenty);
          //    yt = RT*sin((double)k*one_twenty);
          ri[X] = obji.position[X]
            + (radiusi * cos(thetai))
            * cos(obji.position[THETA])
            - (radiusi * sin(thetai))
            * sin(obji.position[THETA]);
          ri[Y] = obji.position[Y]
            + (radiusi * cos(thetai))
            * sin(obji.position[THETA])
            + (radiusi * sin(thetai))
            * cos(obji.position[THETA]);

          // ri[X] = obji.position[X] +
          //   radiusi * cos(obji.position[THETA] + thetai);
          // ri[Y] = obji.position[Y] +
          //   radiusi * sin(obji.position[THETA] + thetai);


          for (row = 0; row<NBINS; ++row) {
            for (col = 0; col<NBINS; ++col) {
              if (Rogers[0].world_map.occupancy_map[row][col] == OBSTACLE) {
                // printf("\t row: %d, col: %d, x: %.4f, y: %.4f\n", row, col, (min_x + (col + 0.5)*x_delta), (max_y - (row + 0.5)*y_delta), row);

                dr[X] = ri[X] - (min_x + (col + 0.5)*x_delta);
                dr[Y] = ri[Y] - (max_y - (row + 0.5)*y_delta);
                mag = sqrt(SQR(dr[X]) + SQR(dr[Y]));
                dij = MAX(0.0, (obji.radii[ii] + r_obstacle - mag));
                fx = K_COLLIDE*dij*(dr[X] / mag);
                fy = K_COLLIDE*dij*(dr[Y] / mag);

                rx = radiusi * cos(obji.position[THETA] + thetai);
                ry = radiusi * sin(obji.position[THETA] + thetai);
                tz = rx * fy - fx * ry;



                // if (obji.N > 1 && (abs(fx) > 0 || abs(fy) > 0)) {
                //   printf("\tjj, %d, objj->thetai: %.4f, radiusi: %.4f, radii: %.4f \n", jj, thetai, radiusi, rad, rad);
                //   printf("\tfx: %.4f, fy: %.4f ii: %d, \n", fx, fy, ii, jj);
                //   printf("\trow: %d, col: %d\n", row, col, jj);
                //   printf("\tx_obs : %.4f, y_obs: %.4f\n", (min_x + (col + 0.5)*x_delta), (max_y - (row + 0.5)*y_delta), jj);
                //   printf("\tx_cir : %.4f, y_cir: %.4f\n", ri[X], ri[Y], jj);
                //   printf("\tx_obj : %.4f, y_obj: %.4f, theta_obj: %.4f\n", obji.position[X], obji.position[Y], obji.position[THETA], jj);
                //   Rogers[0].world_map.color_map[row][col] = LIGHTYELLOW;
                // }

                objects_total[i].net_extForce[X] += fx;
                objects_total[i].net_extForce[Y] += fy;
                objects_total[i].net_extForce[THETA] += tz;
              }
            }
          }
        }

        if (VERBOSE) {
          // if (objects_total[i].N > 1) {
            printf("\tforce on body i=%d  f = [%6.4lf %6.4lf %6.4lf]\n", i, objects_total[i].net_extForce[X], objects_total[i].net_extForce[Y],
                    objects_total[i].net_extForce[THETA]);
            printf("\tobject id: %d\n", objects_total[i].id);
          // }
        }
      } else { // j not the occupacy grid: BASE||ARM#1||ARM#2||Toy
        // iterate over the list of "vertex" elastic spheres for object i
        for (ii = 0; ii<obji.N; ++ii) {
          // printf("obji[%d]->N: %d\n", i, obji.N);
          thetai = obji.vertices[ii][1];
          radiusi = obji.vertices[ii][0];

          // printf("obji[%d]->N: %d\n", i, obji.N);
          // printf("\tii, %d, obji->thetai: %.4f, radiusii: %.4f, radius: %.4f\n", ii, thetai, radiusi, obji.radii[ii]);

          // position of the first vertex
          //    theta = (double)k*(2.0*M_PI/3.0);
          //    xt = RT*cos((double)k*one_twenty);
          //    yt = RT*sin((double)k*one_twenty);
          ri[X] = obji.position[X] +
            radiusi * cos(obji.position[THETA] + thetai);
          ri[Y] = obji.position[Y] +
            radiusi * sin(obji.position[THETA] + thetai);

          // iterate over the list of "vertex" elastic spheres for object i
          // if (objj.N > 1) {
          //   printf("objj[%d]->N: %d\n", j, objj.N);
          // }
          
          for (jj = 0; jj<objj.N; ++jj) {
            thetaj = objj.vertices[jj][1];
            radiusj = objj.vertices[jj][0];
            double rad = objj.radii[jj];

            // if (objj.id == SQUARE) {
            //   printf("\tjj, %d, objj->thetaj: %.4f, radiusj: %.4f, radii: %.4f \n", jj, thetaj, radiusj, rad);
            // }

            rj[X] = objj.position[X] +
              radiusj * cos(objj.position[THETA] + thetaj);
            rj[Y] = objj.position[Y] +
              radiusj * sin(objj.position[THETA] + thetaj);

            dr[X] = ri[X] - rj[X];
            dr[Y] = ri[Y] - rj[Y];
            mag = sqrt(SQR(dr[X]) + SQR(dr[Y]));
            dij = MAX(0.0, (obji.radii[ii] + objj.radii[jj] - mag));
            fx = K_COLLIDE*dij*(dr[X] / mag);
            fy = K_COLLIDE*dij*(dr[Y] / mag);

            rx = radiusi * cos(obji.position[THETA] + thetai);
            ry = radiusi * sin(obji.position[THETA] + thetai);
            tz = rx * fy - fx * ry;

            objects_total[i].net_extForce[X] += fx;
            objects_total[i].net_extForce[Y] += fy;
            objects_total[i].net_extForce[THETA] += tz;

            rx = radiusj * cos(objj.position[THETA] + thetaj);
            ry = radiusj * sin(objj.position[THETA] + thetaj);
            tz = rx * fy - fx * ry;

            // if (objj.N > 1 && (abs(fx) > 0 || abs(fy) > 0)) {
            //   printf("\tfx: %.4f, fy: %.4f jj: %d, \n", fx, fy, jj, jj);
            // }

            objects_total[j].net_extForce[X] -= fx;
            objects_total[j].net_extForce[Y] -= fy;
            objects_total[j].net_extForce[THETA] -= tz;
          }
        }
      }
    }
  }


  // Update the RogersBodys with the calculated collision forces
  int base_counter = 0;
  for (i = 0; i < numRoger; ++i) {
    base_counter = i * N_BODY_ROBOT;

    // BASE
    RogersBodys[i].mobile_base.extForce[X] = 
      objects_total[base_counter + BASE].net_extForce[X];
    RogersBodys[i].mobile_base.extForce[Y] = 
      objects_total[base_counter + BASE].net_extForce[Y];

    // ARM #1
    //  reality check: why do you need the negative of fb?
    RogersBodys[i].arms[LEFT][NARM_FRAMES - 1].extForce[X] = 
      -objects_total[base_counter + ARM1].net_extForce[X];
    RogersBodys[i].arms[LEFT][NARM_FRAMES - 1].extForce[Y] = 
      -objects_total[base_counter + ARM1].net_extForce[Y];

    // ARM #2
    //  reality check: why do you need the negative of fb?
    RogersBodys[i].arms[RIGHT][NARM_FRAMES - 1].extForce[X] = 
      -objects_total[base_counter + ARM2].net_extForce[X];
    RogersBodys[i].arms[RIGHT][NARM_FRAMES - 1].extForce[Y] = 
      -objects_total[base_counter + ARM2].net_extForce[Y];
  }
  base_counter = base_counter + N_BODY_ROBOT;

  // TOY OBJECT
  // TODO -> need to iterate over all active toys

  for (int q = 0; q < n_active_toys; ++q) {
    int toy_counter = base_counter + q;

    active_toys[q].net_extForce[X] = objects_total[toy_counter].net_extForce[X];
    active_toys[q].net_extForce[Y] = objects_total[toy_counter].net_extForce[Y];
    active_toys[q].net_extForce[THETA] = objects_total[toy_counter].net_extForce[THETA];
  }

  // toy.net_extForce[X] = objects_total[base_counter].net_extForce[X];
  // toy.net_extForce[Y] = objects_total[base_counter].net_extForce[Y];
  // toy.net_extForce[THETA] = objects_total[base_counter].net_extForce[THETA];

  if (VERBOSE) { 
    printf("exiting compute_external_forces()\n");
    fflush(stdout);
  }
}

// function to simulate every active toy
void simulate_object_toys() {
  ToyObject obj;
  int i;

  // printf("\tRequest to simulate toy objects!\n");
  // make sure we have some active toys
  if (n_active_toys > 0) {
    // loop over all active toys
    for (i = 0; i < n_active_toys; ++i) {
      // printf("\tsimulating object %d\n", i);
      // simulate this particular toy
      simulate_object_polyball(&active_toys[i]);
    }
  }
}


void draw_circle(cu, cv, r, fill)
int cu, cv, r, fill;
{
  if (fill == NOFILL)
    XDrawArc(display, pixmap, gc, cu - r, cv - r, 2 * r, 2 * r, 0, 64 * 360);
  else
    XFillArc(display, pixmap, gc, cu - r, cv - r, 2 * r, 2 * r, 0, 64 * 360);
}

void draw_frames_dev()
{
#define FRAME_L 0.04
#define FRAME_T 0.045

  XSetForeground(display, gc, foreground);

  // the Cartesian frame
  /* x-axis */
  XDrawLine(display, pixmap, gc,
	    ConvertWorld2PixmapX(zoom, 0.0, kEnvironment), 
	    ConvertWorld2PixmapY(zoom, 0.0, kEnvironment),
	    ConvertWorld2PixmapX(zoom, (FRAME_L*4.0), kEnvironment),
	    ConvertWorld2PixmapY(zoom, 0.0, kEnvironment));
  XDrawString(display, pixmap, gc, 
	      ConvertWorld2PixmapX(zoom, FRAME_T*4.0, kEnvironment),
	      ConvertWorld2PixmapY(zoom, 0.0, kEnvironment), "x", 1);

  /* y-axis */
  XDrawLine(display, pixmap, gc,
	    ConvertWorld2PixmapX(zoom, 0.0, kEnvironment), 
	    ConvertWorld2PixmapY(zoom, 0.0, kEnvironment),
	    ConvertWorld2PixmapX(zoom, 0.0, kEnvironment), 
	    ConvertWorld2PixmapY(zoom, FRAME_L*4.0, kEnvironment));
  XDrawString(display, pixmap, gc, 
	      ConvertWorld2PixmapX(zoom, 0.0, kEnvironment), 
	      ConvertWorld2PixmapY(zoom, FRAME_T*4.0, kEnvironment), "y", 1);

  // the LEFT CSpace frame
  /* q1-axis */
  XDrawLine(display, pixmap, gc, 
	    T12LD(zoom, 0.0), T22LD(zoom, 0.0),
	    T12LD(zoom, FRAME_L*2.0*M_PI), T22LD(zoom, 0.0));
  XDrawString(display, pixmap, gc, 
	      T12LD(zoom, FRAME_T*2.0*M_PI), T22LD(zoom, 0.0), "q1", 2);

  /* q2-axis */
  XDrawLine(display, pixmap, gc,
	    T12LD(zoom, 0.0), T22LD(zoom, 0.0),
	    T12LD(zoom, 0.0), T22LD(zoom, FRAME_L*2.0*M_PI));
  XDrawString(display, pixmap, gc, 
	      T12LD(zoom, 0.0), T22LD(zoom, FRAME_T*2.0*M_PI), "q2", 2);

  XDrawString(display, pixmap, gc, 
	      T12LD(zoom, -0.75), T22LD(zoom, -3.5), "left", 4);
  XDrawString(display, pixmap, gc, 
	      T12LD(zoom, 0.25), T22LD(zoom, -3.5), "/", 1);
  XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
  XDrawString(display, pixmap, gc, 
	      T12LD(zoom, -0.2), T22LD(zoom, -3.5), "arm", 3);
  XSetForeground(display, gc, world_colors[EYE_COLOR].display_color);
  XDrawString(display, pixmap, gc, 
	      T12LD(zoom, 0.35), T22LD(zoom, -3.5), "eye", 3);

  XSetForeground(display, gc, foreground);
  // the RIGHT CSpace frame
  /* q1-axis */
  XDrawLine(display, pixmap, gc,
	    T12RD(zoom, 0.0), T22RD(zoom, 0.0), 
	    T12RD(zoom, FRAME_L*2.0*M_PI), T22RD(zoom, 0.0));
  XDrawString(display, pixmap, gc, 
	      T12RD(zoom, FRAME_T*2.0*M_PI), T22RD(zoom, 0.0), "q1", 2);

  /* q2-axis */
  XDrawLine(display, pixmap, gc,
	    T12RD(zoom, 0.0), T22RD(zoom, 0.0), 
	    T12RD(zoom, 0.0), T22RD(zoom, FRAME_L*2.0*M_PI));
  XDrawString(display, pixmap, gc, 
	      T12RD(zoom, 0.0), T22RD(zoom, FRAME_T*2.0*M_PI), "q2", 2);

  XDrawString(display, pixmap, gc, 
	      T12RD(zoom, -0.85), T22RD(zoom, -3.5), "right", 5);
  XDrawString(display, pixmap, gc,
	      T12RD(zoom, 0.25), T22RD(zoom, -3.5), "/", 1);
  XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
  XDrawString(display, pixmap, gc, 
	      T12RD(zoom, -0.15), T22RD(zoom, -3.5), "arm", 3);
  XSetForeground(display, gc, world_colors[EYE_COLOR].display_color);
  XDrawString(display, pixmap, gc, 
	      T12RD(zoom, 0.4), T22RD(zoom, -3.5), "eye", 3);

#undef FRAME_L // 0.04
#undef FRAME_T // 0.045
}


void draw_frames()
{
#define FRAME_L 0.04
#define FRAME_T 0.045

  XSetForeground(display, gc, foreground);

  // the Cartesian frame
  /* x-axis */
  XDrawLine(display, pixmap, gc,
	    ConvertWorld2PixmapX(zoom, 0.0, kEnvironment),
	    ConvertWorld2PixmapY(zoom, 0.0, kEnvironment),
	    ConvertWorld2PixmapX(zoom, (FRAME_L*4.0), kEnvironment),
	    ConvertWorld2PixmapY(zoom, 0.0, kEnvironment));
  XDrawString(display, pixmap, gc,
	      ConvertWorld2PixmapX(zoom, FRAME_T*4.0, kEnvironment),
	      ConvertWorld2PixmapY(zoom, 0.0, kEnvironment), "x", 1);

  /* y-axis */
  XDrawLine(display, pixmap, gc, 
	    ConvertWorld2PixmapX(zoom, 0.0, kEnvironment), 
	    ConvertWorld2PixmapY(zoom, 0.0, kEnvironment),
	    ConvertWorld2PixmapX(zoom, 0.0, kEnvironment), 
	    ConvertWorld2PixmapY(zoom, FRAME_L*4.0, kEnvironment));
  XDrawString(display, pixmap, gc,
	      ConvertWorld2PixmapX(zoom, 0.0, kEnvironment),
	      ConvertWorld2PixmapY(zoom, FRAME_T*4.0, kEnvironment), "y", 1);

#undef FRAME_L // 0.04
#undef FRAME_T // 0.045
}

void draw_frame(xform)
double xform[4][4]; {
#define FRAME_L 0.04
#define FRAME_T 0.045

  XSetForeground(display, gc, foreground);

  /* x-axis */
  XDrawLine(display, pixmap, gc, 
	    ConvertWorld2PixmapX(zoom, xform[0][3], kEnvironment),
	    ConvertWorld2PixmapY(zoom, xform[1][3], kEnvironment),
	    ConvertWorld2PixmapX(zoom, xform[0][3] + FRAME_L*xform[0][0],
				 kEnvironment),
	    ConvertWorld2PixmapY(zoom, xform[1][3] + FRAME_L*xform[1][0],
				 kEnvironment));
  XDrawString(display, pixmap, gc,
	      ConvertWorld2PixmapX(zoom, xform[0][3] + FRAME_T*xform[0][0],
				   kEnvironment),
	      ConvertWorld2PixmapY(zoom, xform[1][3] + FRAME_T*xform[1][0],
				   kEnvironment), "x", 1);

  /* y-axis */
  XDrawLine(display, pixmap, gc,
	    ConvertWorld2PixmapX(zoom, xform[0][3], kEnvironment),
	    ConvertWorld2PixmapY(zoom, xform[1][3], kEnvironment),
	    ConvertWorld2PixmapX(zoom, xform[0][3] + FRAME_L*xform[0][1],
				 kEnvironment),
	    ConvertWorld2PixmapY(zoom, xform[1][3] + FRAME_L*xform[1][1],
				 kEnvironment));
  XDrawString(display, pixmap, gc,
	      ConvertWorld2PixmapX(zoom, xform[0][3] + FRAME_T*xform[0][1],
				   kEnvironment),
	      ConvertWorld2PixmapY(zoom, xform[1][3] + FRAME_T*xform[1][1],
				   kEnvironment), "y", 1);

#undef FRAME_L // 0.04
#undef FRAME_T // 0.045
}

void draw_boundaries_dev()
{
  /******************************************************************/
  /**  draw world                                                  **/
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, MIN_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y_DEV, kEnvironment),
    ConvertWorld2PixmapX(zoom, MAX_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y_DEV, kEnvironment));
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, MAX_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y_DEV, kEnvironment),
    ConvertWorld2PixmapX(zoom, MAX_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y_DEV, kEnvironment));
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, MAX_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y_DEV, kEnvironment),
    ConvertWorld2PixmapX(zoom, MIN_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y_DEV, kEnvironment));
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, MIN_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y_DEV, kEnvironment),
    ConvertWorld2PixmapX(zoom, MIN_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y_DEV, kEnvironment));

  /* draw LEFT boundaries */
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc,
    T12LD(zoom, T1_MIN), T22LD(zoom, T2_MAX), T12LD(zoom, T1_MAX), T22LD(zoom, T2_MAX));
  XDrawLine(display, pixmap, gc,
    T12LD(zoom, T1_MAX), T22LD(zoom, T2_MAX), T12LD(zoom, T1_MAX), T22LD(zoom, T2_MIN));
  XDrawLine(display, pixmap, gc,
    T12LD(zoom, T1_MAX), T22LD(zoom, T2_MIN), T12LD(zoom, T1_MIN), T22LD(zoom, T2_MIN));
  XDrawLine(display, pixmap, gc,
    T12LD(zoom, T1_MIN), T22LD(zoom, T2_MIN), T12LD(zoom, T1_MIN), T22LD(zoom, T2_MAX));

  /* draw RIGHT boundaries */
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc,
    T12RD(zoom, T1_MIN), T22RD(zoom, T2_MAX), T12RD(zoom, T1_MAX), T22RD(zoom, T2_MAX));
  XDrawLine(display, pixmap, gc,
    T12RD(zoom, T1_MAX), T22RD(zoom, T2_MAX), T12RD(zoom, T1_MAX), T22RD(zoom, T2_MIN));
  XDrawLine(display, pixmap, gc,
    T12RD(zoom, T1_MAX), T22RD(zoom, T2_MIN), T12RD(zoom, T1_MIN), T22RD(zoom, T2_MIN));
  XDrawLine(display, pixmap, gc,
    T12RD(zoom, T1_MIN), T22RD(zoom, T2_MIN), T12RD(zoom, T1_MIN), T22RD(zoom, T2_MAX));
}


void draw_boundaries()
{
  /******************************************************************/
  /**  draw world                                                  **/
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, MIN_X, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y, kEnvironment),
    ConvertWorld2PixmapX(zoom, MAX_X, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y, kEnvironment));
  XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, MAX_X, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y, kEnvironment),
    ConvertWorld2PixmapX(zoom, MAX_X, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y, kEnvironment));
  XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, MAX_X, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y, kEnvironment),
    ConvertWorld2PixmapX(zoom, MIN_X, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y, kEnvironment));
  XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, MIN_X, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y, kEnvironment),
    ConvertWorld2PixmapX(zoom, MIN_X, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y, kEnvironment));

  XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, MIN_X + (MAX_X - MIN_X) / 2, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y, kEnvironment),
    ConvertWorld2PixmapX(zoom, MIN_X + (MAX_X - MIN_X) / 2, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y, kEnvironment));
}

// draw the NBINSxNBINS potential maps in their respective areas of the canvas
void draw_potential_maps_dev()
{
  int i, j, Cart_grey_index, left_arm_grey_index, right_arm_grey_index;
  double x, y, t1, t2;
  double Cart_bin_potential, left_arm_bin_potential, right_arm_bin_potential;

  for (i = 0; i < NBINS; ++i) {
    y = MAX_Y_DEV - i*YDELTA;
    t2 = T2_MAX - i*TDELTA;
    for (j = 0; j < NBINS; ++j) {
      x = MIN_X_DEV + j*XDELTA_DEV;
      t1 = T1_MIN + j*TDELTA;
      // user map grey level fill
      Cart_bin_potential = Rogers[0].world_map.potential_map[i][j];
      left_arm_bin_potential = Rogers[0].arm_map[LEFT].potential_map[i][j];
      right_arm_bin_potential = Rogers[0].arm_map[RIGHT].potential_map[i][j];

      // 0 <= grey indices <= 100
      Cart_grey_index = (int)(Cart_bin_potential * 100.0);
      left_arm_grey_index = (int)(left_arm_bin_potential * 100.0);
      right_arm_grey_index = (int)(right_arm_bin_potential * 100.0);

      // Cartesian Map
      // fill is either:
      //   a grey level depicting the user defined potential
      XSetForeground(display, gc, world_colors[Cart_grey_index].display_color);
      //   a user map perceived obstacle color, or
      if (Rogers[0].world_map.occupancy_map[i][j] == OBSTACLE)
        XSetForeground(display, gc,
          world_colors[Rogers[0].world_map.color_map[i][j]].display_color);
      else if (Rogers[0].world_map.occupancy_map[i][j] == DILATED_OBSTACLE)
        XSetForeground(display, gc,
          world_colors[Rogers[0].world_map.color_map[i][j]].display_color);
      //   a user defined goal
      else if (Rogers[0].world_map.occupancy_map[i][j] == GOAL)
        XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc,
        ConvertWorld2PixmapX(zoom, x, kEnvironment), ConvertWorld2PixmapY(zoom, y, kEnvironment),
        (ConvertWorld2PixmapR(zoom, XDELTA_DEV, kEnvironment) + 1), (ConvertWorld2PixmapR(zoom, YDELTA_DEV, kEnvironment) + 1));

      //      // each real obstacle should be outlined in the obstacle color
      //      if (real_world.occupancy_map[i][j] == OBSTACLE) {
      //  XSetForeground(display, gc,
      //         world_colors[real_world.color_map[i][j]].display_color);
      //  x = MIN_X + j*XDELTA; y = MAX_Y - i*YDELTA;
      //  XDrawRectangle(display, pixmap, gc, ConvertWorld2PixmapX(zoom,x), ConvertWorld2PixmapY(zoom,y),
      //         (ConvertWorld2PixmapR(zoom,XDELTA)), (ConvertWorld2PixmapR(zoom,YDELTA)));
      //      }

      // Left Arm Map
      XSetForeground(display, gc,
        world_colors[left_arm_grey_index].display_color);
      if (Rogers[0].arm_map[LEFT].occupancy_map[i][j] == OBSTACLE)
        XSetForeground(display, gc, world_colors[RED].display_color);
      else if (Rogers[0].arm_map[LEFT].occupancy_map[i][j] == GOAL)
        XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc,
        T12LD(zoom, t1), T22LD(zoom, t2), (T2DR(zoom, TDELTA) + 1), (T2DR(zoom, TDELTA) + 1));

      // Right Arm Map
      XSetForeground(display, gc,
        world_colors[right_arm_grey_index].display_color);
      if (Rogers[0].arm_map[RIGHT].occupancy_map[i][j] == OBSTACLE)
        XSetForeground(display, gc, world_colors[RED].display_color);
      else if (Rogers[0].arm_map[RIGHT].occupancy_map[i][j] == GOAL)
        XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc,
        T12RD(zoom, t1), T22RD(zoom, t2), (T2DR(zoom, TDELTA) + 1), (T2DR(zoom, TDELTA) + 1));
    }
  }
}

// draw the NBINSxNBINS potential maps in their respective areas of the canvas
void draw_potential_maps() {
  int i, j, k, Cart_grey_index, left_arm_grey_index, right_arm_grey_index;
  double x, y;
  double Cart_bin_potential, left_arm_bin_potential, right_arm_bin_potential;

  for (k = 0; k < numRoger; ++k) {
    for (i = 0; i < NBINS; ++i) {
      y = MAX_Y - i*YDELTA;
      for (j = 0; j < NBINS; ++j) {
        x = MIN_X + j*XDELTA;
        // user map grey level fill
        Cart_bin_potential = Rogers[k].world_map.potential_map[i][j];
        left_arm_bin_potential = Rogers[k].arm_map[LEFT].potential_map[i][j];
        right_arm_bin_potential = Rogers[k].arm_map[RIGHT].potential_map[i][j];

        // 0 <= grey indices <= 100
        Cart_grey_index = (int)(Cart_bin_potential * 100.0);
        left_arm_grey_index = (int)(left_arm_bin_potential * 100.0);
        right_arm_grey_index = (int)(right_arm_bin_potential * 100.0);

        // Cartesian Map
        // fill is either:
        //   a grey level depicting the user defined potential
        XSetForeground(display, gc, world_colors[Cart_grey_index].display_color);
        //   a user map perceived obstacle color, or
        if (Rogers[k].world_map.occupancy_map[i][j] == OBSTACLE)
          XSetForeground(display, gc,
            world_colors[Rogers[k].world_map.color_map[i][j]].display_color);
        else if (Rogers[k].world_map.occupancy_map[i][j] == DILATED_OBSTACLE)
          XSetForeground(display, gc,
            world_colors[Rogers[k].world_map.color_map[i][j]].display_color);
        //   a user defined goal
        else if (Rogers[k].world_map.occupancy_map[i][j] == GOAL)
          XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
        XFillRectangle(display, pixmap, gc, ConvertWorld2PixmapX(zoom, x, kEnvironment), ConvertWorld2PixmapY(zoom, y, kEnvironment),
          (ConvertWorld2PixmapR(zoom, XDELTA, kEnvironment) + 1), (ConvertWorld2PixmapR(zoom, YDELTA, kEnvironment) + 1));
      }
    }
  }
}

// TODO update this to handle new PolyBall objects definition
void draw_object_poly(obj)
ToyObject obj;
{
  int k;
  double xw, yw, theta, radius;

  // right now all objects are by default the same color
  XSetForeground(display, gc, world_colors[obj.color].display_color);

  // renders every ball that makes up polyball objects
  for (k = 0; k<obj.N; ++k) {
    theta = obj.vertices[k][1];
    radius = obj.vertices[k][0];
    // printf("drawing object fill circle k: %d, with radius: %.4f, and object radius: %.4f\n", k, radius, obj.radii[k]);
    xw = obj.position[X]
      + cos(obj.position[THETA]) * (radius*cos(theta))
      - sin(obj.position[THETA]) * (radius*sin(theta));
    yw = obj.position[Y]
      + sin(obj.position[THETA]) * (radius*cos(theta))
      + cos(obj.position[THETA]) * (radius*sin(theta));
    draw_circle(ConvertWorld2PixmapX(zoom, xw, kEnvironment), ConvertWorld2PixmapY(zoom, yw, kEnvironment),
      ConvertWorld2PixmapR(zoom, obj.radii[k], kEnvironment), FILL);
  }
}

void draw_object(obj)
Obj obj;
{
  XSetForeground(display, gc, world_colors[OBJECT_COLOR].display_color);
  draw_circle(ConvertWorld2PixmapX(zoom, obj.position[X], kEnvironment), ConvertWorld2PixmapY(zoom, obj.position[Y], kEnvironment),
    ConvertWorld2PixmapR(zoom, R_OBJ, kEnvironment), FILL);
}

/*****************************************************************/
// Not used by current PolyBall based code
// TODO: try to reuse for new object types and PolyBall definition
// could try to update this for the graphics. This function does the straight-line drawing 
//     that Dr. Grupen suggested as an alternative to rendering every ball
// void draw_toy(toy, fill)
// eObject toy;
// int fill;
// {
//   int i, j, n;
//   double x0, y0, x1, y1;
//   XPoint points[12];

//   if (VERBOSE) printf("         inside draw_toy()\n");
//   // BLACK: 0   RED: 102    BLUE: 105    GREEN: 108
//   if (toy.N == 1) { //special case: CIRCLE
//     XSetForeground(display, gc, toy.color[0].display_color);
//     if (VERBOSE) {
//       if (fill == FILL) printf("            i=0: drawing CIRCLE FILL\n");
//       else printf("            i=0: drawing CIRCLE NOFILL\n");
//     }
//     draw_circle(ConvertWorld2PixmapX(zoom, toy.pos[0], kEnvironment), ConvertWorld2PixmapY(zoom, toy.pos[1], kEnvironment),
//       ConvertWorld2PixmapR(zoom, R_TOY, kEnvironment), fill);
//   }
//   else if ((toy.N > 2) && (toy.N <= 12)) {// polygon w/N vertices
//     if (fill == FILL) {
//       XSetForeground(display, gc, toy.color[0].display_color);
//       // XFillRectangle(display, pixmap, gc,
//       //            ConvertWorld2PixmapX(zoom,x), ConvertWorld2PixmapY(zoom,y),
//       //      (ConvertWorld2PixmapR(zoom,XDELTA) + 1), (ConvertWorld2PixmapR(zoom,YDELTA) + 1));
//       if (VERBOSE) {
//         if (toy.N == 3)
//           printf("            i=1: drawing TRIANGLE FILL\n");
//         if (toy.N == 4)
//           printf("            i=2: drawing SQUARE FILL\n");
//         if (toy.N == 11)
//           printf("            i=3: drawing TRIANGLE RECEPTICAL FILL\n");
//         if (toy.N == 12)
//           printf("            i=4: drawing SQUARE RECEPTICAL FILL\n");
//       }
//       for (j = 0; j<toy.N; ++j) {
//         points[j].x = (short)ConvertWorld2PixmapX(zoom, (toy.pos[X] + toy.vertices[j][X]), kEnvironment);
//         points[j].y = (short)ConvertWorld2PixmapY(zoom, (toy.pos[Y] + toy.vertices[j][Y]), kEnvironment);
//       }

//       XFillPolygon(display, pixmap, gc, points,
//         toy.N, Complex, CoordModeOrigin);
//     }
//     else {
//       if (VERBOSE) {
//         if (toy.N == 3)
//           printf("            i=1: drawing TRIANGLE NOFILL\n");
//         if (toy.N == 4)
//           printf("            i=2: drawing SQUARE NOFILL\n");
//         if (toy.N == 11)
//           printf("            i=3: drawing TRIANGLE RECEPTICAL NOFILL\n");
//         if (toy.N == 12)
//           printf("            i=4: drawing SQUARE RECEPTICAL NOFILL\n");
//       }
//     }
//     for (j = 0; j<toy.N; ++j) {
//       XSetForeground(display, gc, toy.color[j].display_color);
//       x0 = toy.pos[X] + toy.vertices[j][X];
//       y0 = toy.pos[Y] + toy.vertices[j][Y];
//       if (j == toy.N - 1) {
//         x1 = toy.pos[X] + toy.vertices[0][X];
//         y1 = toy.pos[Y] + toy.vertices[0][Y];
//       }
//       else {
//         x1 = toy.pos[X] + toy.vertices[j + 1][X];
//         y1 = toy.pos[Y] + toy.vertices[j + 1][Y];
//       }
//       XDrawLine(display, pixmap, gc,
//         ConvertWorld2PixmapX(zoom, x0, kEnvironment), ConvertWorld2PixmapY(zoom, y0, kEnvironment), ConvertWorld2PixmapX(zoom, x1, kEnvironment), ConvertWorld2PixmapY(zoom, y1, kEnvironment));
//     }
//   }
//   else { // N=2
//     printf("illegal polygon\n");
//     //      return;
//   }

//   XSetForeground(display, gc, world_colors[GREEN].display_color);
//   /* x-axis */
//   XDrawLine(display, pixmap, gc,
//     ConvertWorld2PixmapX(zoom, (toy.pos[X] - 0.05), kEnvironment),
//     ConvertWorld2PixmapY(zoom, toy.pos[Y], kEnvironment),
//     ConvertWorld2PixmapX(zoom, (toy.pos[X] + 0.05), kEnvironment),
//     ConvertWorld2PixmapY(zoom, toy.pos[Y], kEnvironment));

//   /* y-axis */
//   XDrawLine(display, pixmap, gc,
//     ConvertWorld2PixmapX(zoom, toy.pos[X], kEnvironment),
//     ConvertWorld2PixmapY(zoom, (toy.pos[Y] - 0.05), kEnvironment),
//     ConvertWorld2PixmapX(zoom, toy.pos[X], kEnvironment),
//     ConvertWorld2PixmapY(zoom, (toy.pos[Y] + 0.05), kEnvironment));
// }

// void draw_active_toys()
// {
//   int i, j, n;
//   double x0, y0, x1, y1;
//   XPoint points[12];

//   // BLACK: 0   RED: 102    BLUE: 105    GREEN: 108
//   if (VERBOSE) printf("  inside draw_active_toys()\n");
//   for (i = 0; i<n_active_toys; ++i) {
//     draw_toy(active_toys[i], FILL);
//   }
//   if (VERBOSE) printf("\n");
// }

// void draw_toybox()
// {
//   int i;

//   // BLACK: 0   RED: 102    BLUE: 105    GREEN: 108
//   if (VERBOSE)  printf("   inside draw_toybox():\n");
//   for (i = 0; i<N_TOY_TYPES; ++i) {
//     if (i == input_toy_type) {
//       if (VERBOSE) printf("      before draw_toy()\n");
//       draw_toy(toybox[i], FILL);
//       if (VERBOSE) printf("      returned from draw_toy()\n");
//     }
//     else {
//       if (VERBOSE) printf("      before draw_toy()\n");
//       draw_toy(toybox[i], NOFILL);
//       if (VERBOSE) printf("      returned from draw_toy()\n");
//     }
//   }
//   if (VERBOSE) printf("\n");
// }

// Not used by current PolyBall based code
// TODO: try to reuse for new object types and PolyBall definition
/*****************************************************************/

void draw_toy_object(obj)
ToyObject obj;
{
  int i, j, n;
  double xw, yw;
  double draw_rad, draw_theta;
  XPoint points[NUM_DRAW_VERTS];

  if (VERBOSE) {
    // printf("\tinside draw_toy()\n");
  }

  if (obj.id == CIRCLE) { //special case: CIRCLE
    draw_object_poly(obj);
  } else {
    printf("\tInside of draw_toy for not a circle!!!!\n");
    printf("\tdrawing object id: %d, with %d number of draw verts\n", obj.id, obj.N_draw);
    
    XSetForeground(display, gc, world_colors[obj.color].display_color);


    for (j = 0; j<obj.N_draw; ++j) {
      draw_rad = obj.draw_verts[j][0];
      draw_theta = obj.draw_verts[j][1];
      xw = obj.position[X] + cos(obj.position[THETA]) * (draw_rad*cos(draw_theta)) - sin(obj.position[THETA]) * (draw_rad*sin(draw_theta));
      yw = obj.position[Y] + sin(obj.position[THETA]) * (draw_rad*cos(draw_theta)) + cos(obj.position[THETA]) * (draw_rad*sin(draw_theta));
      points[j].x = (short)ConvertWorld2PixmapX(zoom, xw, kEnvironment);
      points[j].y = (short)ConvertWorld2PixmapY(zoom, yw, kEnvironment);
    }
    
    XFillPolygon(display, pixmap, gc, points, obj.N_draw, Complex, CoordModeOrigin);
    // int color = obj.color;
    // obj.color = YELLOW;
    // draw_object_poly(obj);
    // obj.color = color;
  }
}

void draw_object_toys() {
  ToyObject obj;
  int i;

  // printf("\tRequest to draw toy objects!\n");
  // make sure we have some active toys
  if (n_active_toys > 0) {
    // loop over all active toys
    for (i = 0; i < n_active_toys; ++i) {
      // printf("\tsimulating object %d\n", i);
      // copy a toy from the array of active toys
      copy_object(i, &obj, active_toys);
      // simulate this particular toy
      draw_toy_object(obj);
    }
  }
}

void draw_eye(base, eye, side)
Base base;
Eye eye;
int side;
{
  double px, py;
  double rx, ry, from_x, from_y, to_x, to_y;
  double lambda_x, lambda_y;
  int xbin, ybin;

  // Definie delta_x and delta_y size given the environment
  float x_delta, y_delta;
  double max_x, min_x, max_y, min_y;

  if (kEnvironment == DEVELOPMENT) {
    x_delta = XDELTA_DEV;
    y_delta = YDELTA_DEV;
    max_x = MAX_X_DEV;
    min_x = MIN_X_DEV;
    max_y = MAX_Y_DEV;
    min_y = MIN_Y_DEV;
  }
  else {
    x_delta = XDELTA;
    y_delta = YDELTA;
    max_x = MAX_X;
    min_x = MIN_X;
    max_y = MAX_Y;
    min_y = MIN_Y;
  }

  px = base.wTb[0][0] * eye.position[0] + base.wTb[0][1] * eye.position[1] +
    base.wTb[0][3];
  py = base.wTb[1][0] * eye.position[0] + base.wTb[1][1] * eye.position[1] +
    base.wTb[1][3];

  from_x = px; from_y = py;
  rx = cos(base.theta + eye.theta);
  ry = sin(base.theta + eye.theta);

  //trace the eye direction till you hit an obstacle
  to_x = from_x;
  to_y = from_y;

  while (to_x < max_x && to_x > min_x && to_y < max_y && to_y > min_y) {
    //get bin for location
    ybin = (int)((max_y - to_y) / y_delta);
    xbin = (int)((to_x - min_x) / x_delta);

    //check for obstacle collision
    if (Rogers[0].world_map.occupancy_map[ybin][xbin] == OBSTACLE) break;

    to_x += rx * 0.001;
    to_y += ry * 0.001;
  }

  XSetForeground(display, gc, world_colors[GAZE_COLOR].display_color);
  XDrawLine(display, pixmap, gc,
	    ConvertWorld2PixmapX(zoom, from_x, kEnvironment),
	    ConvertWorld2PixmapY(zoom, from_y, kEnvironment),
	    ConvertWorld2PixmapX(zoom, to_x, kEnvironment),
	    ConvertWorld2PixmapY(zoom, to_y, kEnvironment));

  //                               0, 1, 2, 3,  4,  5,  6,  7
  // SATURATION FLAG [NDOF];  // [LW,RW,LE,RE,LAS,LAE,RAS,RAE]
  XSetForeground(display, gc, foreground);
  if (((side==LEFT) && (saturation_flag[2] == TRUE)) ||
      ((side==RIGHT) && (saturation_flag[3] == TRUE))) {
    XSetForeground(display, gc, world_colors[SAT_COLOR].display_color);
  }

  draw_circle(ConvertWorld2PixmapX(zoom, px, kEnvironment),
	      ConvertWorld2PixmapY(zoom, py, kEnvironment),
	      ConvertWorld2PixmapR(zoom, R_EYE, kEnvironment), NOFILL);
  draw_circle(ConvertWorld2PixmapX(zoom, px + (R_EYE - 0.8*R_PUPIL)*rx,
				   kEnvironment),
	      ConvertWorld2PixmapY(zoom, py + (R_EYE - 0.8*R_PUPIL)*ry,
				   kEnvironment),
	      ConvertWorld2PixmapR(zoom, R_PUPIL, kEnvironment), FILL);

  XSetForeground(display, gc, foreground);
  }

void draw_image_dev(eye)
int eye;
{
  register int i, color, dx;
  float scaled_left_image_x;
  float scaled_right_image_x;

  // Scale the position of image_x such that the image center is scaled
  // linearly with zoom factor
  scaled_left_image_x = zoom * LEFT_IMAGE_X + 
    ((float)(IMAGE_WIDTH) / 2.0) * (zoom - 1);
  scaled_right_image_x = zoom * RIGHT_IMAGE_X + 
    ((float)(IMAGE_WIDTH) / 2.0) * (zoom - 1);

  XSetForeground(display, gc, foreground);
  if (eye == LEFT)
    XDrawRectangle(display, pixmap, gc,
    (scaled_left_image_x - 1), (zoom*IMAGE_Y - 1),
      (IMAGE_WIDTH + 1), (PIXEL_HEIGHT + 1));
  else if (eye == RIGHT)
    XDrawRectangle(display, pixmap, gc,
    (scaled_right_image_x - 1), (zoom*IMAGE_Y - 1),
      (IMAGE_WIDTH + 1), (PIXEL_HEIGHT + 1));

  for (i = 0, dx = 0; i < NPIXELS; i++, dx += PIXEL_WIDTH) {
    color = RogersBodys[0].eyes[eye].image[i];
    XSetForeground(display, gc, world_colors[color].display_color);

    if (eye == LEFT)
      XFillRectangle(display, pixmap, gc,
      (scaled_left_image_x + dx), zoom*IMAGE_Y, PIXEL_WIDTH, PIXEL_HEIGHT);
    else if (eye == RIGHT)
      XFillRectangle(display, pixmap, gc,
      (scaled_right_image_x + dx), zoom*IMAGE_Y, PIXEL_WIDTH, PIXEL_HEIGHT);
  }
}

void draw_image(eye, rogerID)
int eye;
int rogerID;
{
  int i, color, dx;
  float scaled_left_image_x_1;
  float scaled_right_image_x_1;
  float scaled_left_image_x_2;
  float scaled_right_image_x_2;

  // Scale the position of image_x such that the image center is scaled
  // linearly with zoom factor
  scaled_left_image_x_1 = zoom * LEFT_IMAGE_X_1 + 
    ((float)(IMAGE_WIDTH) / 2.0) * (zoom - 1);
  scaled_right_image_x_1 = zoom * RIGHT_IMAGE_X_1 + 
    ((float)(IMAGE_WIDTH) / 2.0) * (zoom - 1);
  scaled_left_image_x_2 = zoom * LEFT_IMAGE_X_2 + 
    ((float)(IMAGE_WIDTH) / 2.0) * (zoom - 1);
  scaled_right_image_x_2 = zoom * RIGHT_IMAGE_X_2 + 
    ((float)(IMAGE_WIDTH) / 2.0) * (zoom - 1);

  if (rogerID == 0) {
    XSetForeground(display, gc, foreground);
    if (eye == LEFT)
      XDrawRectangle(display, pixmap, gc, scaled_left_image_x_1-1,
		     (zoom*IMAGE_Y-1), IMAGE_WIDTH+1, PIXEL_HEIGHT+1);
    else if (eye == RIGHT)
      XDrawRectangle(display, pixmap, gc, scaled_right_image_x_1-1,
		     (zoom*IMAGE_Y-1), IMAGE_WIDTH+1, PIXEL_HEIGHT+1);

    for (i = 0, dx = 0; i < NPIXELS; i++, dx += PIXEL_WIDTH) {
      color = RogersBodys[rogerID].eyes[eye].image[i];
      //    XSetForeground(display, gc, color);

      // Commented out by Dan.
      //printf("color=%d\n", color);

      //    if (color > 99) XSetForeground(display, gc, background);
      //    else if (intensity >= 0)
      //      XSetForeground(display, gc, image_color[intensity]);
      //    if (color == ARM_COLOR)
      //      XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
      //    else if (color == ARM_COLOR)
      //      XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
      //    else if (color == OBJECT_COLOR)
      //      XSetForeground(display, gc,
      //         world_colors[OBJECT_COLOR].display_color);
      XSetForeground(display, gc, world_colors[color].display_color);

      if (eye == LEFT)
        XFillRectangle(display, pixmap, gc, 
		       scaled_left_image_x_1+dx, zoom*IMAGE_Y, 
		       PIXEL_WIDTH, PIXEL_HEIGHT);
      else if (eye == RIGHT)
        XFillRectangle(display, pixmap, gc,
		       scaled_right_image_x_1+dx, zoom*IMAGE_Y, 
		       PIXEL_WIDTH, PIXEL_HEIGHT);
    }
  }
  else if (rogerID == 1) {
    XSetForeground(display, gc, foreground);
    if (eye == LEFT)
      XDrawRectangle(display, pixmap, gc, 
		     scaled_left_image_x_2 - 1, (zoom*IMAGE_Y - 1),
		     IMAGE_WIDTH + 1, PIXEL_HEIGHT + 1);
    else if (eye == RIGHT)
      XDrawRectangle(display, pixmap, gc, 
		     scaled_right_image_x_2 - 1, (zoom*IMAGE_Y - 1),
		     IMAGE_WIDTH + 1, PIXEL_HEIGHT + 1);

    for (i = 0, dx = 0; i < NPIXELS; i++, dx += PIXEL_WIDTH) {
      color = RogersBodys[rogerID].eyes[eye].image[i];
      //    XSetForeground(display, gc, color);

      // Commented out by Dan.
      //printf("color=%d\n", color);

      //    if (color > 99) XSetForeground(display, gc, background);
      //    else if (intensity >= 0)
      //      XSetForeground(display, gc, image_color[intensity]);
      //    if (color == ARM_COLOR)
      //      XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
      //    else if (color == ARM_COLOR)
      //      XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
      //    else if (color == OBJECT_COLOR)
      //      XSetForeground(display, gc,
      //         world_colors[OBJECT_COLOR].display_color);
      XSetForeground(display, gc, world_colors[color].display_color);

      if (eye == LEFT)
        XFillRectangle(display, pixmap, gc,
		       scaled_left_image_x_2 + dx, zoom*IMAGE_Y,
		       PIXEL_WIDTH, PIXEL_HEIGHT);
      else if (eye == RIGHT)
        XFillRectangle(display, pixmap, gc, 
		       scaled_right_image_x_2 + dx, zoom*IMAGE_Y,
		       PIXEL_WIDTH, PIXEL_HEIGHT);
    }
  }
}

void draw_roger(mobile_base, arms, eyes, rogerID)
Base mobile_base;
Arm arms[NARMS][NARM_FRAMES];
Eye eyes[NEYES];
int rogerID;
{
  int i, j, k;
  double r_b[4], r_w[4], fhat[2];
  double theta1, theta2, mag;
  double temp0[4][4], temp1[4][4];
  XPoint rect[4];
  void draw_history();

  /*************************************************************************/
  // SATURATION FLAG [NDOF];  // [LW,RW,LE,RE,LAS,LAE,RAS,RAE]

  /*************************************************************************/

  /* DRAW MOBILE BASE     ************************************************ */

  XSetForeground(display, gc, foreground);
  draw_circle(ConvertWorld2PixmapX(zoom, mobile_base.wTb[0][3], kEnvironment),
	      ConvertWorld2PixmapY(zoom, mobile_base.wTb[1][3], kEnvironment), 
	      ConvertWorld2PixmapR(zoom, R_BASE, kEnvironment), NOFILL);
  // draw contact forces on object from body
  mag = sqrt(SQR(mobile_base.extForce[X]) + SQR(mobile_base.extForce[Y]));
  if (mag > 0.0) {
    fhat[X] = mobile_base.extForce[X] / mag;
    fhat[Y] = mobile_base.extForce[Y] / mag;
    XDrawLine(display, pixmap, gc,
      ConvertWorld2PixmapX(zoom, mobile_base.x-R_BASE*fhat[X],kEnvironment),
      ConvertWorld2PixmapY(zoom, mobile_base.y-R_BASE*fhat[Y],kEnvironment),
      ConvertWorld2PixmapX(zoom, mobile_base.x-(R_BASE+0.08)*fhat[X],
			   kEnvironment),
      ConvertWorld2PixmapY(zoom, mobile_base.y-(R_BASE+0.08)*fhat[Y],
			   kEnvironment));
  }

  //  DRAW WHEELS: draw_wheels();
  /***** left wheel - actuated, subject to limits from motor model *********/
  if ((rogerID==0) && (saturation_flag[0] == TRUE))
    XSetForeground(display, gc, world_colors[SAT_COLOR].display_color);

  r_b[0] = R_BASE / 2.0; r_b[1] = R_BASE + WHEEL_THICKNESS; 
  r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment), 
	      ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment), 
	      ConvertWorld2PixmapR(zoom, WHEEL_THICKNESS, kEnvironment), FILL);
  r_b[0] = -R_BASE / 2.0; r_b[1] = R_BASE + WHEEL_THICKNESS; 
  r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment), 
	      ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment),
	      ConvertWorld2PixmapR(zoom, WHEEL_THICKNESS, kEnvironment), FILL);

  r_b[0] = R_BASE / 2.0; r_b[1] = R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[0].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[0].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  r_b[0] = R_BASE / 2.0; r_b[1] = (R_BASE + 2 * WHEEL_THICKNESS); 
  r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[1].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[1].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  r_b[0] = -R_BASE / 2.0; r_b[1] = (R_BASE + 2 * WHEEL_THICKNESS);
  r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[2].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[2].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  r_b[0] = -R_BASE / 2.0; r_b[1] = R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[3].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[3].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  XFillPolygon(display, pixmap, gc, rect, 4, Convex, CoordModeOrigin);

  XSetForeground(display, gc, foreground);
  /**************************************************************************/

  /***** right wheel ********************************************************/
  if ((rogerID==0) && (saturation_flag[1] == TRUE))
    XSetForeground(display, gc, world_colors[SAT_COLOR].display_color);

  r_b[0]=R_BASE/2.0; r_b[1]=-R_BASE-WHEEL_THICKNESS; r_b[2]=0.0; r_b[3]=1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment), 
	      ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment), 
	      ConvertWorld2PixmapR(zoom, WHEEL_THICKNESS, kEnvironment), FILL);
  r_b[0] = -R_BASE/2.0; r_b[1]=-R_BASE-WHEEL_THICKNESS; r_b[2]=0.0; r_b[3]=1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment), 
	      ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment),
	      ConvertWorld2PixmapR(zoom, WHEEL_THICKNESS, kEnvironment), FILL);

  r_b[0] = R_BASE/2.0; r_b[1] =-R_BASE; r_b[2]=0.0; r_b[3]=1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[0].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[0].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  r_b[0]=R_BASE/2.0; r_b[1]= -(R_BASE+2*WHEEL_THICKNESS);
  r_b[2]=0.0; r_b[3]=1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[1].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[1].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  r_b[0]= -R_BASE/2.0; r_b[1]= -(R_BASE+2*WHEEL_THICKNESS);
  r_b[2]=0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[2].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[2].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  r_b[0] = -R_BASE / 2.0; r_b[1] = -R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[3].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[3].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  XFillPolygon(display, pixmap, gc, rect, 4, Convex, CoordModeOrigin);
  XSetForeground(display, gc, foreground);
  /*************************************************************************/

  /*************************************************************************/
  /* draw eyes */
  for (i = 0; i < NEYES; i++)
  {
    draw_eye(mobile_base, eyes[i], i);

    /******************************************************************/
    /* draw displays **************************************************/
    /* draw coordinate in configuration space for left and right eyes */
    if (kEnvironment == DEVELOPMENT) {
      XSetForeground(display, gc, world_colors[EYE_COLOR].display_color);
      if (i == LEFT)
        XFillRectangle(display, pixmap, gc, T12LD(zoom, eyes[i].theta),
        T22LD(zoom, 0.0), (T2DR(zoom, TDELTA) + 1), (T2DR(zoom, TDELTA) + 1));
      else if (i == RIGHT)
        XFillRectangle(display, pixmap, gc, T12RD(zoom, eyes[i].theta),
        T22RD(zoom, 0.0), (T2DR(zoom, TDELTA) + 1), (T2DR(zoom, TDELTA) + 1));
    }
  }
  /******************************************************************/
  /* draw arms */
  for (j = 0; j<NARMS; j++) {
    XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);

    SIMmatXmat(mobile_base.wTb, arms[j][0].iTj, temp0);

    for (i = 1; i<NARM_FRAMES; i++) {
      SIMmatXmat(temp0, arms[j][i].iTj, temp1);
      XDrawLine(display, pixmap, gc, 
		ConvertWorld2PixmapX(zoom, temp0[0][3], kEnvironment),
		ConvertWorld2PixmapY(zoom, temp0[1][3], kEnvironment), 
		ConvertWorld2PixmapX(zoom, temp1[0][3], kEnvironment),
		ConvertWorld2PixmapY(zoom, temp1[1][3], kEnvironment));
      if (i == (NARM_FRAMES - 1))
        draw_circle(ConvertWorld2PixmapX(zoom, temp1[0][3], kEnvironment),
		    ConvertWorld2PixmapY(zoom, temp1[1][3], kEnvironment),
		    ConvertWorld2PixmapR(zoom, R_TACTILE, kEnvironment), FILL);
      else {
        // ASSIGN JOINT SATURATION COLOR HERE - ONLY ROGER 0
        //                               0, 1, 2, 3,  4,  5,  6,  7
	// SATURATION FLAG [NDOF];  // [LW,RW,LE,RE,LAS,LAE,RAS,RAE]
	if (rogerID==0) {
	  if (((j==0) && (saturation_flag[4] == TRUE)) ||
	      ((j==1) && (saturation_flag[5] == TRUE)) ||
	      ((j==0) && (saturation_flag[6] == TRUE)) ||
	      ((j==1) && (saturation_flag[7] == TRUE))) {
	    XSetForeground(display, gc, world_colors[SAT_COLOR].display_color);
	  }
	}
	draw_circle(ConvertWorld2PixmapX(zoom, temp1[0][3], kEnvironment),
		    ConvertWorld2PixmapY(zoom, temp1[1][3], kEnvironment),
		    ConvertWorld2PixmapR(zoom, R_JOINT, kEnvironment),
		    NOFILL);

	XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
	SIMcopy_matrix(temp1, temp0);
      }
    }

    // draw endpoint forces
    mag = sqrt(SQR(arms[j][NARM_FRAMES - 1].extForce[X]) +
	       SQR(arms[j][NARM_FRAMES - 1].extForce[Y]));
    if (mag>0.0) {
      XSetForeground(display, gc, foreground);
      fhat[X] = arms[j][NARM_FRAMES-1].extForce[X]/mag;
      fhat[Y] = arms[j][NARM_FRAMES-1].extForce[Y]/mag;
      XDrawLine(display, pixmap, gc,
		ConvertWorld2PixmapX(zoom,(temp1[0][3] + R_TACTILE*fhat[X]),
				     kEnvironment),
		ConvertWorld2PixmapY(zoom,(temp1[1][3] + R_TACTILE*fhat[Y]),
				     kEnvironment),
		ConvertWorld2PixmapX(zoom,(temp1[0][3] + 2*R_TACTILE*fhat[X]),
				     kEnvironment),
		ConvertWorld2PixmapY(zoom,(temp1[1][3] + 2*R_TACTILE*fhat[Y]),
				     kEnvironment));
    }

    /******************************************************************/
    /* draw displays **************************************************/
    /* draw coordinate in configuration space for left and right arms */
    if (kEnvironment == DEVELOPMENT) {
      if (j == LEFT)
        XFillRectangle(display, pixmap, gc,
           T12LD(zoom, arms[j][1].theta), T22LD(zoom, arms[j][2].theta),
           (T2DR(zoom, TDELTA)+1), (T2DR(zoom, TDELTA)+1));
      else if (j == RIGHT)
        XFillRectangle(display, pixmap, gc,
           T12RD(zoom, arms[j][1].theta), T22RD(zoom, arms[j][2].theta),
           (T2DR(zoom, TDELTA)+1), (T2DR(zoom, TDELTA)+1));
    }
  }
  if (HISTORY) draw_history(rogerID);

  /* visual images *************************************************/
  if (kEnvironment == ARENA) {
    if (numRoger <= 2) {
      draw_image(LEFT, rogerID);
      draw_image(RIGHT, rogerID);
    }

  } else {
    draw_image_dev(LEFT);
    draw_image_dev(RIGHT);
  }
}


void draw_all_dev()
{
  int n;
  char buffer[64];
  void draw_frames_dev(), draw_boundaries_dev(), draw_potential_maps_dev();
  void draw_toybox(), draw_active_toys();

  x_clear();

  int i;

  // draw localizability/manipulability ellipses on top (i.e. render them last)
  // draw harmonic function streamlines on bottom (i.e. render them first)

  //  draw_ellipse(manipulator(LEFT));
  //  draw_ellipse(manipulator(RIGHT));

  // XSetForeground(display, gc, goal_color);
  //  draw_ellipse(observation);
  //  draw_ellipse(spatial_goals[CENTROID]);
  //  draw_ellipse(spatial_goals[LEFT_EDGE]);
  //  draw_ellipse(spatial_goals[RIGHT_EDGE]);

  //  if (p_index==6) draw_ellipse(grasp_goal);
  //  XSetForeground(display, gc, target_color);
  //  draw_ellipse(target);

  draw_potential_maps_dev();
  draw_boundaries_dev();
  draw_frames_dev();
  
  
  // if (VERBOSE) printf("before draw_toybox()\n");
  // draw_toybox();

  // TODO update to iterate over all active toys
  // draw_object_poly(toy);

  draw_object_toys();

  for (i = 0; i < numRoger; ++i) {
    // we want to draw saturated motors in red inside draw_roger()
    draw_roger(RogersBodys[i].mobile_base, RogersBodys[i].arms,
	       RogersBodys[i].eyes, i);
  }

  XSetForeground(display, gc, foreground);
  n = sprintf(buffer, "total elapsed time = %6.3lf", simtime);
  XDrawString(display, pixmap, gc, 
	      ConvertWorld2PixmapX(zoom, 2.2, kEnvironment),
	      ConvertWorld2PixmapY(zoom, 1.65, kEnvironment), buffer, n);

  if (draw_visual) {
    n = sprintf(buffer, "VISUALIZE: ON");
    XDrawString(display, pixmap, gc, 
		ConvertWorld2PixmapX(zoom, 2.6, kEnvironment), 
		ConvertWorld2PixmapY(zoom, 1.8, kEnvironment), buffer, n);
    // Draws stuff with visualize
    // initialize_room_overlay(&Rogers[0]);
  }
  else {
    n = sprintf(buffer, "VISUALIZE: OFF");
    XDrawString(display, pixmap, gc, 
		ConvertWorld2PixmapX(zoom, 2.6, kEnvironment), 
		ConvertWorld2PixmapY(zoom, 1.8, kEnvironment), buffer, n);
  }

  if (COUPLED_DYNAMICS) {
    n = sprintf(buffer, "WHOLE-BODY DYNAMICS");
    XDrawString(display, pixmap, gc, 
		ConvertWorld2PixmapX(zoom, -3.625, kEnvironment), 
		ConvertWorld2PixmapY(zoom, 1.8, kEnvironment), buffer, n);
  }
  else {
    n = sprintf(buffer, "BLOCK DIAGONAL DYNAMICS");
    XDrawString(display, pixmap, gc, 
		ConvertWorld2PixmapX(zoom, -3.875, kEnvironment), 
		ConvertWorld2PixmapY(zoom, 1.8, kEnvironment), buffer, n);
  }

  if (ACTUATE_BASE) {
    n = sprintf(buffer, "BASE: ON");
    XDrawString(display, pixmap, gc, 
		ConvertWorld2PixmapX(zoom, -4.0, kEnvironment),
		ConvertWorld2PixmapY(zoom, 1.65, kEnvironment), buffer, n);
  }
  else {
    n = sprintf(buffer, "BASE: OFF");
    XDrawString(display, pixmap, gc, 
		ConvertWorld2PixmapX(zoom, -4.0, kEnvironment),
		ConvertWorld2PixmapY(zoom, 1.65, kEnvironment), buffer, n);
  }

  if (ACTUATE_ARMS) {
    n = sprintf(buffer, "ARMS: ON");
    XDrawString(display, pixmap, gc,
		ConvertWorld2PixmapX(zoom, -3.3125, kEnvironment),
		ConvertWorld2PixmapY(zoom, 1.65, kEnvironment), buffer, n);
  }
  else {
    n = sprintf(buffer, "ARMS: OFF");
    XDrawString(display, pixmap, gc, 
		ConvertWorld2PixmapX(zoom, -3.3125, kEnvironment),
		ConvertWorld2PixmapY(zoom, 1.65, kEnvironment), buffer, n);
  }

  if (ACTUATE_EYES) {
    n = sprintf(buffer, "EYES: ON");
    XDrawString(display, pixmap, gc, 
		ConvertWorld2PixmapX(zoom, -2.625, kEnvironment), 
		ConvertWorld2PixmapY(zoom, 1.65, kEnvironment), buffer, n);
  }
  else {
    n = sprintf(buffer, "EYES: OFF");
    XDrawString(display, pixmap, gc, 
		ConvertWorld2PixmapX(zoom, -2.625, kEnvironment),
		ConvertWorld2PixmapY(zoom, 1.65, kEnvironment), buffer, n);
  }

  // printf("About to draw requests\n");
  HandleDrawingRequests(&Rogers[0]);
  // printf("drew")
  x_expose();
}

void draw_all()
{
  int n, i;
  char buffer[64];
  void draw_potential_maps(), draw_object(), draw_roger();

  x_clear();

  draw_potential_maps();
  for (i = 0; i < numRoger; ++i) {
    if(socknew[i] > 0) {
      HandleDrawingRequests(&Rogers[i]);
    }
  }
  draw_boundaries();
  draw_frames();
  
  // TODO loop over active toys array
  // draw_object_poly(toy);
  draw_object_toys();
  
  
  
  
  for (i = 0; i < numRoger; ++i) {
    // we want to draw saturated motors in red inside draw_roger()
    draw_roger(RogersBodys[i].mobile_base, RogersBodys[i].arms, 
	       RogersBodys[i].eyes, i);
  }

  n = sprintf(buffer, "total elapsed time = %6.3lf", simtime);
  XSetForeground(display, gc, foreground);
  XDrawString(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, 3.0, kEnvironment), ConvertWorld2PixmapY(zoom, 1.8, kEnvironment), buffer, n);



  x_expose();
}


void draw_history(rogerID)
int rogerID;
{
  int h;
  // Definie delta_x and delta_y size given the environment
  float x_delta, y_delta;
  if (kEnvironment == DEVELOPMENT) {
    x_delta = XDELTA_DEV;
    y_delta = YDELTA_DEV;
  }
  else {
    x_delta = XDELTA;
    y_delta = YDELTA;
  }

  // draw history of all Cartesian arm postures
  XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);

  for (h = 0; h < history_ptr[rogerID]; ++h) {
    // draw Cartesian history of the mobile platform
    XFillRectangle(display, pixmap, gc,
      ConvertWorld2PixmapX(zoom, (history[rogerID][h].base_pos[0] - x_delta / 4.0), kEnvironment),
      ConvertWorld2PixmapY(zoom, (history[rogerID][h].base_pos[1] - y_delta / 4.0), kEnvironment),
      ConvertWorld2PixmapR(zoom, x_delta / 2.0, kEnvironment),
      ConvertWorld2PixmapR(zoom, y_delta / 2.0, kEnvironment));
  }
}

#define VMAG 0.5

// observations in world coordinates
void draw_estimate0(scale, est)
double scale;
Estimate est;
{
  draw_circle(ConvertWorld2PixmapX(ZOOM_SCALE, est.state[X], kEnvironment),
    ConvertWorld2PixmapY(ZOOM_SCALE, est.state[Y], kEnvironment),
    ConvertWorld2PixmapR(ZOOM_SCALE, R_BASE, kEnvironment), NOFILL);

  x_draw_line(BLUE, est.state[X], est.state[Y],
    (est.state[X] + VMAG*est.state[XDOT]),
    (est.state[Y] + VMAG*est.state[YDOT]));
}

#define FRAME_L 0.08

// observations in world coordinates
void draw_observation(obs)
Observation obs;
{
  double a, b, c, root[2];

  double dx, dy, mag, eigenvalue[2], eigenvector[2][2];
  double ref_w[4], ref_b[4], wTb[4][4], bTw[4][4];
  double theta, dx0, dy0, dx1, dy1;

  // printf("inside draw_observation()\n");
  // printf("x=%6.4lf y=%6.4lf \n\n", obs.pos[X], obs.pos[Y]);

  //printf("   %lf %lf\n", obs.cov[0][0], obs.cov[0][1]);
  //printf("   %lf %lf\n", obs.cov[1][0], obs.cov[1][1]);

  // DRAW THE CURRENT ESTIMATED STATE VARIABLES AND THEIR VELOCITIES
  //  draw_circle(ConvertWorld2PixmapX(zoom,est.state[X]), ConvertWorld2PixmapY(zoom,est.state[Y]),
  //            ConvertWorld2PixmapR(zoom,R_JOINT), FILL);

  // cov = [A  B] => det |JJt| = a(lambda)^2 + b(lambda) + c
  //       [B  C]
  a = 1.0;
  b = -(obs.cov[0][0] + obs.cov[1][1]);
  c = obs.cov[0][0] * obs.cov[1][1] - obs.cov[1][0] * obs.cov[0][1];

  root[0] = (-b + sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);
  root[1] = (-b - sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);

  if (fabs(obs.cov[1][0]) > 0.0001) {
    dy = 1.0;
    dx = -(obs.cov[1][1] - root[0]) / obs.cov[1][0];

    mag = sqrt(SQR(dx) + SQR(dy));
    //    eigenvalue[0] = scale*sqrt(root[0]);
    eigenvalue[0] = sqrt(root[0]);
    eigenvector[0][0] = dx / mag;
    eigenvector[0][1] = dy / mag;

    // the eigenvector for root 1
    //    eigenvalue[1] = scale*sqrt(root[1]);
    eigenvalue[1] = sqrt(root[1]);
    eigenvector[1][0] = -eigenvector[0][1];
    eigenvector[1][1] = eigenvector[0][0];
  }
  // when ball is directly in front of Roger:
  else {
    // set eigenvalue 0 to be the greater root
    if (fabs(root[0]) > fabs(root[1])) {
      eigenvalue[0] = sqrt(root[0]);
      eigenvalue[1] = sqrt(root[1]);
    }
    else {
      eigenvalue[0] = sqrt(root[1]);
      eigenvalue[1] = sqrt(root[0]);
    }

    // when cov item A > cov item C, Roger is facing either of the green
    // walls; when cov item A < cov item C, Roger is facing either of the
    // blue walls
    if (fabs(obs.cov[0][0]) > fabs(obs.cov[1][1])) {
      eigenvector[0][0] = 1.0;
      eigenvector[0][1] = 0.0;
      eigenvector[1][0] = -0.0;
      eigenvector[1][1] = 1.0;
    }
    else {
      eigenvector[0][0] = 0.0;
      eigenvector[0][1] = 1.0;
      eigenvector[1][0] = -1.0;
      eigenvector[1][1] = 0.0;
    }
  }

  // all observations will be displayed in green goal_color
  XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);

  // draw cross hair
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, (obs.pos[X] - (FRAME_L / 2.0)*eigenvector[0][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (obs.pos[Y] - (FRAME_L / 2.0)*eigenvector[0][Y]), kEnvironment),
    ConvertWorld2PixmapX(zoom, (obs.pos[X] + (FRAME_L / 2.0)*eigenvector[0][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (obs.pos[Y] + (FRAME_L / 2.0)*eigenvector[0][Y]), kEnvironment));
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, (obs.pos[X] - (FRAME_L / 2.0)*eigenvector[1][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (obs.pos[Y] - (FRAME_L / 2.0)*eigenvector[1][Y]), kEnvironment),
    ConvertWorld2PixmapX(zoom, (obs.pos[X] + (FRAME_L / 2.0)*eigenvector[1][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (obs.pos[Y] + (FRAME_L / 2.0)*eigenvector[1][Y]), kEnvironment));
  dx0 = eigenvalue[0] * eigenvector[0][X];
  dy0 = eigenvalue[0] * eigenvector[0][Y];

  //printf("observation cov:\n");
  //printf("\t%lf %lf\n\t%lf %lf\n", obs.cov[0][0], obs.cov[0][1], obs.cov[1][0], obs.cov[1][1]);

  for (theta = 0.0; theta <= 2 * M_PI; theta += M_PI / 20.0) {
    dx1 = (eigenvalue[0] * cos(theta))*eigenvector[0][X] +
      (eigenvalue[1] * sin(theta))*eigenvector[1][X];
    dy1 = (eigenvalue[0] * cos(theta))*eigenvector[0][Y] +
      (eigenvalue[1] * sin(theta))*eigenvector[1][Y];
    XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, (obs.pos[X] + dx0), kEnvironment),
      ConvertWorld2PixmapY(zoom, (obs.pos[Y] + dy0), kEnvironment),
      ConvertWorld2PixmapX(zoom, (obs.pos[X] + dx1), kEnvironment),
      ConvertWorld2PixmapY(zoom, (obs.pos[Y] + dy1), kEnvironment));
    dx0 = dx1;
    dy0 = dy1;
  }
}

// estimates in world coordinates
void draw_estimate(scale, est)
double scale;
Estimate est;
{
  double a, b, c, root[2];
  double dx, dy, mag, eigenvalue[2], eigenvector[2][2];
  double ref_w[2], ref_b[2], wTb[4][4], bTw[4][4];
  double theta, dx0, dy0, dx1, dy1;

  printf("inside draw_estimate()\n");
  printf("x=%6.4lf y=%6.4lf xdot=%6.4lf ydot=%6.4lf\n\n", est.state[X], est.state[Y], est.state[XDOT], est.state[YDOT]);

  //printf("   %lf %lf\n", est.cov[0][0], est.cov[0][1]);
  //printf("   %lf %lf\n", est.cov[1][0], est.cov[1][1]);

  // cov = [A  B] => det |JJt| = a(lambda)^2 + b(lambda) + c
  //       [B  C]
  a = 1.0;
  b = -(est.cov[0][0] + est.cov[1][1]);
  c = est.cov[0][0] * est.cov[1][1] - est.cov[1][0] * est.cov[0][1];

  root[0] = (-b + sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);
  root[1] = (-b - sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);

  if (fabs(est.cov[1][0]) > 0.000001) {
    dy = 1.0;
    dx = -(est.cov[1][1] - root[0]) / est.cov[1][0];

    mag = sqrt(SQR(dx) + SQR(dy));
    //      eigenvalue[0] = scale*0.0221*sqrt(root[0]);
    eigenvalue[0] = scale*sqrt(root[0]);
    eigenvector[0][0] = dx / mag;
    eigenvector[0][1] = dy / mag;

    // the eigenvector for root 1
    //      eigenvalue[1] = scale*0.0221*sqrt(root[1]);
    eigenvalue[1] = scale*sqrt(root[1]);
    eigenvector[1][0] = -eigenvector[0][1];
    eigenvector[1][1] = eigenvector[0][0];
  }
  else {
    // set eigenvalue[0] to be the greater root
    if (fabs(root[0]) > fabs(root[1])) {
      eigenvalue[0] = scale*sqrt(root[0]);
      eigenvalue[1] = scale*sqrt(root[1]);
    }
    else {
      eigenvalue[0] = scale*sqrt(root[1]);
      eigenvalue[1] = scale*sqrt(root[0]);
    }

    // when cov item A > cov item C, Roger is facing either of the green
    // walls; when cov item A < cov item C, Roger is facing either of the
    // blue walls
    if (fabs(est.cov[0][0]) > fabs(est.cov[1][1])) {
      eigenvector[0][0] = 1.0;
      eigenvector[0][1] = 0.0;
      eigenvector[1][0] = -0.0;
      eigenvector[1][1] = 1.0;
    }
    else {
      eigenvector[0][0] = 0.0;
      eigenvector[0][1] = 1.0;
      eigenvector[1][0] = -1.0;
      eigenvector[1][1] = 0.0;
    }
  }

  // all "estimates" will be displayed in blue
  XSetForeground(display, gc, world_colors[BLUE].display_color);


  // draw cross hair
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, (est.state[X] - (FRAME_L / 2.0)*eigenvector[0][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (est.state[Y] - (FRAME_L / 2.0)*eigenvector[0][Y]), kEnvironment),
    ConvertWorld2PixmapX(zoom, (est.state[X] + (FRAME_L / 2.0)*eigenvector[0][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (est.state[Y] + (FRAME_L / 2.0)*eigenvector[0][Y]), kEnvironment));
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, (est.state[X] - (FRAME_L / 2.0)*eigenvector[1][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (est.state[Y] - (FRAME_L / 2.0)*eigenvector[1][Y]), kEnvironment),
    ConvertWorld2PixmapX(zoom, (est.state[X] + (FRAME_L / 2.0)*eigenvector[1][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (est.state[Y] + (FRAME_L / 2.0)*eigenvector[1][Y]), kEnvironment));
  dx0 = eigenvalue[0] * eigenvector[0][X];
  dy0 = eigenvalue[0] * eigenvector[0][Y];

  //printf("estimate cov:\n");
  //printf("\t%lf %lf\n\t%lf %lf\n", est.cov[0][0], est.cov[0][1], est.cov[1][0], est.cov[1][1]);

  for (theta = 0.0; theta <= 2 * M_PI; theta += M_PI / 20.0) {
    dx1 = (eigenvalue[0] * cos(theta))*eigenvector[0][X] +
      (eigenvalue[1] * sin(theta))*eigenvector[1][X];
    dy1 = (eigenvalue[0] * cos(theta))*eigenvector[0][Y] +
      (eigenvalue[1] * sin(theta))*eigenvector[1][Y];
    XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, (est.state[X] + dx0), kEnvironment),
      ConvertWorld2PixmapY(zoom, (est.state[Y] + dy0), kEnvironment),
      ConvertWorld2PixmapX(zoom, (est.state[X] + dx1), kEnvironment),
      ConvertWorld2PixmapY(zoom, (est.state[Y] + dy1), kEnvironment));
    dx0 = dx1;
    dy0 = dy1;
  }
}

void draw_ellipse(est)
Estimate est;
{
  double m[2][2], a, b, c, root[2];
  double dx, dy, mag, eigenvalue[2], eigenvector[2][2];
  double theta, dx0, dy0, dx1, dy1;

  //  printf("observation time = %lf  time = %lf\n", est.t, simtime);

  //if ((est.t == simtime)) {

  // DRAW THE CURRENT ESTIMATED STATE VARIABLES AND THEIR VELOCITIES
  draw_circle(ConvertWorld2PixmapX(zoom, est.state[X], kEnvironment), ConvertWorld2PixmapY(zoom, est.state[Y], kEnvironment),
    ConvertWorld2PixmapR(zoom, R_JOINT, kEnvironment), FILL);

  m[0][0] = est.cov[0][0];
  m[0][1] = est.cov[0][1];
  m[1][0] = est.cov[1][0];
  m[1][1] = est.cov[1][1];

  // cov = [A  B] => det |JJt| = a(lambda)^2 + b(lambda) +c
  //       [B  C]
  a = 1.0;
  b = -(m[0][0] + m[1][1]);
  c = m[0][0] * m[1][1] - m[1][0] * m[0][1];

  root[0] = (-b + sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);
  root[1] = (-b - sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);

  // the eigenvector for root 0
  dy = 1.0;
  dx = -m[0][1] / (m[0][0] - root[0]);
  mag = sqrt(SQR(dx) + SQR(dy));
  eigenvalue[0] = sqrt(root[0]);
  eigenvector[0][0] = dx / mag;
  eigenvector[0][1] = dy / mag;

  // the eigenvector for root 1
  dy = 1.0;
  dx = -m[0][1] / (m[0][0] - root[1]);
  mag = sqrt(SQR(dx) + SQR(dy));
  eigenvalue[1] = sqrt(root[1]);
  eigenvector[1][0] = dx / mag;
  eigenvector[1][1] = dy / mag;

  dx0 = eigenvalue[0] * eigenvector[0][X];
  dy0 = eigenvalue[0] * eigenvector[0][Y];
  for (theta = M_PI / 20.0; theta < 2 * M_PI; theta += M_PI / 20.0) {
    dx1 = (eigenvalue[0] * cos(theta))*eigenvector[0][X] +
      (eigenvalue[1] * sin(theta))*eigenvector[1][X];
    dy1 = (eigenvalue[0] * cos(theta))*eigenvector[0][Y] +
      (eigenvalue[1] * sin(theta))*eigenvector[1][Y];
    XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, (est.state[X] + dx0), kEnvironment),
      ConvertWorld2PixmapY(zoom, (est.state[Y] + dy0), kEnvironment),
      ConvertWorld2PixmapX(zoom, (est.state[X] + dx1), kEnvironment),
      ConvertWorld2PixmapY(zoom, (est.state[Y] + dy1), kEnvironment));
    dx0 = dx1;
    dy0 = dy1;
  }
  // }
}

/* the streamline visualization is not yet implemented in this version */
void draw_streamlines(roger, color)
Robot* roger;
int color;
{
  // void sor();
  double from[2], to[2];
  int i, row, col, current_row, current_col, already_used[NBINS][NBINS], count;
  double grad[2], oldx, oldy, x, y, mag, sim_compute_gradient();
  void mark_used(), draw_current_streamline();

  /* make sure the harmonic function is fully converged */
  // sor(roger);

  // initialize auxilliary structure for controlling the
  // density of streamlines rendered
  for (row=0; row<NBINS; ++row) {
    for (col=0; col<NBINS; ++col) {
      already_used[row][col] = FALSE;
    }
  }

  XSetForeground(display, gc, world_colors[LIGHTBLUE].display_color);

  float x_delta, y_delta, r_obstacle;
  float max_x, max_y, min_x, min_y;
  if (kEnvironment == DEVELOPMENT) {
    x_delta = XDELTA_DEV;
    y_delta = YDELTA_DEV;
    r_obstacle = R_OBSTACLE_DEV;
    max_x = MAX_X_DEV;
    min_x = MIN_X_DEV;
    max_y = MAX_Y_DEV;
    min_y = MIN_Y_DEV;
  }
  else {
    x_delta = XDELTA;
    y_delta = YDELTA;
    r_obstacle = R_OBSTACLE;
    max_x = MAX_X;
    min_x = MIN_X;
    max_y = MAX_Y;
    min_y = MIN_Y;
  }
  // if [col,row] is FREESPACE and at least one of its neighbors
  // is OBSTACLE (could be DIRICHLET and NEUMANN), then draw a streamline
  for (row=1;row<(NBINS-1); ++row) {
    for (col=1;col<(NBINS-1); ++col) {
      if ((roger->world_map.occupancy_map[row][col] == FREESPACE) &&
    ((roger->world_map.occupancy_map[row-1][col-1] == OBSTACLE) ||
     (roger->world_map.occupancy_map[row-1][col] == OBSTACLE)   ||
     (roger->world_map.occupancy_map[row-1][col+1] == OBSTACLE) ||
     (roger->world_map.occupancy_map[row][col-1] == OBSTACLE)   ||
     (roger->world_map.occupancy_map[row][col+1] == OBSTACLE)   ||
     (roger->world_map.occupancy_map[row+1][col-1] == OBSTACLE) ||
     (roger->world_map.occupancy_map[row+1][col] == OBSTACLE)   ||
     (roger->world_map.occupancy_map[row+1][col+1] == OBSTACLE) ||
     (roger->world_map.occupancy_map[row-1][col-1] == DILATED_OBSTACLE) ||
     (roger->world_map.occupancy_map[row-1][col] == DILATED_OBSTACLE)   ||
     (roger->world_map.occupancy_map[row-1][col+1] == DILATED_OBSTACLE) ||
     (roger->world_map.occupancy_map[row][col-1] == DILATED_OBSTACLE)   ||
     (roger->world_map.occupancy_map[row][col+1] == DILATED_OBSTACLE)   ||
     (roger->world_map.occupancy_map[row+1][col-1] == DILATED_OBSTACLE) ||
     (roger->world_map.occupancy_map[row+1][col] == DILATED_OBSTACLE)   ||
     (roger->world_map.occupancy_map[row+1][col+1] == DILATED_OBSTACLE) ) &&
    (!already_used[row][col]) ) {

  /* follow a stream line */

  oldx = min_x + (col+0.5)*x_delta;
  oldy = max_y - (row+0.5)*y_delta;

  count = 0;
  current_row = row;
  current_col = col;
  while ((roger->world_map.occupancy_map[current_row][current_col]!=GOAL)
         && (mag=sim_compute_gradient(oldx, oldy, roger, grad))
         && (count++ < 500)) {

    x = oldx - STEP*grad[X];
    y = oldy - STEP*grad[Y];
    XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom,oldx, kEnvironment),
        ConvertWorld2PixmapY(zoom,oldy, kEnvironment),
        ConvertWorld2PixmapX(zoom,x, kEnvironment),
        ConvertWorld2PixmapY(zoom,y, kEnvironment));
    /*  circle((int)x,(int)y,2,background);  */
    oldx=x; oldy=y;
    current_row = (int)(y - max_y)/y_delta;
    current_col = (int)(min_x - x)/x_delta;
  }

  //  mark_used((col+1),(row+1),already_used);
      mark_used(row, col, already_used);
      }
    }
  }
  //  draw_history();
  x_expose();
}

// corrected version 10-6-09
double sim_compute_gradient(x, y, roger, grad)
double x, y;
Robot * roger;
double grad[2]; // grad = [ d(phi)/dx  d(phi)/dy ]
{

  float x_delta, y_delta, r_obstacle;
  float max_x, max_y, min_x, min_y;
  if (kEnvironment == DEVELOPMENT) {
    x_delta = XDELTA_DEV;
    y_delta = YDELTA_DEV;
    r_obstacle = R_OBSTACLE_DEV;
    max_x = MAX_X_DEV;
    min_x = MIN_X_DEV;
    max_y = MAX_Y_DEV;
    min_y = MIN_Y_DEV;
  }
  else {
    x_delta = XDELTA;
    y_delta = YDELTA;
    r_obstacle = R_OBSTACLE;
    max_x = MAX_X;
    min_x = MIN_X;
    max_y = MAX_Y;
    min_y = MIN_Y;
  }

  int i0,i1,j0,j1;
  double mag, dphi_di, dphi_dj, del_x, del_y;

  j0 = (int) ((x-min_x)/x_delta);
  j1 = (j0+1);
  i1 = NBINS - (int) ((y - min_y)/y_delta); // (int) ((MAX_Y - y)/YDELTA);?
  i0 = (i1-1);

  del_x = (x-min_x)/x_delta - j0;
  del_y = (NBINS - (y - min_y)/y_delta) - i0;

  dphi_dj = ((1.0-del_y)*(roger->world_map.potential_map[i0][j1]
			  - roger->world_map.potential_map[i0][j0] ) +
	     (del_y)*(roger->world_map.potential_map[i1][j1]
		      - roger->world_map.potential_map[i1][j0]  ) );
  dphi_di = ((1.0-del_x)*(roger->world_map.potential_map[i1][j0]
			  - roger->world_map.potential_map[i0][j0] ) +
	     (del_x)*(roger->world_map.potential_map[i1][j1]
		      - roger->world_map.potential_map[i0][j1]  ) );

  grad[0] = dphi_dj; grad[1] = -dphi_di;

  mag = sqrt(SQR(grad[0])+SQR(grad[1]));

  if (mag>THRESHOLD) {
    grad[0] /= mag; grad[1] /= mag;
  }
  else grad[0] = grad[1] = 0;
  return(mag);
}


typedef struct _visible_object {
  double dx, dy;
  double radius;
  int color;
} VisibleObject;

void insertion_sort(vob, sort, num)
VisibleObject *vob;
int *sort, num;
{
  int i, j, temp;

  for (i = 0; i<num; i++) sort[i] = i;

  for (i = 1; i<num; i++) {
    j = i - 1;
    temp = sort[i];
    while ((j >= 0) && (sqrt(SQR(vob[sort[j]].dx) + SQR(vob[sort[j]].dy)) <=
      sqrt(SQR(vob[temp].dx) + SQR(vob[temp].dy)))) {
      sort[j + 1] = sort[j];
      j--;
    }
    sort[j + 1] = temp;
  }
}


void pinhole_camera(vob, i, rogerID)
VisibleObject vob;
int i;
int rogerID;
{
  int j, low_bin_index, high_bin_index, paint_image;
  double phi, alpha;

  phi = atan2(vob.dy, vob.dx) - RogersBodys[rogerID].eyes[i].theta; // eye frame heading eye->object


    //  printf("      phi for eye %d = %6.4lf\n", i, phi);
  alpha = atan2(vob.radius, sqrt(SQR(vob.dx)+SQR(vob.dy) - SQR(vob.radius)));

  paint_image = FALSE;

  if (fabs(phi + alpha) < FOV) { /* CCW edge of the feature projects in FOV */
    high_bin_index = (int) (NPIXELS / 2.0 * (1.0 + tan(phi + alpha)));
    paint_image = TRUE;
  }
  else high_bin_index = 127;

  if (fabs(phi - alpha) < FOV) { /* CW edge of the feature projects in FOV */
    low_bin_index = (int) (NPIXELS / 2.0 * (1.0 + tan(phi - alpha)));
    paint_image = TRUE;
  }
  else low_bin_index = 0;

  // low_bin_index = MAX(low_bin_index,0);
  // high_bin_index = MIN(high_bin_index,(NPIXELS-1));

  if (paint_image) {
    for (j = low_bin_index; j <= high_bin_index; j++) {
      RogersBodys[rogerID].eyes[i].image[j] = vob.color;
    }
  }
}


void make_images(rogerID)
int rogerID;
{
  int i, j, eye, o_index, k; // intensity[3];
  double x, y;
  double p_b[4], p_w[4], bTw[4][4];


  // Define delta_x, delta_y, and r_obstacle size given the environment
  float x_delta, y_delta, r_obstacle;
  float max_x, max_y, min_x, min_y;
  if (kEnvironment == DEVELOPMENT) {
    x_delta = XDELTA_DEV;
    y_delta = YDELTA_DEV;
    r_obstacle = R_OBSTACLE_DEV;
    max_x = MAX_X_DEV;
    min_x = MIN_X_DEV;
    max_y = MAX_Y_DEV;
    min_y = MIN_Y_DEV;
  }
  else {
    x_delta = XDELTA;
    y_delta = YDELTA;
    r_obstacle = R_OBSTACLE;
    max_x = MAX_X;
    min_x = MIN_X;
    max_y = MAX_Y;
    min_y = MIN_Y;
  }


  int sort[NBINS*NBINS + (MaxRobotNum * N_VISIBLEOBJ_ROBOT) + MaxToyNum];
  VisibleObject vobject[NBINS*NBINS + (MaxRobotNum * N_VISIBLEOBJ_ROBOT) + MaxToyNum];


  /* initialize image white */
  // make sure eye's images keep changing when roger is moving
  for (i = 0; i < NPIXELS; i++) {
    RogersBodys[rogerID].eyes[LEFT].image[i] = RogersBodys[rogerID].eyes[RIGHT].image[i] = 100;
  }


  for (eye = LEFT; eye <= RIGHT; ++eye) {
    // Visual objects counter
    o_index = 0;

    // Go through visual body parts of all robots first (arm1, arm2)
    for (k = 0; k < numRoger; ++k) {
      //    first 3 elements in the range array are the hands and the ball
      /* LEFT_ARM - in body frame */
      sim_fwd_kinematics(LEFT, RogersBodys[k].arms[LEFT][1].theta, RogersBodys[k].arms[LEFT][2].theta, &x, &y);
      // convert to eye coordinates
      vobject[o_index].dx = x - RogersBodys[rogerID].eyes[eye].position[X];
      vobject[o_index].dy = y - RogersBodys[rogerID].eyes[eye].position[Y];
      vobject[o_index].radius = R_JOINT;
      vobject[o_index++].color = ARM_COLOR;

      /* RIGHT_ARM - in body frame */
      sim_fwd_kinematics(RIGHT, RogersBodys[k].arms[RIGHT][1].theta, RogersBodys[k].arms[RIGHT][2].theta, &x, &y);
      vobject[o_index].dx = x - RogersBodys[rogerID].eyes[eye].position[X];
      vobject[o_index].dy = y - RogersBodys[rogerID].eyes[eye].position[Y];
      vobject[o_index].radius = R_JOINT;
      vobject[o_index++].color = ARM_COLOR;
    }


    /* OBJECT */

    // TODO iterate over all active toys, adding them to the vobjects array

    SIMinv_transform(RogersBodys[rogerID].mobile_base.wTb, bTw);

    for (int q = 0; q < n_active_toys; ++q) {
      p_w[0] = active_toys[q].position[X]; p_w[1] = active_toys[q].position[Y];
      p_w[2] = 0.0; p_w[3] = 1.0;
      SIMmatXvec(bTw, p_w, p_b);
      vobject[o_index].dx = p_b[X] - RogersBodys[rogerID].eyes[eye].position[X];
      vobject[o_index].dy = p_b[Y] - RogersBodys[rogerID].eyes[eye].position[Y];
      
      // TODO will we have to increase this?
      vobject[o_index].radius = active_toys[q].image_radius;
      vobject[o_index++].color = active_toys[q].color;
    }

    


    // after the first three, the rest are colored obstacles in the occupancy
    // grid
    // points at the next empty element of the range array
    for (i = 0; i<NBINS; ++i) {
      y = max_y - (i + 0.5)*y_delta;
      for (j = 0; j<NBINS; ++j) {
        if (Rogers[0].world_map.occupancy_map[i][j] == OBSTACLE) {
          p_w[0] = min_x + (j + 0.5)*x_delta; p_w[1] = y;
          p_w[2] = 0.0; p_w[3] = 1.0;
          SIMmatXvec(bTw, p_w, p_b);
          vobject[o_index].dx = p_b[X] - RogersBodys[rogerID].eyes[eye].position[X];
          vobject[o_index].dy = p_b[Y] - RogersBodys[rogerID].eyes[eye].position[Y];
          vobject[o_index].radius = r_obstacle;
          vobject[o_index++].color = Rogers[0].world_map.color_map[i][j];
        }
      }
    }

    // printf("o_index: %d\n", o_index);
    insertion_sort(vobject, sort, o_index);
    for (i = 0; i<o_index; ++i) {
      pinhole_camera(vobject[sort[i]], eye, rogerID);
    }
  }
}


void initialize_simulator(rst)
int rst;
{
  void initialize_inertial_objects();
  void initialize_roger(); void intialize_history();
  int i;

  if (rst) {
    initialize_roger();

    for (i = 0; i < numRoger; ++i) {
      Rogers[i].environment = kEnvironment;
      Rogers[i].graphics.zoom = zoom;
    }

    // this function simply initlizes the toybox, which is where we define the default parameters
    //     for all of the toy types
    initialize_toybox();

    // this procedure initilizes the polyballs
    initialize_inertial_objects();

    // all params and GUI position for the 5 objects in the toybox
    // initialize_toybox();

    /************************************************************************/
    // initialize the volatile elements (afferents and efferents)
    // of the applications interface for user control code
    write_interface(rst);
  }
}

void intialize_history() {
  int i;
  for (i = 0; i < MaxRobotNum; ++i) {
    history_ptr[i] = 0;
  }
}

void initialize_roger()
{
  int i, j, k, l;

  // Development environment *************************
  if (kEnvironment == DEVELOPMENT) {
  /************************************************************************/
    // MOBILE BASE
    for (i = 0; i<4; ++i) {
      for (j = 0; j<4; ++j) {
        RogersBodys[0].mobile_base.wTb[i][j] = mobile_base_home.wTb[i][j];
      }
    }

    RogersBodys[0].mobile_base.x = mobile_base_home.x;
    RogersBodys[0].mobile_base.x_dot = mobile_base_home.x_dot;
    RogersBodys[0].mobile_base.y = mobile_base_home.y;
    RogersBodys[0].mobile_base.y_dot = mobile_base_home.y_dot;
    RogersBodys[0].mobile_base.theta = mobile_base_home.theta;
    RogersBodys[0].mobile_base.theta_dot = mobile_base_home.theta_dot;
    for (i = 0; i<2; ++i) {
      RogersBodys[0].mobile_base.wheel_torque[i] = mobile_base_home.wheel_torque[i];
    }
    RogersBodys[0].mobile_base.contact_theta = mobile_base_home.contact_theta;
    for (i = 0; i<2; ++i) {
      RogersBodys[0].mobile_base.extForce[i] = mobile_base_home.extForce[i];
    }

    /************************************************************************/
    // LEFT AND RIGHT EYE
    for (i = LEFT; i <= RIGHT; ++i) {
      RogersBodys[0].eyes[i].position[X] = eyes_home[i].position[X];
      RogersBodys[0].eyes[i].position[Y] = eyes_home[i].position[Y];
      RogersBodys[0].eyes[i].theta = eyes_home[i].theta;
      RogersBodys[0].eyes[i].theta_dot = eyes_home[i].theta_dot;
      for (j = 0; j<NPIXELS; ++j) {
        RogersBodys[0].eyes[i].image[j] = eyes_home[i].image[j];
      }
      RogersBodys[0].eyes[i].torque = eyes_home[i].torque;
    }

    /************************************************************************/
    // LEFT AND RIGHT ARM
    for (i = 0; i<NARMS; ++i) {
      for (j = 0; j<NARM_FRAMES; ++j) {
        for (k = 0; k<4; ++k) {
          for (l = 0; l<4; ++l) {
            RogersBodys[0].arms[i][j].iTj[k][l] = arms_home[i][j].iTj[k][l];
          }
        }
        RogersBodys[0].arms[i][j].dof_type = arms_home[i][j].dof_type;
        RogersBodys[0].arms[i][j].axis = arms_home[i][j].axis;
        RogersBodys[0].arms[i][j].theta = arms_home[i][j].theta;
        RogersBodys[0].arms[i][j].theta_dot = arms_home[i][j].theta_dot;
        RogersBodys[0].arms[i][j].torque = arms_home[i][j].torque;
        RogersBodys[0].arms[i][j].extForce[0] = arms_home[i][j].extForce[0];
        RogersBodys[0].arms[i][j].extForce[1] = arms_home[i][j].extForce[1];
      }
    }

  // Arena environment ****************************************************
  } else {

    // *******************************************************************
    // Roger#0 ***********************************************************
    // *******************************************************************
      /************************************************************************/
    // MOBILE BASE
    for (i = 0; i<4; ++i) {
      for (j = 0; j<4; ++j) {
        RogersBodys[0].mobile_base.wTb[i][j] = mobile_base_home_1.wTb[i][j];
      }
    }

    RogersBodys[0].mobile_base.x = mobile_base_home_1.x;
    RogersBodys[0].mobile_base.x_dot = mobile_base_home_1.x_dot;
    RogersBodys[0].mobile_base.y = mobile_base_home_1.y;
    RogersBodys[0].mobile_base.y_dot = mobile_base_home_1.y_dot;
    RogersBodys[0].mobile_base.theta = mobile_base_home_1.theta;
    RogersBodys[0].mobile_base.theta_dot = mobile_base_home_1.theta_dot;
    for (i = 0; i<2; ++i) {
      RogersBodys[0].mobile_base.wheel_torque[i] = mobile_base_home_1.wheel_torque[i];
    }
    RogersBodys[0].mobile_base.contact_theta = mobile_base_home_1.contact_theta;
    for (i = 0; i<2; ++i) {
      RogersBodys[0].mobile_base.extForce[i] = mobile_base_home_1.extForce[i];
    }

    /************************************************************************/
    // LEFT AND RIGHT EYE
    for (i = LEFT; i <= RIGHT; ++i) {
      RogersBodys[0].eyes[i].position[X] = eyes_home[i].position[X];
      RogersBodys[0].eyes[i].position[Y] = eyes_home[i].position[Y];
      RogersBodys[0].eyes[i].theta = eyes_home[i].theta;
      RogersBodys[0].eyes[i].theta_dot = eyes_home[i].theta_dot;
      for (j = 0; j<NPIXELS; ++j) {
        RogersBodys[0].eyes[i].image[j] = eyes_home[i].image[j];
      }
      RogersBodys[0].eyes[i].torque = eyes_home[i].torque;
    }

    /************************************************************************/
    // LEFT AND RIGHT ARM
    for (i = 0; i<NARMS; ++i) {
      for (j = 0; j<NARM_FRAMES; ++j) {
        for (k = 0; k<4; ++k) {
          for (l = 0; l<4; ++l) {
            RogersBodys[0].arms[i][j].iTj[k][l] = arms_home_1[i][j].iTj[k][l];
          }
        }
        RogersBodys[0].arms[i][j].dof_type = arms_home_1[i][j].dof_type;
        RogersBodys[0].arms[i][j].axis = arms_home_1[i][j].axis;
        RogersBodys[0].arms[i][j].theta = arms_home_1[i][j].theta;
        RogersBodys[0].arms[i][j].theta_dot = arms_home_1[i][j].theta_dot;
        RogersBodys[0].arms[i][j].torque = arms_home_1[i][j].torque;
        RogersBodys[0].arms[i][j].extForce[0] = arms_home_1[i][j].extForce[0];
        RogersBodys[0].arms[i][j].extForce[1] = arms_home_1[i][j].extForce[1];
      }
    }

    // *******************************************************************
    // Roger#1 ***********************************************************
    // *******************************************************************
      /************************************************************************/
    // MOBILE BASE
    for (i = 0; i<4; ++i) {
      for (j = 0; j<4; ++j) {
        RogersBodys[1].mobile_base.wTb[i][j] = mobile_base_home_2.wTb[i][j];
      }
    }

    RogersBodys[1].mobile_base.x = mobile_base_home_2.x;
    RogersBodys[1].mobile_base.x_dot = mobile_base_home_2.x_dot;
    RogersBodys[1].mobile_base.y = mobile_base_home_2.y;
    RogersBodys[1].mobile_base.y_dot = mobile_base_home_2.y_dot;
    RogersBodys[1].mobile_base.theta = mobile_base_home_2.theta;
    RogersBodys[1].mobile_base.theta_dot = mobile_base_home_2.theta_dot;
    for (i = 0; i<2; ++i) {
      RogersBodys[1].mobile_base.wheel_torque[i] = mobile_base_home_2.wheel_torque[i];
    }
    RogersBodys[1].mobile_base.contact_theta = mobile_base_home_2.contact_theta;
    for (i = 0; i<2; ++i) {
      RogersBodys[1].mobile_base.extForce[i] = mobile_base_home_2.extForce[i];
    }

    /************************************************************************/
    // LEFT AND RIGHT EYE
    for (i = LEFT; i <= RIGHT; ++i) {
      RogersBodys[1].eyes[i].position[X] = eyes_home[i].position[X];
      RogersBodys[1].eyes[i].position[Y] = eyes_home[i].position[Y];
      RogersBodys[1].eyes[i].theta = eyes_home[i].theta;
      RogersBodys[1].eyes[i].theta_dot = eyes_home[i].theta_dot;
      for (j = 0; j<NPIXELS; ++j) {
        RogersBodys[1].eyes[i].image[j] = eyes_home[i].image[j];
      }
      RogersBodys[1].eyes[i].torque = eyes_home[i].torque;
    }

    /************************************************************************/
    // LEFT AND RIGHT ARM
    for (i = 0; i<NARMS; ++i) {
      for (j = 0; j<NARM_FRAMES; ++j) {
        for (k = 0; k<4; ++k) {
          for (l = 0; l<4; ++l) {
            RogersBodys[1].arms[i][j].iTj[k][l] = arms_home_2[i][j].iTj[k][l];
          }
        }
        RogersBodys[1].arms[i][j].dof_type = arms_home_2[i][j].dof_type;
        RogersBodys[1].arms[i][j].axis = arms_home_2[i][j].axis;
        RogersBodys[1].arms[i][j].theta = arms_home_2[i][j].theta;
        RogersBodys[1].arms[i][j].theta_dot = arms_home_2[i][j].theta_dot;
        RogersBodys[1].arms[i][j].torque = arms_home_2[i][j].torque;
        RogersBodys[1].arms[i][j].extForce[0] = arms_home_2[i][j].extForce[0];
        RogersBodys[1].arms[i][j].extForce[1] = arms_home_2[i][j].extForce[1];
      }
    }


    // *******************************************************************
    // Roger#2 ***********************************************************
    // *******************************************************************
      /************************************************************************/
    // MOBILE BASE
    for (i = 0; i<4; ++i) {
      for (j = 0; j<4; ++j) {
        RogersBodys[2].mobile_base.wTb[i][j] = mobile_base_home_3.wTb[i][j];
      }
    }

    RogersBodys[2].mobile_base.x = mobile_base_home_3.x;
    RogersBodys[2].mobile_base.x_dot = mobile_base_home_3.x_dot;
    RogersBodys[2].mobile_base.y = mobile_base_home_3.y;
    RogersBodys[2].mobile_base.y_dot = mobile_base_home_3.y_dot;
    RogersBodys[2].mobile_base.theta = mobile_base_home_3.theta;
    RogersBodys[2].mobile_base.theta_dot = mobile_base_home_3.theta_dot;
    for (i = 0; i<2; ++i) {
      RogersBodys[2].mobile_base.wheel_torque[i] = mobile_base_home_3.wheel_torque[i];
    }
    RogersBodys[2].mobile_base.contact_theta = mobile_base_home_3.contact_theta;
    for (i = 0; i<2; ++i) {
      RogersBodys[2].mobile_base.extForce[i] = mobile_base_home_3.extForce[i];
    }

    /************************************************************************/
    // LEFT AND RIGHT EYE
    for (i = LEFT; i <= RIGHT; ++i) {
      RogersBodys[2].eyes[i].position[X] = eyes_home[i].position[X];
      RogersBodys[2].eyes[i].position[Y] = eyes_home[i].position[Y];
      RogersBodys[2].eyes[i].theta = eyes_home[i].theta;
      RogersBodys[2].eyes[i].theta_dot = eyes_home[i].theta_dot;
      for (j = 0; j<NPIXELS; ++j) {
        RogersBodys[2].eyes[i].image[j] = eyes_home[i].image[j];
      }
      RogersBodys[2].eyes[i].torque = eyes_home[i].torque;
    }

    /************************************************************************/
    // LEFT AND RIGHT ARM
    for (i = 0; i<NARMS; ++i) {
      for (j = 0; j<NARM_FRAMES; ++j) {
        for (k = 0; k<4; ++k) {
          for (l = 0; l<4; ++l) {
            RogersBodys[2].arms[i][j].iTj[k][l] = arms_home_3[i][j].iTj[k][l];
          }
        }
        RogersBodys[2].arms[i][j].dof_type = arms_home_3[i][j].dof_type;
        RogersBodys[2].arms[i][j].axis = arms_home_3[i][j].axis;
        RogersBodys[2].arms[i][j].theta = arms_home_3[i][j].theta;
        RogersBodys[2].arms[i][j].theta_dot = arms_home_3[i][j].theta_dot;
        RogersBodys[2].arms[i][j].torque = arms_home_3[i][j].torque;
        RogersBodys[2].arms[i][j].extForce[0] = arms_home_3[i][j].extForce[0];
        RogersBodys[2].arms[i][j].extForce[1] = arms_home_3[i][j].extForce[1];
      }
    }

  }
}

// define geometrical and inertial parameters for the NBODY dynamical system
void initialize_inertial_objects()
{
  double pb[4], pw[4]; // homogeneous position vectors in base and world coords
  double vb[4], vw[4]; // homogeneous velocity vectors in base and world coords
  double x, y, J[2][2];
  int i;

  void sim_fwd_kinematics(), sim_arm_Jacobian();

  // initialize an array "PolyBall objects[NBODY]" that support computing
  //    collision forces between "bodies" which are rigid body arrangements
  //    of elastic circular objects moving in the Cartesian world plane
  //  typedef struct _polyball {
  //    int id;                // CIRCLE || TRIANGLE
  //    int N;                 // a  composite of N elastic spheres
  //    double Rsphere;        // the radius of the elastic spheres
  //    double radius;         // distance from sphere center to body frame
  //    double mass;           // total mass of polyball
  //    double total_moment;   // moment of inertia (mass*SQR(radius) )
  //    double position[3];    // position (x,y,theta) of the object
  //    double velocity[3];    // velocity of the object
  //    double net_extForce[3]; // from collisions with other objects
  //  } PolyBall;

  int base_counter = 0;
  for (i = 0; i < numRoger; ++i) {
    base_counter = i * N_BODY_ROBOT;

    /****************************** 0: BASE ************************************/
    objects_total[base_counter + BASE].id = -3;
    objects_total[base_counter + BASE].N = 1;
    objects_total[base_counter + BASE].radii[0] = R_BASE;
    objects_total[base_counter + BASE].vertices[0][0] = 0.0;
    objects_total[base_counter + BASE].vertices[0][0] = 0.0;

    for (int q = 1; q < NUM_VERTICES; ++q) {
      objects_total[base_counter + ARM1].radii[q] = 0.0;
      objects_total[base_counter + ARM1].vertices[q][0] = 0.0;
      objects_total[base_counter + ARM1].vertices[q][1] = 0.0;
    }

    objects_total[base_counter + BASE].mass = M_BASE;
    objects_total[base_counter + BASE].moi = I_BASE;

    objects_total[base_counter + BASE].position[X] = RogersBodys[i].mobile_base.x;
    objects_total[base_counter + BASE].position[Y] = RogersBodys[i].mobile_base.y;
    objects_total[base_counter + BASE].position[THETA] = RogersBodys[i].mobile_base.theta;

    objects_total[base_counter + BASE].velocity[X] = RogersBodys[i].mobile_base.x_dot;
    objects_total[base_counter + BASE].velocity[Y] = RogersBodys[i].mobile_base.y_dot;
    objects_total[base_counter + BASE].velocity[THETA] = RogersBodys[i].mobile_base.theta_dot;

    objects_total[base_counter + BASE].net_extForce[X] = objects_total[base_counter + BASE].net_extForce[Y] =
      objects_total[base_counter + BASE].net_extForce[THETA] = 0.0;

    /************************** 1: LEFT ARM ************************************/
    // left hand position in world coordinates
    sim_fwd_kinematics(LEFT, RogersBodys[i].arms[LEFT][1].theta, RogersBodys[i].arms[LEFT][2].theta, &x, &y);
    pb[0] = x; pb[1] = y; pb[2] = 0.0; pb[3] = 1.0;
    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, pb, pw);

    // left hand velocity relative to base written in base coordinates
    //          (figure 2 in RogerDynamics document defines frames)
    // ^w(v)_10 = wRb[ ^b(v)_7 +  J_arm theta_dot_arm ]
    //    = ^w(v)_b + wRb[ -ARM_OFFSET omega_0 xhat_b + J_arm theta_dot_arm ]virtually;
    vb[X] = J[0][0] * RogersBodys[i].arms[LEFT][1].theta_dot +
      J[0][1] * RogersBodys[i].arms[LEFT][2].theta_dot - ARM_OFFSET*RogersBodys[i].mobile_base.theta_dot;
    vb[Y] = J[1][0] * RogersBodys[i].arms[LEFT][1].theta_dot + J[1][1] * RogersBodys[i].arms[LEFT][2].theta_dot;
    vb[2] = 0.0;
    vb[3] = 0.0; // homogeneous vector

    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, vb, vw);
    //  v[1][X] = base->x_dot + v_w[X];
    //  v[1][Y] = base->y_dot + v_w[Y];
    //  R[1] = R_TACTILE;

    objects_total[base_counter + ARM1].id = -2;
    objects_total[base_counter + ARM1].N = 1;
    objects_total[base_counter + ARM1].radii[0] = R_TACTILE;
    objects_total[base_counter + ARM1].vertices[0][0] = 0.0;
    objects_total[base_counter + ARM1].vertices[0][1] = 0.0;

    for (int q = 1; q < NUM_VERTICES; ++q) {
      objects_total[base_counter + ARM1].radii[q] = 0.0;
      objects_total[base_counter + ARM1].vertices[q][0] = 0.0;
      objects_total[base_counter + ARM1].vertices[q][1] = 0.0;
    }


    objects_total[base_counter + ARM1].mass = M_ARM1;
    objects_total[base_counter + ARM1].moi = I_ARM1;

    objects_total[base_counter + ARM1].position[X] = pw[X];
    objects_total[base_counter + ARM1].position[Y] = pw[Y];
    objects_total[base_counter + ARM1].position[THETA] = 0.0;
    // hand orientation is not relevant to system dynamics

    objects_total[base_counter + ARM1].velocity[X] = RogersBodys[i].mobile_base.x_dot + vw[X];
    objects_total[base_counter + ARM1].velocity[Y] = RogersBodys[i].mobile_base.y_dot + vw[Y];
    objects_total[base_counter + ARM1].velocity[THETA] = 0.0;
    // angular acceleration in the hand is not relevant to system dynamics

    objects_total[base_counter + ARM1].net_extForce[X] = objects_total[base_counter + ARM1].net_extForce[Y] =
      objects_total[base_counter + ARM1].net_extForce[THETA] = 0.0;

    /************************** 2: RIGHT ARM ************************************/
    // right hand position in world coordinates
    sim_fwd_kinematics(RIGHT, RogersBodys[i].arms[RIGHT][1].theta, RogersBodys[i].arms[RIGHT][2].theta, &x, &y);
    pb[0] = x; pb[1] = y; pb[2] = 0.0; pb[3] = 1.0;
    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, pb, pw);

    // left hand velocity relative to base written in base coordinates
    //          (figure 2 in RogerDynamics document defines frames)
    // ^w(v)_10 = wRb[ ^b(v)_7 +  J_arm theta_dot_arm ]
    //    = ^w(v)_b + wRb[ -ARM_OFFSET omega_0 xhat_b + J_arm theta_dot_arm ]
    sim_arm_Jacobian(RogersBodys[i].arms[RIGHT][1].theta, RogersBodys[i].arms[RIGHT][2].theta, J);
    vb[X] = J[0][0] * RogersBodys[i].arms[RIGHT][1].theta_dot +
      J[0][1] * RogersBodys[i].arms[RIGHT][2].theta_dot - ARM_OFFSET*RogersBodys[i].mobile_base.theta_dot;
    vb[Y] = J[1][0] * RogersBodys[i].arms[RIGHT][1].theta_dot + J[1][1] * RogersBodys[i].arms[RIGHT][2].theta_dot;
    vb[2] = 0.0;
    vb[3] = 0.0; // homogeneous vector

    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, vb, vw);

    objects_total[base_counter + ARM2].id = -1;
    objects_total[base_counter + ARM2].N = 1;
    objects_total[base_counter + ARM2].radii[0] = R_TACTILE;
    objects_total[base_counter + ARM2].vertices[0][0] = 0.0;
    objects_total[base_counter + ARM2].vertices[0][1] = 0.0;

    for (int q = 1; q < NUM_VERTICES; ++q) {
      objects_total[base_counter + ARM1].radii[q] = 0.0;
      objects_total[base_counter + ARM1].vertices[q][0] = 0.0;
      objects_total[base_counter + ARM1].vertices[q][1] = 0.0;
    }

    objects_total[base_counter + ARM2].mass = M_ARM1;
    objects_total[base_counter + ARM2].moi = I_ARM1;

    objects_total[base_counter + ARM2].position[X] = pw[X];
    objects_total[base_counter + ARM2].position[Y] = pw[Y];
    objects_total[base_counter + ARM2].position[THETA] = 0.0;
    // hand orientation not relevant to system dynamics

    objects_total[base_counter + ARM2].velocity[X] = RogersBodys[i].mobile_base.x_dot + vw[X];
    objects_total[base_counter + ARM2].velocity[Y] = RogersBodys[i].mobile_base.y_dot + vw[Y];
    objects_total[base_counter + ARM2].velocity[THETA] = 0.0;
    // angular acceleration of hand is not relevant to system dynamics

    objects_total[base_counter +ARM2].net_extForce[X] = objects_total[base_counter + ARM2].net_extForce[Y] =
    objects_total[base_counter + ARM2].net_extForce[THETA] = 0.0;

  }
  base_counter = base_counter + N_BODY_ROBOT;

  /****************** 3: TOY OBJECT - CIRCLE || TRIANGLE *********************/

  // intially make everything in the objects_total and active_toys arrays a circle with no velocity or force

  for (int q = 0; q < MaxToyNum; ++q) {
    int toy_counter = base_counter + q;
    
    objects_total[toy_counter].id = active_toys[q].id = toybox[CIRCLE].id;
    objects_total[toy_counter].N = active_toys[q].N = toybox[CIRCLE].N;

    for (int i = 0; i < NUM_VERTICES; ++i) {
      objects_total[toy_counter].radii[i] = active_toys[q].radii[i] = toybox[CIRCLE].radii[i];
      objects_total[toy_counter].vertices[i][0] = active_toys[q].vertices[i][0] = toybox[CIRCLE].vertices[i][0];
      objects_total[toy_counter].vertices[i][1] = active_toys[q].vertices[i][1] = toybox[CIRCLE].vertices[i][1];
      // printf("intialize toy: radii[%d]: %.4f, theta: %.4f, radial-offset: %.4f\n", i, toy_home_triangle.radii[i], toy_home_triangle.vertices[i][0], toy_home_triangle.vertices[i][1]);
    }

    objects_total[toy_counter].N_draw = toybox[CIRCLE].N_draw;

    for (int i = 0; i < NUM_DRAW_VERTS; ++i) {
      objects_total[toy_counter].draw_verts[i][0] = active_toys[q].draw_verts[i][0] = toybox[CIRCLE].draw_verts[i][0];
      objects_total[toy_counter].draw_verts[i][1] = active_toys[q].draw_verts[i][1] = toybox[CIRCLE].draw_verts[i][1];
    }

    objects_total[toy_counter].color = active_toys[q].color = toybox[CIRCLE].color;
    objects_total[toy_counter].image_radius = active_toys[q].image_radius = toybox[CIRCLE].image_radius;
    objects_total[toy_counter].mass = active_toys[q].mass = toybox[CIRCLE].mass;
    objects_total[toy_counter].moi = active_toys[q].moi = toybox[CIRCLE].moi;

    objects_total[toy_counter].position[X] = active_toys[q].position[X] = toybox[CIRCLE].position[X];
    objects_total[toy_counter].position[Y] = active_toys[q].position[Y] = toybox[CIRCLE].position[Y];
    objects_total[toy_counter].position[THETA] = active_toys[q].position[THETA] = toybox[CIRCLE].position[THETA];

    objects_total[toy_counter].velocity[X] = active_toys[q].velocity[X] = toybox[CIRCLE].velocity[X];
    objects_total[toy_counter].velocity[Y] = active_toys[q].velocity[Y] = toybox[CIRCLE].velocity[Y];
    objects_total[toy_counter].velocity[THETA] = active_toys[q].velocity[THETA] = toybox[CIRCLE].velocity[THETA];

    objects_total[toy_counter].net_extForce[X] = active_toys[q].net_extForce[X] = toybox[CIRCLE].net_extForce[X]; 
    objects_total[base_counter].net_extForce[Y] = active_toys[q].net_extForce[Y] = toybox[CIRCLE].net_extForce[X]; 
    objects_total[toy_counter].net_extForce[THETA] = active_toys[q].net_extForce[THETA] = toybox[CIRCLE].net_extForce[X];
  }
}

//check if input is in cartesian space area
int isCartesianInput(x_input, y_input, environment, x, y)
double x_input, y_input;
int environment;
double *x, *y;
{
  if (environment == ARENA) {
    if (x_input<(MIN_X + XDELTA) || x_input>(MAX_X - XDELTA) || y_input<(MIN_Y + YDELTA) || y_input>(MAX_Y- YDELTA) ) {
      //printf("Location out of bounds!!!\n");
      return FALSE;
    }
  } else {
    if (x_input<(MIN_X_DEV + XDELTA_DEV) || x_input>(MAX_X_DEV - XDELTA_DEV) || y_input<(MIN_Y_DEV + YDELTA_DEV) || y_input>(MAX_Y_DEV - YDELTA_DEV) ) {
      //printf("Location out of bounds!!!\n");
      return FALSE;
    }
  }

  *x = x_input;
  *y = y_input;

  return TRUE;
}

// Handles drawing requests that are made on the project side and
// sent to the simulator
void HandleDrawingRequests(roger)
Robot* roger;
{
  int i, j;

  if (draw_visual) {
    // Address draw_observation() requests
    for (i = 0; i < roger->drawing_request.obs_number; i++) {
      draw_observation(roger->drawing_request.obs_args[i].obs);
    }

    // Address draw_estimate() requests
    for (i = 0; i < roger->drawing_request.est_number; i++) {
      draw_estimate(roger->drawing_request.est_args[i].scale, roger->drawing_request.est_args[i].est);
    }

    // printf("past draw estimate\n");
    
    // // Address draw_estimate() requests
    for (i = 0; i < roger->drawing_request.line_number; i++) {


      printf("start_x: %.4f, stop_x: %.4f, start_y: %.4f, stop_y: %.4f\n",
        roger->drawing_request.line_args[i].line.start_x,
        roger->drawing_request.line_args[i].line.stop_x,
        roger->drawing_request.line_args[i].line.start_y,
        roger->drawing_request.line_args[i].line.stop_y);



      x_draw_line(roger->drawing_request.line_args[i].color,
        roger->drawing_request.line_args[i].line.start_x,
        roger->drawing_request.line_args[i].line.start_y,
        roger->drawing_request.line_args[i].line.stop_x,
        roger->drawing_request.line_args[i].line.stop_y
        );
    }

    for (i = 0; i < roger->drawing_request.text_number; i++) {
      // printf("MESS %p \t %s\n", roger->drawing_request.text_args[i].text, roger->drawing_request.text_args[i].text);
      char str[100];
      strcpy(str, roger->drawing_request.text_args[i].text);
      x_draw_string(roger->drawing_request.text_args[i].color,
        roger->drawing_request.text_args[i].x,
        roger->drawing_request.text_args[i].y,
        str);
    }

    printf("Drawing %d circles\n", roger->drawing_request.circle_number);
    for (i = 0; i < roger->drawing_request.circle_number; i++) {
      x_draw_circle(roger->drawing_request.circle_args[i].color,
        roger->drawing_request.circle_args[i].x,
        roger->drawing_request.circle_args[i].y,
        roger->drawing_request.circle_args[i].radius,
        roger->drawing_request.circle_args[i].fill);
    }

    // Address draw_history() requests
    if (roger->drawing_request.draw_history == TRUE) {
      // Only available in Development environment
      if(kEnvironment == DEVELOPMENT) {
        draw_history(0);
      }
    }

    // Address draw_steamline() requests
    if (roger->drawing_request.draw_streamline == TRUE) {
      // TODO: Modify the draw_streamline() in the project side
      // to send a list of line segment endpoints along with the
      // drawing_request data structure and draw them here
      draw_streamlines(roger,BLUE);
    }
  }


  // Reset requests
  roger->drawing_request.obs_number = 0;
  roger->drawing_request.est_number = 0;
  // roger->drawing_request.line_number = 0;
  // roger->drawing_request.text_number = 0;
  // roger->drawing_request.circle_number = 0;
  roger->drawing_request.draw_history = FALSE;
  roger->drawing_request.draw_streamline = FALSE;

}


// Handles map editing and ball placement interactions through the GUI
void HandleGuiInteraction(roger)
Robot * roger;
{
  int xbin, ybin;
  //cartesian coordinates
  double x, y;

  if (kEnvironment == DEVELOPMENT) {
      if (roger->button_event) {
      switch(roger->input_mode) {

        case BALL_INPUT:
          //check if inputs are valid
          if (isCartesianInput(roger->button_reference[X],
             roger->button_reference[Y], kEnvironment, &x, &y) == FALSE) {
            break;
          }
          
          // TODO updated to handle more object types
          //    instead of left or right click, try click while holding
          //    a specific key (1-5) for toys 0-4 for example
          else {
            if (roger->button_event == LEFT_BUTTON) {
              switch (roger->key_event) {
                case '1': place_object(x,y,  CIRCLE); break;
                case '2': place_object(x, y, TRIANGLE); break;
                case '3': place_object(x, y, SQUARE); break;
                case '4': place_object(x, y, ISO_TRIANGLE); break;
                case '5': place_object(x, y, PENTAGON); 
              }
            } else if (roger->button_event == RIGHT_BUTTON) {
              remove_object(x, y);
            }
            break;
          }

        case MAP_INPUT:
          if (isCartesianInput(roger->button_reference[X],
                 roger->button_reference[Y], kEnvironment, &x, &y) == FALSE)
          break;

          //      int xbin, ybin; already defined above
          printf("Map editor input - x: %4.3f, y: %4.3f - button: %d\n",
           x, y, roger->button_event);

          if (kEnvironment == ARENA) {
            xbin = (x - MIN_X) / XDELTA;
            ybin = NBINS - (y - MIN_Y) / YDELTA;
          } else {
            xbin = (x - MIN_X_DEV) / XDELTA_DEV;
            ybin = NBINS - (y - MIN_Y_DEV) / YDELTA_DEV;
          }


          if ((xbin<0) || (xbin>(NBINS-1)) || (ybin<0) || (ybin>(NBINS-1))) {
          printf("Out of the boundary!!!\n");
              }
          else {
            if (roger->button_event==LEFT_BUTTON) {// obstacles in Cartesian space
              if (roger->world_map.occupancy_map[ybin][xbin] == OBSTACLE) {
                printf("deleting an obstacle xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                roger->world_map.occupancy_map[ybin][xbin] = FREESPACE;
                //      delete_bin_bumper(xbin,ybin);
              }
              else if ((roger->world_map.occupancy_map[ybin][xbin] == FREESPACE) ||
                        (roger->world_map.occupancy_map[ybin][xbin] == DILATED_OBSTACLE)) {
                printf("inserting an obstacle xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                roger->world_map.occupancy_map[ybin][xbin] = OBSTACLE;
                roger->world_map.potential_map[ybin][xbin] = 1.0;
                roger->world_map.color_map[ybin][xbin] = LIGHTBLUE;
              }
            }
            else if (roger->button_event == MIDDLE_BUTTON) { }
            else if (roger->button_event == RIGHT_BUTTON) {
              if (roger->world_map.occupancy_map[ybin][xbin] == GOAL) {
                printf("deleting an goal xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                int row, col;
                for(row=xbin-1; row<=xbin+1; row++) {
                  for(col=ybin-1; col<=ybin+1; col++) {
                    roger->world_map.occupancy_map[col][row] = FREESPACE;
                  }
                }
              }
              else if (roger->world_map.occupancy_map[ybin][xbin] == FREESPACE) {
                printf("inserting an goal xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                int row, col;
                for(row=xbin-1; row<=xbin+1; row++) {
                  for(col=ybin-1; col<=ybin+1; col++) {
                    roger->world_map.occupancy_map[col][row] = GOAL;
                    roger->world_map.potential_map[col][row] = 0.0;
                  }
                }
                }
              }

          }

        break;
        default:
        break;
      }
    }

  } else {
    if (roger->button_event) {
      switch(roger->input_mode) {

        case BALL_INPUT_ARENA:
          //check if inputs are valid
          if (isCartesianInput(roger->button_reference[X],
             roger->button_reference[Y], kEnvironment, &x, &y) == FALSE) {
            break;
          } 
          
          // TODO updated to handle more object types
          //    instead of left or right click, try click while holding
          //    a specific key (1-5) for toys 0-4 for example
          else {
            if (roger->button_event == LEFT_BUTTON) {
              switch (roger->key_event) {
                case '1': place_object(x,y,CIRCLE); break;
                case '2': place_object(x, y, TRIANGLE);
              }
            } else if (roger->button_event == RIGHT_BUTTON) {
              remove_object(x,y);
            }
            break;
          }

        case MAP_INPUT_ARENA:
          if (isCartesianInput(roger->button_reference[X],
                 roger->button_reference[Y], kEnvironment, &x, &y) == FALSE)
          break;

          //      int xbin, ybin; already defined above
          printf("Map editor input - x: %4.3f, y: %4.3f - button: %d\n",
           x, y, roger->button_event);

          if (kEnvironment == ARENA) {
            xbin = (x - MIN_X) / XDELTA;
            ybin = NBINS - (y - MIN_Y) / YDELTA;
          } else {
            xbin = (x - MIN_X_DEV) / XDELTA_DEV;
            ybin = NBINS - (y - MIN_Y_DEV) / YDELTA_DEV;
          }


          if ((xbin<0) || (xbin>(NBINS-1)) || (ybin<0) || (ybin>(NBINS-1))) {
          printf("Out of the boundary!!!\n");
              }
          else {
            if (roger->button_event==LEFT_BUTTON) {// obstacles in Cartesian space
              if (roger->world_map.occupancy_map[ybin][xbin] == OBSTACLE) {
                printf("deleting an obstacle xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                roger->world_map.occupancy_map[ybin][xbin] = FREESPACE;
                //      delete_bin_bumper(xbin,ybin);
              }
              else if ((roger->world_map.occupancy_map[ybin][xbin] == FREESPACE) ||
                        (roger->world_map.occupancy_map[ybin][xbin] == DILATED_OBSTACLE)) {
                printf("inserting an obstacle xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                roger->world_map.occupancy_map[ybin][xbin] = OBSTACLE;
                roger->world_map.potential_map[ybin][xbin] = 1.0;
                roger->world_map.color_map[ybin][xbin] = LIGHTBLUE;
              }
            }
            else if (roger->button_event == MIDDLE_BUTTON) { }
            else if (roger->button_event == RIGHT_BUTTON) {
              if (roger->world_map.occupancy_map[ybin][xbin] == GOAL) {
                printf("deleting an goal xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                int row, col;
                for(row=xbin-1; row<=xbin+1; row++) {
                  for(col=ybin-1; col<=ybin+1; col++) {
                    roger->world_map.occupancy_map[col][row] = FREESPACE;
                  }
                }
              }
              else if (roger->world_map.occupancy_map[ybin][xbin] == FREESPACE) {
                printf("inserting an goal xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                int row, col;
                for(row=xbin-1; row<=xbin+1; row++) {
                  for(col=ybin-1; col<=ybin+1; col++) {
                    roger->world_map.occupancy_map[col][row] = GOAL;
                    roger->world_map.potential_map[col][row] = 0.0;
                  }
                }
                }
              }

          }

        break;
        default:
        break;
      }
    }
  }
}


// TODO: see if we can use the below for initilization

// define geometrical and inertial parameters for the NBODY dynamical system
void initialize_toybox()
{
  /****************************** 0: CIRCLE **********************************/
  toybox[CIRCLE].id = toy_home_circle.id;
  toybox[CIRCLE].N = 1;
  for (int i = 0; i < NUM_VERTICES; ++i) {
    toybox[CIRCLE].vertices[i][0] = toy_home_circle.vertices[i][0];
    toybox[CIRCLE].vertices[i][1] = toy_home_circle.vertices[i][1];
    toybox[CIRCLE].radii[i] = toy_home_circle.radii[i];
  }
  
  toybox[CIRCLE].N_draw = toy_home_circle.N_draw;

  for (int i = 0; i < NUM_DRAW_VERTS; ++i) {
    toybox[CIRCLE].draw_verts[i][0] = toy_home_circle.draw_verts[i][0];
    toybox[CIRCLE].draw_verts[i][1] = toy_home_circle.draw_verts[i][1];
  }

  toybox[CIRCLE].color = toy_home_circle.color;
  toybox[CIRCLE].image_radius = toy_home_circle.image_radius;
  toybox[CIRCLE].mass = M_BALL;
  toybox[CIRCLE].moi = I_BALL;
  toybox[CIRCLE].position[X] = toy_home_circle.position[X];
  toybox[CIRCLE].position[Y] = toy_home_circle.position[Y];
  toybox[CIRCLE].position[THETA] = 0.0;
  toybox[CIRCLE].velocity[X] = toybox[CIRCLE].velocity[Y] = toybox[CIRCLE].velocity[THETA] = 0.0;
  toybox[CIRCLE].net_extForce[X] = toybox[CIRCLE].net_extForce[Y] = toybox[CIRCLE].net_extForce[THETA] = 0.0;

  /**************************** 1: TRIANGLE **********************************/

  toybox[TRIANGLE].id = toy_home_triangle.id;
  toybox[TRIANGLE].N = toy_home_triangle.N;
  for (int i = 0; i < NUM_VERTICES; ++i) {
    toybox[TRIANGLE].vertices[i][0] = toy_home_triangle.vertices[i][0];
    toybox[TRIANGLE].vertices[i][1] = toy_home_triangle.vertices[i][1];
    toybox[TRIANGLE].radii[i] = toy_home_triangle.radii[i];
  }

  toybox[TRIANGLE].N_draw = toy_home_triangle.N_draw;

  for (int i = 0; i < NUM_DRAW_VERTS; ++i) {
    toybox[TRIANGLE].draw_verts[i][0] = toy_home_triangle.draw_verts[i][0];
    toybox[TRIANGLE].draw_verts[i][1] = toy_home_triangle.draw_verts[i][1];
  }
  toybox[TRIANGLE].color = toy_home_triangle.color;
  toybox[TRIANGLE].image_radius = toy_home_triangle.image_radius;
  toybox[TRIANGLE].mass = M_TRIANGLE;
  toybox[TRIANGLE].moi = I_TRIANGLE;
  toybox[TRIANGLE].position[X] = toy_home_triangle.position[X];
  toybox[TRIANGLE].position[Y] = toy_home_triangle.position[Y];
  toybox[TRIANGLE].position[THETA] = 0.0;
  toybox[TRIANGLE].velocity[X] = toybox[TRIANGLE].velocity[Y] = toybox[TRIANGLE].velocity[THETA] = 0.0;
  toybox[TRIANGLE].net_extForce[X] = toybox[TRIANGLE].net_extForce[Y] = toybox[TRIANGLE].net_extForce[THETA] = 0.0;

  /**************************** 2: SQUARE V1 **********************************/

  toybox[SQUARE].id = toy_home_square.id;
  toybox[SQUARE].N = toy_home_square.N;
  for (int i = 0; i < NUM_VERTICES; ++i) {
    toybox[SQUARE].vertices[i][0] = toy_home_square.vertices[i][0];
    toybox[SQUARE].vertices[i][1] = toy_home_square.vertices[i][1];
    toybox[SQUARE].radii[i] = toy_home_square.radii[i];
  }

  toybox[SQUARE].N_draw = toy_home_square.N_draw;

  for (int i = 0; i < NUM_DRAW_VERTS; ++i) {
    toybox[SQUARE].draw_verts[i][0] = toy_home_square.draw_verts[i][0];
    toybox[SQUARE].draw_verts[i][1] = toy_home_square.draw_verts[i][1];
  }
  toybox[SQUARE].color = toy_home_square.color;
  toybox[SQUARE].image_radius = toy_home_square.image_radius;
  toybox[SQUARE].mass = M_TRIANGLE;
  toybox[SQUARE].moi = I_TRIANGLE;
  toybox[SQUARE].position[X] = toy_home_square.position[X];
  toybox[SQUARE].position[Y] = toy_home_square.position[Y];
  toybox[SQUARE].position[THETA] = 0.0;
  toybox[SQUARE].velocity[X] = toybox[SQUARE].velocity[Y] = toybox[SQUARE].velocity[THETA] = 0.0;
  toybox[SQUARE].net_extForce[X] = toybox[SQUARE].net_extForce[Y] = toybox[SQUARE].net_extForce[THETA] = 0.0;

  /**************************** 3: ISO TRIANGLE **********************************/

  toybox[ISO_TRIANGLE].id = toy_home_iso_triangle.id;
  toybox[ISO_TRIANGLE].N = toy_home_iso_triangle.N;
  for (int i = 0; i < NUM_VERTICES; ++i) {
    toybox[ISO_TRIANGLE].vertices[i][0] = toy_home_iso_triangle.vertices[i][0];
    toybox[ISO_TRIANGLE].vertices[i][1] = toy_home_iso_triangle.vertices[i][1];
    toybox[ISO_TRIANGLE].radii[i] = toy_home_iso_triangle.radii[i];
  }

  toybox[ISO_TRIANGLE].N_draw = toy_home_iso_triangle.N_draw;

  for (int i = 0; i < NUM_DRAW_VERTS; ++i) {
    toybox[ISO_TRIANGLE].draw_verts[i][0] = toy_home_iso_triangle.draw_verts[i][0];
    toybox[ISO_TRIANGLE].draw_verts[i][1] = toy_home_iso_triangle.draw_verts[i][1];
  }
  toybox[ISO_TRIANGLE].color = toy_home_iso_triangle.color;
  toybox[ISO_TRIANGLE].image_radius = toy_home_iso_triangle.image_radius;
  toybox[ISO_TRIANGLE].mass = M_TRIANGLE;
  toybox[ISO_TRIANGLE].moi = I_TRIANGLE;
  toybox[ISO_TRIANGLE].position[X] = toy_home_iso_triangle.position[X];
  toybox[ISO_TRIANGLE].position[Y] = toy_home_iso_triangle.position[Y];
  toybox[ISO_TRIANGLE].position[THETA] = 0.0;
  toybox[ISO_TRIANGLE].velocity[X] = toybox[ISO_TRIANGLE].velocity[Y] = toybox[ISO_TRIANGLE].velocity[THETA] = 0.0;
  toybox[ISO_TRIANGLE].net_extForce[X] = toybox[ISO_TRIANGLE].net_extForce[Y] = toybox[ISO_TRIANGLE].net_extForce[THETA] = 0.0;

  /**************************** 4: PENTAGON **********************************/

  toybox[PENTAGON].id = toy_home_pentagon.id;
  toybox[PENTAGON].N = toy_home_pentagon.N;
  for (int i = 0; i < NUM_VERTICES; ++i) {
    toybox[PENTAGON].vertices[i][0] = toy_home_pentagon.vertices[i][0];
    toybox[PENTAGON].vertices[i][1] = toy_home_pentagon.vertices[i][1];
    toybox[PENTAGON].radii[i] = toy_home_pentagon.radii[i];
  }

  toybox[PENTAGON].N_draw = toy_home_pentagon.N_draw;

  for (int i = 0; i < NUM_DRAW_VERTS; ++i) {
    toybox[PENTAGON].draw_verts[i][0] = toy_home_pentagon.draw_verts[i][0];
    toybox[PENTAGON].draw_verts[i][1] = toy_home_pentagon.draw_verts[i][1];
  }
  toybox[PENTAGON].color = toy_home_pentagon.color;
  toybox[PENTAGON].image_radius = toy_home_pentagon.image_radius;
  toybox[PENTAGON].mass = M_TRIANGLE;
  toybox[PENTAGON].moi = I_TRIANGLE;
  toybox[PENTAGON].position[X] = toy_home_pentagon.position[X];
  toybox[PENTAGON].position[Y] = toy_home_pentagon.position[Y];
  toybox[PENTAGON].position[THETA] = 0.0;
  toybox[PENTAGON].velocity[X] = toybox[PENTAGON].velocity[Y] = toybox[PENTAGON].velocity[THETA] = 0.0;
  toybox[PENTAGON].net_extForce[X] = toybox[PENTAGON].net_extForce[Y] = toybox[PENTAGON].net_extForce[THETA] = 0.0;
}

void x_timer_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  int i, j, reset;
  static int render = RENDER_RATE, servo = SERVO_RATE, image = IMAGE_RATE;

  /***** control loop *****************************************************/
  if (servo++ == SERVO_RATE) {
    reset = FALSE;
    write_interface(reset); // reset eliminates user goals and obstacles
                // from occupancy grids
    for (i = 0; i < numRoger; ++i) {
      if (socknew[i] > 0) {
        HandleGuiInteraction(&Rogers[i]);
        SocketCommunicate(&Rogers[i], socknew[i]);
      }
    }
    read_interface();
    servo = 1;
  }

  /***** simulate object *************************************************/
  /* writes collision forces into respective data structures *************/
  /* obstacle data structure is global ***********************************/
  compute_external_forces();
  
  
  // shouldn't have to edit this call or function at all
  //     UPDATE: maybe replace this functiobn with a simulate_toybox function
  //     Loop over toys
  // simulate_object_polyball(&toy); // just circles...dc polyball()
  simulate_object_toys();

  for (i = 0; i < numRoger; ++i) {
    simulate_roger(&RogersBodys[i].mobile_base, RogersBodys[i].arms,
		   RogersBodys[i].eyes);
  }

  if (image++ == IMAGE_RATE) {
    for (i = 0; i < numRoger; ++i) {
      make_images(i);
    }
    image = 1;
  }

  if (render++ == RENDER_RATE) {
    for (i = 0; i < numRoger; ++i) {
      if ((HISTORY) && (history_ptr[i] < MAX_HISTORY)) {
        history[i][history_ptr[i]].arm_pos[LEFT][0]
	  = RogersBodys[i].arms[LEFT][1].theta;
        history[i][history_ptr[i]].arm_pos[LEFT][1]
	  = RogersBodys[i].arms[LEFT][2].theta;

        history[i][history_ptr[i]].arm_pos[RIGHT][0]
	  = RogersBodys[i].arms[RIGHT][1].theta;
        history[i][history_ptr[i]].arm_pos[RIGHT][1]
	  = RogersBodys[i].arms[RIGHT][2].theta;

        history[i][history_ptr[i]].base_pos[0] = RogersBodys[i].mobile_base.x;
        history[i][history_ptr[i]].base_pos[1] = RogersBodys[i].mobile_base.y;
        history[i][history_ptr[i]].base_pos[2]
	  = RogersBodys[i].mobile_base.theta;

        ++history_ptr[i];
      }
    }

    if (kEnvironment == DEVELOPMENT) {
      draw_all_dev();
    }
    else draw_all();

    render = 1;
  }

  simtime += DT;

  timer = XtAppAddTimeOut(app_con, TIMER_UPDATE, x_timer_proc,
			  (XtPointer)NULL);
}


void x_start_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  if (!timer) {
    XkwSetWidgetLabel(start_w, "Stop");
    timer = XtAppAddTimeOut(app_con, TIMER_UPDATE, x_timer_proc,
      (XtPointer)NULL);
  }
  else {
    XkwSetWidgetLabel(start_w, "Start");
    XtRemoveTimeOut(timer);
    timer = 0;
  }
}


void x_control_mode_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  Rogers[0].control_mode = change_control_mode_dev();

  switch (Rogers[0].control_mode) {
     case PROJECT1:
       XkwSetWidgetLabel(control_mode_w, "1-MotorUnits"); break;
     case PROJECT2:
       XkwSetWidgetLabel(control_mode_w, "2-ArmKinematics"); break;
     case PROJECT3:
       XkwSetWidgetLabel(control_mode_w, "3-Vision"); break;
     case PROJECT4:
       XkwSetWidgetLabel(control_mode_w, "4-SearchTrack"); break;
     case PROJECT5:
       XkwSetWidgetLabel(control_mode_w, "5-StereoKinematics"); break;
     case PROJECT6:
       XkwSetWidgetLabel(control_mode_w, "6-Kalman"); break;
     case PROJECT7:
       XkwSetWidgetLabel(control_mode_w, "7-ChasePunch"); break;
     case PROJECT8:
       XkwSetWidgetLabel(control_mode_w, "8-PathPlanning"); break;
     case PROJECT9:
       XkwSetWidgetLabel(control_mode_w, "9-Grasp"); break;
     case PROJECT10:
       XkwSetWidgetLabel(control_mode_w, "10-Transport"); break;
     case PROJECT11:
       XkwSetWidgetLabel(control_mode_w, "11-Belief"); break;
     case PROJECT12:
       XkwSetWidgetLabel(control_mode_w, "12-PONG"); break;
     default: break;
  }
  //call init here makes it independent of timer running
  initialize_control_mode(&Rogers[0]);
}

int change_room_num()
{
  static int room_num;

  room_num = (room_num + 1) % 11;
  return(room_num);
}

void x_room_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  void initialize_room();
  Rogers[0].room_num = change_room_num();
  
  switch (Rogers[0].room_num) {
     case R0:
       XkwSetWidgetLabel(room_w, "Room: 0");
       break;
     case R1:
       XkwSetWidgetLabel(room_w, "Room: 1");
       break;
     case R2:
       XkwSetWidgetLabel(room_w, "Room: 2");
       break;
     case R3:
        XkwSetWidgetLabel(room_w, "Room: 3");
        break;
      case R4:
        XkwSetWidgetLabel(room_w, "Room: 4");
        break;
      case R5:
        XkwSetWidgetLabel(room_w, "Room: 5");
        break;
      case R6:
        XkwSetWidgetLabel(room_w, "Room: 6");
        break;
      case R7:
        XkwSetWidgetLabel(room_w, "Room: 7");
        break;
      case R8:
        XkwSetWidgetLabel(room_w, "Room: 8");
        break;
      case R9:
        XkwSetWidgetLabel(room_w, "Room: 9");
        break;
      case R10:
        XkwSetWidgetLabel(room_w, "Room: 10");
        break;
     default:
       break;
  }
  initialize_room(&Rogers[0]);
}

void initialize_room_overlay(roger)
Robot * roger;
{
  FILE *fp;
  // Pixmap pixmapOverlay = XCreatePixmap(display, window, width_pixmap, height_pixmap, );


  switch (roger->room_num) {
    case R0:
      fp = fopen("../RogerProjects/ROOMS/merged_aspectsA.txt", "r");
      break;
    case R1:
      fp = fopen("../RogerProjects/ROOMS/merged_aspectsB.txt", "r");
      break;
    case R2:
      fp = fopen("../RogerProjects/ROOMS/merged_aspectsC.txt", "r");
      break;
    case R3:
      fp = fopen("../RogerProjects/ROOMS/merged_aspectsD.txt", "r");
      break;
    case R4:
      fp = fopen("../RogerProjects/ROOMS/merged_aspectsE.txt", "r");
      break;
    case R5:
      fp = fopen("../RogerProjects/ROOMS/merged_aspectsF.txt", "r");
      break;
    case R6:
      fp = fopen("../RogerProjects/ROOMS/merged_aspectsG.txt", "r");
      break;
    case R7:
      fp = fopen("../RogerProjects/ROOMS/merged_aspectsH.txt", "r");
      break;
    case R8:
      fp = fopen("../RogerProjects/ROOMS/R0.txt", "r");
      break;
    case R9:
      fp = fopen("../RogerProjects/ROOMS/R0.txt", "r");
      break;
    case R10:
      fp = fopen("../RogerProjects/ROOMS/h.txt", "r");
      break;
    default:
      break;
    }

    char room_array[64][64];
    char room_array_full[128][128];
    char * line = NULL;
    size_t len = 64;
    int x = 0;
    size_t read = 0;

    while ((read = getline(&line, &len, fp) != EOF)) {
      x++;
      if (x < 65) {
        for (int y = 0; y < 65; y++) {
          //rotation- (64,0) -> (64,64) and (64,64) -> (0,64)
          room_array[y][64-x] = line[y];
          // printf("%c", room_array[x][y]);
        }
      }
    }

    //"expands" room array to full size. janky :/
    for(int x = 0; x < 128; x++) {
      for(int y = 0; y < 128; y++) {
        room_array_full[x][y] = room_array[x / 2][y / 2];
        // printf("%c", room_array_full[x][y]);
      }
    }
    
    for (int x=0; x < 128; x += 1) {
      for (int y=0; y < 128; y += 1) {
        // printf("%d, %d\n", x, y);
        switch(room_array_full[x][y]) {
          case 'B': {
            x_draw_pixel(DARKBLUE, (double) (x-64)*(2.0/64), (double) (y-64)*(2.0/64) + 2.0/64 - 2.0/64/8);
            break;
          }
          case 'G': {
            x_draw_pixel(GREEN, (double) (x-64)*(2.0/64), (double) (y-64)*(2.0/64) + 2.0/64 - 2.0/64/8);
            break;
          }
          case 'R': {
            x_draw_pixel(RED, (double) (x-64)*(2.0/64), (double) (y-64)*(2.0/64) + 2.0/64 - 2.0/64/8);
            break;
          }
          case 'Y': {
            x_draw_pixel(YELLOW, (double) (x-64)*(2.0/64), (double) (y-64)*(2.0/64) + 2.0/64 - 2.0/64/8);
            break;
          }
          case 'K': {
            x_draw_pixel(LIGHTRED, (double) (x-64)*(2.0/64), (double) (y-64)*(2.0/64) + 2.0/64 - 2.0/64/8);
            break;
          }
          case 'O': {
            x_draw_pixel(ORANGE, (double) (x-64)*(2.0/64), (double) (y-64)*(2.0/64) + 2.0/64 - 2.0/64/8);
            break;
          }
          case 'V': {
            x_draw_pixel(DARKGREEN, (double) (x-64)*(2.0/64), (double) (y-64)*(2.0/64) + 2.0/64 - 2.0/64/8);
            break;
          }
          case 'A': {
            x_draw_pixel(AQUA, (double) (x-64)*(2.0/64), (double) (y-64)*(2.0/64) + 2.0/64 - 2.0/64/8);
            break;
          }
          case 'S': {
            x_draw_pixel(LIGHTBLUE, (double) (x-64)*(2.0/64), (double) (y-64)*(2.0/64) + 2.0/64 - 2.0/64/8);
            break;
          }
          case 'P': {
            x_draw_pixel(PURPLE, (double) (x-64)*(2.0/64), (double) (y-64)*(2.0/64) + 2.0/64 - 2.0/64/8);
            break;
          }
          case 'M': {
            x_draw_pixel(MAGENTA, (double) (x-64)*(2.0/64), (double) (y-64)*(2.0/64) + 2.0/64 - 2.0/64/8);
            break;
          }
          default: {
            x_draw_pixel(100, (double) (x-64)*(2.0/64), (double) (y-64)*(2.0/64) + 2.0/64 - 2.0/64/8);
            break;
          }
        }
      }
    }
    fclose(fp);
}

void initialize_room(roger)
Robot * roger;
{
  int i, j;
  FILE *fp;
  char line[NBINS + 2];

  printf("%d", roger->room_num);
  // read in appropriate Room file selected by user
  switch (roger->room_num) {
  case R0:
    fp = fopen("../RogerProjects/ROOMS/R0.txt", "r");
    break;
  case R1:
    fp = fopen("../RogerProjects/ROOMS/R1.txt", "r");
    break;
  case R2:
    fp = fopen("../RogerProjects/ROOMS/R2.txt", "r");
    break;
  case R3:
    fp = fopen("../RogerProjects/ROOMS/a.txt", "r");
    break;
  case R4:
    fp = fopen("../RogerProjects/ROOMS/b.txt", "r");
    break;
  case R5:
    fp = fopen("../RogerProjects/ROOMS/c.txt", "r");
    break;
  case R6:
    fp = fopen("../RogerProjects/ROOMS/d.txt", "r");
    break;
  case R7:
    fp = fopen("../RogerProjects/ROOMS/e.txt", "r");
    break;
  case R8:
    fp = fopen("../RogerProjects/ROOMS/f.txt", "r");
    break;
  case R9:
    fp = fopen("../RogerProjects/ROOMS/g.txt", "r");
    break;
  case R10:
    fp = fopen("../RogerProjects/ROOMS/h.txt", "r");
    break;
  default:
    break;
  }

  // Initialize world geometry according to Room file
  for (i = 0; i<NBINS; i++) {
    fgets(line, NBINS + 2, fp);
    for (j = 0; j<NBINS; j++) {
      roger->world_map.occupancy_map[i][j] = FREESPACE;
      roger->world_map.potential_map[i][j] = 1.0;
      roger->arm_map[LEFT].occupancy_map[i][j] = FREESPACE;
      roger->arm_map[LEFT].potential_map[i][j] = 1.0;
      roger->arm_map[RIGHT].occupancy_map[i][j] = FREESPACE;
      roger->arm_map[RIGHT].potential_map[i][j] = 1.0;

      switch (line[j]) {
      case 'B':
        roger->world_map.occupancy_map[i][j] = OBSTACLE;
        roger->world_map.potential_map[i][j] = 1.0;
        roger->world_map.color_map[i][j] = DARKBLUE;
        break;
      case 'G':
        roger->world_map.occupancy_map[i][j] = OBSTACLE;
        roger->world_map.potential_map[i][j] = 1.0;
        roger->world_map.color_map[i][j] = LIGHTGREEN;
        break;
      case 'K':
        roger->world_map.occupancy_map[i][j] = OBSTACLE;
        roger->world_map.potential_map[i][j] = 1.0;
        roger->world_map.color_map[i][j] = 0;
        break;
      case 'R':
        roger->world_map.occupancy_map[i][j] = OBSTACLE;
        roger->world_map.potential_map[i][j] = 1.0;
        roger->world_map.color_map[i][j] = LIGHTRED;
        break;
      default:
        break;
      }
    }
  }
  fclose(fp);
}


void InitArenaEnv(argc, argv)
int* argc; char **argv;
{
  kEnvironment = ARENA;
  static String fallback_resources[] = {
    "*title:  Roger-the-Crab",
    "*Roger-the-Crab*x: 100",
    "*Roger-the-Crab*y: 100",
    NULL,
  };
  Widget toplevel, form, widget;
  void x_clear();

  toplevel = XtAppInitialize(&app_con, "Roger-the-Crab", NULL, ZERO, argc,
    argv, fallback_resources, NULL, ZERO);
  form = XkwMakeForm(toplevel);
  widget = NULL;
  start_w = widget = XkwMakeCommand(form, NULL, widget, x_start_proc,
				    "Start", BOXW, BOXH);
  input_mode_1_w = widget = 
    XkwMakeCommand(form, NULL, widget, x_input_mode_arena_proc,
		   "Input: Ball Position", BOXW, BOXH);
  control_mode_1_w = widget = 
    XkwMakeCommand(form, NULL, widget, x_control_mode_arena_proc,
		   "Control: 1-Motor Units", BOXW, BOXH);
  // input_mode_2_w = widget = 
  //   XkwMakeCommand(form, NULL, widget, x_input_mode_2_proc,
  //                  "Input 2: Joint angles", BOXW, BOXH);
  // control_mode_2_w = widget = 
  //   XkwMakeCommand(form, NULL, widget, x_control_mode_2_proc,
  //                  "Control 2: ChasePunch", BOXW, BOXH);
  // params_w = widget = XkwMakeCommand(form, NULL, widget, x_params_proc,
  //            "Enter Params", BOXW, BOXH);
  // stream_w = widget = XkwMakeCommand(form, NULL, widget, x_visualize_proc,
  //            "Visualize",  BOXW, BOXH);
  widget = XkwMakeCommand(form, NULL, widget, x_quit_proc, "Quit", BOXW, BOXH);
  canvas_w = widget = XkwMakeCanvas(form, widget, NULL, x_canvas_proc,
				    width, height);
  XtRealizeWidget(toplevel);
  display = XtDisplay(canvas_w);
  window = XtWindow(canvas_w);
  screen = DefaultScreen(display);
  depth = DefaultDepth(display, screen);
  foreground = BlackPixel(display, screen);
  background = WhitePixel(display, screen);

  XGCValues gcv;
  gcv.fill_style = 2;
  gc = XCreateGC(display, window, 0, NULL);
  gcoverlay = XCreateGC(display, overlay, 0, NULL);
  XChangeGC(display, gcoverlay, GCFunction, &gcv);
  XSetFunction(display, gc, GXcopy);
  XSetForeground(display, gc, foreground);
  XSetBackground(display, gc, background);

  pixmap = XCreatePixmap(display, window, width_pixmap, height_pixmap, depth);
  x_clear();

  x_init_colors();

  int i;
  for (i = 0; i < numRoger; ++i) {
    initialize_room(&Rogers[i]);
  }

  reset = TRUE;
  
  // this function eventually calls intialize_internal_objects, which intializes the toy
  initialize_simulator(reset); // initializes world boundaries,
                               // mobile_base, eyes[2], arms[2], and
                               // Roger interface structure

  // shouldn't have to edit this call or function at all
  //     UPDATE: maybe replace this functiobn with a simulate_toybox function
  // simulate_object_polyball(&toy); //remove polyball...toybox?
  simulate_object_toys();


  for (i = 0; i < numRoger; ++i) {
    // initialize_room(&Rogers[i]);
    simulate_roger(&RogersBodys[i].mobile_base, 
		    RogersBodys[i].arms, RogersBodys[i].eyes);
    initialize_control_mode(&Rogers[i]);
  }

  for (i = 0; i < numRoger; ++i) make_images(i);

  draw_all();
  XtAppMainLoop(app_con);
}

void InitDevelopmentEnv(argc, argv)
int* argc; char **argv;
{
  kEnvironment = DEVELOPMENT;
  width = (int)(1.0* WIDTH_DEV);
  height = (int)(1.0*HEIGHT_DEV);
  width_pixmap = (int)((float)(WIDTH)* PIX_MAP_SIZE_RATIO);
  height_pixmap = (int)((float)(HEIGHT)* PIX_MAP_SIZE_RATIO);

  static String fallback_resources[] = { "*title:  Roger-the-Crab",
    "*Roger-the-Crab*x: 100",
    "*Roger-the-Crab*y: 100",NULL, };
  Widget toplevel, form, widget;
  void x_clear();

  //  void intersect_line_circle();
  //  intersect_line_circle();

  toplevel = XtAppInitialize(&app_con, "Roger-the-Crab", NULL, ZERO,
    argc, argv, fallback_resources, NULL, ZERO);


  form = XkwMakeForm(toplevel);
  widget = NULL;
  start_w = widget =
    XkwMakeCommand(form, NULL, widget, x_start_proc, "Start", BOXW_DEV, BOXH);
  input_mode_w = widget =
    XkwMakeCommand(form, NULL, widget, x_input_mode_dev_proc,
		   "Input: Joint angles", BOXW_DEV, BOXH);
  control_mode_w = widget =
    XkwMakeCommand(form, NULL, widget, x_control_mode_proc,
		   "1-Motor Units", BOXW_DEV, BOXH);
  room_w = widget =
    XkwMakeCommand(form, NULL, widget, x_room_proc, "Room: 0", BOXW_DEV, BOXH);
  params_w = widget =
    XkwMakeCommand(form, NULL, widget, x_params_proc, "Enter Params",
		   BOXW_DEV, BOXH);
  stream_w = widget =
    XkwMakeCommand(form, NULL, widget, x_visualize_proc, "Visualize",
		   BOXW_DEV, BOXH);
  widget = XkwMakeCommand(form, NULL, widget, x_quit_proc, "Quit",
			  BOXW_DEV, BOXH);
  canvas_w = widget =
    XkwMakeCanvas(form, widget, NULL, x_canvas_proc, width, height);
  XtRealizeWidget(toplevel);
  display = XtDisplay(canvas_w);
  window = XtWindow(canvas_w);
  screen = DefaultScreen(display);
  depth = DefaultDepth(display, screen);
  foreground = BlackPixel(display, screen);
  background = WhitePixel(display, screen);

  gc = XCreateGC(display, window, 0, NULL);
  XSetFunction(display, gc, GXcopy);
  XSetForeground(display, gc, foreground);
  XSetBackground(display, gc, background);

  pixmap = XCreatePixmap(display, window, width_pixmap, height_pixmap, depth);
  x_clear();

  x_init_colors();

  reset = TRUE;

  initialize_room(&Rogers[0]);

  // this function eventually calls intialize_internal_objects, which intializes the toy
  initialize_simulator(reset);

  //This is where this should be normally, when not rendering roger on top of the visualize func
  simulate_roger(&RogersBodys[0].mobile_base, RogersBodys[0].arms,
		  RogersBodys[0].eyes);
  initialize_control_mode(&Rogers[0]);

  // shouldn't have to edit this call or function at all
  //     UPDATE: maybe replace this functiobn with a simulate_toybox function
  // simulate_object_polyball(&toy);
  simulate_object_toys();
  
  
  make_images(0);

  draw_all_dev();
  // This is not where this should be (see above) but it renders roger on top of the visualize f
  // simulate_roger(&RogersBodys[0].mobile_base, RogersBodys[0].arms,
	// 	  RogersBodys[0].eyes);

  XtAppMainLoop(app_con);
}

int main(argc, argv)
int argc; char **argv;
{
  int i, chosen_env;

  // should there be a default?
  if (argc != 3) {
    printf("Usage: ./simulator EnvironmentNum RobotNum.\n");
    return 0;
  }

  numRoger = atoi(argv[2]);
  chosen_env = atoi(argv[1]);
  numObjects = 3 * numRoger;
  n_active_toys = 0; // just a double check

  // Check the correctness of input arguments
  if (numRoger > MaxRobotNum) {
    printf("Simulation limited to %d robots \n", MaxRobotNum);
    return 0;
  }

  if ( (chosen_env != ARENA) && (chosen_env != DEVELOPMENT) ) {
    printf("Undefined environment!\n ARENA : 0, DEVELOPMENT: 1\n");
    return 0;
  }

  if ( (chosen_env == DEVELOPMENT) && (numRoger > 1) ) {
    printf("Only 1 robot is supported in the DEVELOPMENT environment\n");
    return 0;
  }


  for (i = 0; i < numRoger; ++i) {
    SocketInit(ServerPorts[i], &socknew[i]);
  }


  if (chosen_env == ARENA) InitArenaEnv(&argc, argv);
  else InitDevelopmentEnv(&argc, argv);

}
