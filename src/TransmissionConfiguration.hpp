#include "main.h"
#include "TransmissionMath_lib.hpp"

/****************************************************************************************************************
* 2022 Game field coodinate system setting:
*****************************************************************************************************************/
using namespace pros;

#define WHEEL_CIRCUMFERENCE_CM 31.918
#define ENCODER_CIRCUMFERENCE_CM 21.944

#define ENCODER_X_FROM_CENTRE 2.5
#define ENCODER_Y_FROM_CENTRE -4
// MEASURE WHEN THE ROBOT FACES TO THE Opponent ie. 90 degrees
// ENSURE THAT Y DIFFERENCE IS THE VERTICAL AND X DIFFERENCE IS THE HORIZONTAL IN THIS ORIENTATION
// make either negative according to cartesian coordinates

#define DEBUG_DISPLAY_MASSAGE_OFF 0
#define DEBUG_DISPLAY_MASSAGE_ON 1

// coordinates of centre of robot
#define ROBOT_INITIAL_X 73.5
#define ROBOT_INITIAL_Y 28.5


#define ROBOT_INITIAL_HEADING 90

#define COODINATE_UPDATE_X 1
#define COODINATE_UPDATE_Y 2
#define COODINATE_UPDATE_XY 3
#define COODINATE_UPDATE_INCREMENT 4

#define SELECTION_AUTO 0
#define SELECTION_DRIVE 1

// port assignments //
#define RIGHT_FRONT_DRIVE_MOTOR_PORT 1
#define RIGHT_MID_DRIVE_MOTOR_PORT 3
#define RIGHT_BACK_DRIVE_MOTOR_PORT 5
#define LEFT_FRONT_DRIVE_MOTOR_PORT 6
#define LEFT_MID_DRIVE_MOTOR_PORT 8
#define LEFT_BACK_DRIVE_MOTOR_PORT 10
#define INTAKE_MOTOR_PORT 17
#define ARM_MOTOR_PORT 19


#define INERTIAL_SENSOR_PORT 9

#define FRONT_VISION_PORT 11
#define BACK_VISION_PORT 15

#define Y_ENCODER_TOP_PORT 'B'
#define Y_ENCODER_BOTTOM_PORT 'A'

#define PNEUMATIC_BACK_PORT1 'C'
#define PNEUMATIC_BACK_PORT2 'E'
#define PNEUMATIC_FRONT_PORT 'F'
#define PNEUMATIC_TRANSMISSION_PORT 'D'
#define PNEUMATIC_TOP_PORT 'G'
#define TOUCH_SENSOR_PORT 'H'
//#define LIMIT_SWITCH_PORT 'H'

///////////////////////////////////////////
#define MOVE_FORWARD 1
#define MOVE_BACKWARD -1
#define MOVE_RIGHT 1
#define MOVE_LEFT -1

#define ONE_WHEEL_TURN_FORWARD 2
#define ONE_WHEEL_TURN_BACKWARD 3
#define TWO_WHEEL_TURN_FORWARD 2
#define TWO_WHEEL_TURN_BACKWARD 3
#define ON_SPOT_TURN 1

#define COUNTER_CLOCKWISE -1
#define CLOCKWISE 1
#define TURN_SMALLEST_ANGLE 0

#define FRONT_VISION_X_MIDDLE 158
#define BACK_VISION_X_MIDDLE 158

#define DETECT_NO_GOAL 0
#define DETECT_RED_GOAL_SIG 1
#define DETECT_BLUE_GOAL_SIG 2
#define DETECT_YELLOW_GOAL_SIG 3

//#define ARM_NOT_HOLDING_POSITION 0
#define ARM_HOLDING_POSITION 1
#define ARM_NOT_HOLDING_POSITION 2


/*********************************************************
* ARM related definitions
/*********************************************************/

#define PRESS_BRIDGE 380
#define RELEASE_BRIDGE 740
#define MAX_TOP 1000
#define CARRY_GOAL 250

struct Hardware
{
    pros::Motor leftFrontMotorLib;
    pros::Motor leftBackMotorLib;
    pros::Motor leftMidMotorLib;
    pros::Motor rightFrontMotorLib;
    pros::Motor rightBackMotorLib;
    pros::Motor rightMidMotorLib;
    pros::Motor intakeMotorLib;
    pros::Motor armMotorLib;
    pros::ADIDigitalOut frontPistonLib;
    pros::ADIDigitalOut backPistonLib1;
    pros::ADIDigitalOut backPistonLib2;
    pros::ADIDigitalOut transmissionPistonLib;
    pros::Imu inertialSensorLib;
    pros::ADIEncoder Y_encoderLib;
};

//
typedef struct
{
    bool hold_heading_lib;
    float holding_angle_lib;
} HeadingStruct;

//parallel movements structures

typedef struct
{
    long startingDelay_ms;
    bool actionType; // true = activated, false = deactivated
    int runTimes;
} HookAction;

typedef struct
{
    long startingDelay_ms;
    bool actionType;
    int runTimes;
} ClawAction;

typedef struct
{
    int motorPower;
    long startingDelay_ms;
    int desiredAbsolute;
    int runTimes;
} ArmAction;

typedef struct
{
    int motorPower;
    long startingDelay_ms;
    long lastingTime_ms;
    int endPower;
    int runTimes;
} IntakeAction;


typedef struct {
  // Object signature
  int signature;
  // left boundary coordinate of the object
  int left_coord;
  // top boundary coordinate of the object
  int top_coord;
  // width of the object
  int width;
  // height of the object
  int height;
  // coordinates of the middle of the object (computed from the values above)
  int x_middle_coord;
  int y_middle_coord;
} detected_vision_goal_lib;


extern int sys_display_info_terminal;

// motor declarations //
extern pros::Motor left_front_motor;
extern pros::Motor right_front_motor;
extern pros::Motor left_mid_motor;
extern pros::Motor right_mid_motor;
extern pros::Motor left_back_motor;
extern pros::Motor right_back_motor;
extern pros::Motor intake_motor;
extern pros::Motor arm_motor;
extern pros::ADIDigitalOut back_piston1;
extern pros::ADIDigitalOut back_piston2;
extern pros::ADIDigitalOut front_piston;
extern pros::ADIDigitalOut transmission_piston;
extern pros::ADIDigitalOut top_piston;

extern pros::Imu inertial_sensor;
extern pros::ADIPort touch_sensor;
extern pros::ADIDigitalIn limit_switch;

extern pros::ADIEncoder Y_encoder;

extern pros::Controller master;

//extern bool door_status;

// This variable defines the drifting angle during the period
// after Inertial sensor reset in Initialization() and before the starting of auton.
extern double sys_initial_to_auton_drifting;
extern double sys_initial_robot_heading;
extern double motion_initial_angle;   //used by position tracing system,
                                      //value changed in motion function.

extern Hardware hardwareParameter;
extern Point sys_coordinates;
//extern Point sys_encoderPosition;
extern bool coordinateAccessAllowed;

extern long flyWheelSet;
extern long HOOKSet;
extern long intakesSet;
extern long driveSet;

extern ClawAction clawAction_1;
extern ClawAction clawAction_2;
extern ClawAction clawAction_3;
extern int cmd_flyWheel_run;

extern HookAction hookAction_1;
extern HookAction hookAction_2;
extern HookAction hookAction_3;
extern int cmd_elevator_run;

extern IntakeAction intakeAction_1;
extern IntakeAction intakeAction_2;
extern IntakeAction intakeAction_3;
extern int cmd_intake_run;

extern ArmAction armAction_1;
extern ArmAction armAction_2;
extern ArmAction armAction_3;
extern int cmd_arm_run;
extern bool checkpointBreak;

extern int arm_action_lib;
//extern double arm_hold_target_angle_lib;
extern double arm_move_target_angle_lib;
extern int arm_move_speed_lib;

extern bool clawBreak;
extern bool hookBreak;
extern bool intakeBreak;
extern bool armBreak;

//auton tasks for multitasking
extern void hook_set_fn(void *param);
extern void claw_set_fn(void *param);
extern void intake_set_fn(void *param);
extern void arm_set_fn(void *param);
extern void background_command_fn(void *param);
extern void breakAll();

extern long checkpoint;
extern int auton_15_command;

extern pros::Task intake_set;
extern pros::Task hook_set;
extern pros::Task claw_set;
extern pros::Task arm_set;


extern pros::Vision front_vision;
extern pros::Vision back_vision;

void waitForTouch();

double convert_target_to_relative_angle_lib(double currentRelativeHeading, double targetHeading);

void goStraightCmPID_lib(double cmDistance, double robotHeadingLib, int maxSpeed, int robotDirection, double headingKP, double headingKI, double headingKD, double distanceKP, double distanceKI, double distanceKD, long timeoutMili, int exitConditionExpectedPasses, Hardware robot);

void turnDegreesPID_lib(double targetHeadingLib, double turnType, long maxPower, int turnDirection, double headingKP, double headingKI, double headingKD, long timeoutMili, int exitConditionExpectedPasses, Hardware config);

void twoWheelTurnDegreesPID(double targetHeadingLib, double turnType, double smallerPower, double largerPower, int turnDirection, double headingKP,
                            double headingKI, double headingKD, long timeoutMili, int exitConditionExpectedPasses, Hardware config);

double get_robot_heading_radians_lib(Hardware robot);
double get_robot_heading_lib(Hardware robot);


void update_coordinate(Point p);
void update_Coodinate(int updateType, double pos);
void update_coordinate(int updateType, double xPos, double yPos);



Point get_coordinate();

void odometry_fn(void *param);

void infoPrint_fn(void *param);

void goStraightCm_Front_Vision(double cmDistance, double robotInertialHeadingLib, int maxSpeed,
                         int goal_color_signature, Vision vision_sensor,
                         double headingKP, double headingKI, double headingKD,
                         double distanceKP, double distanceKI, double distanceKD,
                         double visionKP, double visionKI, double visionKD,
                         long timeoutMili, int exitConditionExpectedPasses, Hardware robot);

void goStraightCm_Back_Vision(double cmDistance, double robotInertialHeadingLib, int maxSpeed,
                         int goal_color_signature, Vision vision_sensor,
                         double headingKP, double headingKI, double headingKD,
                         double distanceKP, double distanceKI, double distanceKD,
                         double visionKP, double visionKI, double visionKD,
                         long timeoutMili, int exitConditionExpectedPasses, Hardware robot);

void search_Goal_Back_Vision(int maxSpeed, int searchDirection, double maxSearchintAngle,
                      double goal_min_width, int goal_color_signature, pros::Vision vision_sensor,
                      double visionKP, double visionKI, double visionKD,
                      double inertialKP, double inertialKI, double inertialKD,
                      long timeoutMili, int exitConditionExpectedPasses, Hardware robot);

void display_vision_error(int line, std::string msg);



void new_goStraightCm_Front_Vision(double cmDistance, double robotInertialHeadingLib, int maxSpeed,
                                   int goal_color_signature, Vision vision_sensor,
                                   double headingKP, double headingKI, double headingKD,
                                   double distanceKP, double distanceKI, double distanceKD,
                                   double visionKP, double visionKI, double visionKD,
                                   long timeoutMili, int exitConditionExpectedPasses, Hardware robot);


double get_distance_back_vision(Vision vision_sensor, int goal_color_signature,
	                              long samplingNumber, long sensingTimeInterval_Millis,
															  int min_width, long timeoutMillis);
                                
double get_distance_front_vision(Vision vision_sensor, int goal_color_signature,
	                              long samplingNumber, long sensingTimeInterval_Millis,
															  int min_width, long timeoutMillis);


void balance_bridge_PID_lib(int maxPower, double target_pitch, double balance_KP, double balance_KI, double balance_KD,
                            long timeoutMili, int exitConditionExpectedPasses, Hardware robot);


detected_vision_goal_lib get_goal_object_front_vision(int goal_color_signature);

typedef struct
{
  double targetHeadingLib;
  double turnType;
  long maxPower;
  int turnDirection;
  double headingKP;
  double headingKI;
  double headingKD;
  long timeoutMili;
  int exitConditionExpectedPasses;
} turnDegreesPID_cmd_struct;

extern turnDegreesPID_cmd_struct turnDegreesPID_Parameter;

void background_execution_turnDegreesPID_lib(void* param);
//bool is_limit_switch_pressed();
//void get_middle(int i);

