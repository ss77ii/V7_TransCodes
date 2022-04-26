#include "main.h"
#include "TransmissionConfiguration.hpp"

#define POSITION_TRACE_FN_ON_012345 ON

using namespace pros;

Point sys_coordinates = {ROBOT_INITIAL_X, ROBOT_INITIAL_Y};
//Point sys_encoderPosition;
bool coordinateAccessAllowed = true;

ClawAction clawAction_1 = {0, 0, 0};
ClawAction clawAction_2 = {0, 0, 0};
ClawAction clawAction_3 = {0, 0, 0};
ClawAction runTimes;

HookAction hookAction_1 = {0, 0, 0};
HookAction hookAction_2 = {0, 0, 0};
HookAction hookAction_3 = {0, 0, 0};

IntakeAction intakeAction_1 = {0, 0, 0, 0, 0};
IntakeAction intakeAction_2 = {0, 0, 0, 0, 0};
IntakeAction intakeAction_3 = {0, 0, 0, 0, 0};

ArmAction armAction_1 = {0, 0, 0, 0};
ArmAction armAction_2 = {0, 0, 0, 0};
ArmAction armAction_3 = {0, 0, 0, 0};

bool clawBreak = false;
bool hookBreak = false;
bool intakeBreak = false;
bool armBreak = false;

int arm_action_lib = ARM_NOT_HOLDING_POSITION;
//double arm_hold_target_angle_lib = 0;
double arm_move_target_angle_lib = 0;
int arm_move_speed_lib = 0;

void breakAll(){
  clawBreak = true;
  hookBreak = true;
  intakeBreak = true;
  armBreak = true;
}

void hookDelayWithBreak(long millis){
  long startingTime = pros::millis();
  while (true) {
    if(pros::millis() - startingTime >= millis){
      break;
    }
    if (hookBreak == true){
      break;
    }
    pros::delay(5);
  }
}

void hook_single_action(HookAction action){
  hookDelayWithBreak(action.startingDelay_ms);
  back_piston1.set_value(action.actionType);
  back_piston2.set_value(action.actionType);
}

/********************************************************************
simplifies setting the speed of the fly wheel
********************************************************************/
void hook_set_fn(void* param){
	hookAction_1 = {0, 0, 0};
	hookAction_2 = {0, 0, 0};
	hookAction_3 = {0, 0, 0};

	while(true){
    switch(hookAction_1.runTimes){
			case 0:
			  break;
			case 1:
        hook_single_action(hookAction_1);
        hookBreak = false;
        hookAction_1.runTimes = 0;
			  break;
			case 2:
        hook_single_action(hookAction_1);
        hook_single_action(hookAction_2);
        hookBreak = false;
        hookAction_1.runTimes = 0;
			  break;
			case 3:
        hook_single_action(hookAction_1);
        hook_single_action(hookAction_2);
        hook_single_action(hookAction_3);
        hookBreak = false;
        hookAction_1.runTimes = 0;
			  break;
		}
		pros::delay(10);
	}
}


void clawDelayWithBreak(long millis){
  long startingTime = pros::millis();
  while (true) {
    if(pros::millis() - startingTime >= millis){
      break;
    }
    if (clawBreak == true){
      break;
    }
    pros::delay(5);
  }
}

void claw_single_action(ClawAction action){
  clawDelayWithBreak(action.startingDelay_ms);
  front_piston.set_value(action.actionType);
}



/********************************************************************
simplifies setting the speed of the fly wheel
********************************************************************/
void claw_set_fn(void* param){
	clawAction_1 = {0, 0, 0};
	clawAction_2 = {0, 0, 0};
	clawAction_3 = {0, 0, 0};

	while(true){
    switch(clawAction_1.runTimes){
			case 0:
			  break;
			case 1:
        claw_single_action(clawAction_1);
        clawBreak = false;
        clawAction_1.runTimes = 0;
			  break;
			case 2:
        claw_single_action(clawAction_1);
        claw_single_action(clawAction_2);
        clawBreak = false;
        clawAction_1.runTimes = 0;
			  break;
			case 3:
        claw_single_action(clawAction_1);
        claw_single_action(clawAction_2);
        claw_single_action(clawAction_3);
        clawBreak = false;
        clawAction_1.runTimes = 0;
			  break;
		}
    pros::delay(10);
	}
}

void intakeDelayWithBreak(long millis){
  long startingTime = pros::millis();
  while (true) {
    if(pros::millis() - startingTime >= millis){
      break;
    }
    if (intakeBreak == true){
      break;
    }
    pros::delay(5);
  }
}

void intake_single_action(IntakeAction action){
  intakeDelayWithBreak(action.startingDelay_ms);
  intake_motor.move(action.motorPower);
  intakeDelayWithBreak(action.lastingTime_ms);
  intake_motor.move(action.endPower);
}

/********************************************************************
simplifies setting the speed of the fly wheel
********************************************************************/
void intake_set_fn(void* param){
	intakeAction_1 = {0, 0, 0, 0, 0};
	intakeAction_2 = {0, 0, 0, 0, 0};
	intakeAction_3 = {0, 0, 0, 0, 0};

	while(true){
    switch(intakeAction_1.runTimes){
			case 0:
			  break;
			case 1:
        intake_single_action(intakeAction_1);
        clawBreak = false;
        intakeAction_1.runTimes = 0;
			  break;
			case 2:
        intake_single_action(intakeAction_1);
        intake_single_action(intakeAction_2);
        clawBreak = false;
        intakeAction_1.runTimes = 0;
			  break;
			case 3:
        intake_single_action(intakeAction_1);
        intake_single_action(intakeAction_2);
        intake_single_action(intakeAction_3);
        clawBreak = false;
        intakeAction_1.runTimes = 0;
			  break;
		}
    pros::delay(10);
	}
}

void armDelayWithBreak(long millis){
  long startingTime = pros::millis();
  while (true) {
    if(pros::millis() - startingTime >= millis){
      break;
    }
    if (armBreak == true){
      break;
    }
    pros::delay(5);
  }
}

void arm_single_action(ArmAction action){
  armDelayWithBreak(action.startingDelay_ms);

  arm_move_speed_lib = action.motorPower;
  arm_move_target_angle_lib = action.desiredAbsolute;

  if(arm_action_lib == ARM_NOT_HOLDING_POSITION){
    arm_motor.move_absolute(action.desiredAbsolute, action.motorPower);
  }
}



/********************************************************************
simplifies setting the speed of the fly wheel
********************************************************************/
void arm_set_fn(void* param){
	armAction_1 = {0, 0, 0, 0};
	armAction_2 = {0, 0, 0, 0};
	armAction_3 = {0, 0, 0, 0};

	while(true){
    switch(armAction_1.runTimes){
			case 0:
			  break;
			case 1:
        arm_single_action(armAction_1);
        armBreak = false;
        armAction_1.runTimes = 0;
			  break;
			case 2:
        arm_single_action(armAction_1);
        arm_single_action(armAction_2);
        armBreak = false;
        armAction_1.runTimes = 0;
			  break;
			case 3:
        arm_single_action(armAction_1);
        arm_single_action(armAction_2);
        arm_single_action(armAction_3);
        armBreak = false;
        armAction_1.runTimes = 0;
			  break;
		}
    pros::delay(10);
	}
}

/***********************************************************************************
 TRANSMISSION TASK
 **********************************************************************************/

    /**********************************************************************************************************************************
      float degreeToDistance(float degreeRotated_lib)
        return distance when driving motor rotates degrees specified by variable degreeRotated_lib.
        its unit is centimeters;
    **********************************************************************************************************************************/
double degreeToDistanceCM(double degree_lib)
{
  return WHEEL_CIRCUMFERENCE_CM * degree_lib/360.0;
}

/**********************************************************************************************************************************
  float degreeToDistance(float degreeRotated_lib)
    return distance when driving motor rotates degrees specified by variable degreeRotated_lib.
    its unit is centimeters;
**********************************************************************************************************************************/
double degreeToDistanceCM(double degree_lib, double circumference_cm) {
	return circumference_cm * degree_lib/360.0;
}

void update_coordinate(Point p){
  while (true){
    if (coordinateAccessAllowed == true){
      coordinateAccessAllowed = false;
      sys_coordinates.x = p.x;
      sys_coordinates.y = p.y;
      coordinateAccessAllowed = true;
      break;
    }
  }
}

/*******************************************************************************************
* This function update the values of x and y coordinates of the robot center
*******************************************************************************************/
void update_coordinate(int updateType, double xPos, double yPos){
	while(true){
		if(coordinateAccessAllowed == true) {
			coordinateAccessAllowed = false;  //set this  value to avoid other tasks update the following variables
			break;
		}
	}

  if(updateType == COODINATE_UPDATE_XY){
    sys_coordinates.x = xPos;
  	sys_coordinates.y = yPos;
  } else if(updateType == COODINATE_UPDATE_INCREMENT){
    sys_coordinates.x = sys_coordinates.x + xPos;
  	sys_coordinates.y = sys_coordinates.y + yPos;
  }
  coordinateAccessAllowed = true; //free permission to allow other tasks update the variables
}

/*******************************************************************************************
* This function update one of the values for x and y coordinates of the robot center
*******************************************************************************************/
void update_Coodinate(int updateType, double pos){
	while(true){
		if(coordinateAccessAllowed == true) {
			coordinateAccessAllowed = false;  //set this  value to avoid other tasks update the following variables
			break;
		}
	}

  if(updateType == COODINATE_UPDATE_X) {
    sys_coordinates.x = pos;
  } else if(updateType == COODINATE_UPDATE_Y){
    sys_coordinates.y = pos;
  }
  coordinateAccessAllowed = true; //free permission to allow other tasks update the variables
}


Point get_coordinate(){
  Point p(0, 0);
  while (true){
    if (coordinateAccessAllowed == true){
      coordinateAccessAllowed = false;
      p.x = sys_coordinates.x;
      p.y = sys_coordinates.y;
      coordinateAccessAllowed = true;
      break;
    }
  }
  return p;
}


long num1 = 0;
long num2 = 0;
long num3 = 0;
double sum_x_Encoder_positive = 0;
double sum_x_Encoder_nagetive = 0;

void infoPrint_fn(void* param){
  while(true){
//    pros::lcd::print(0, "LF=%.1f, LM=%.1f,, LB=%.1f", left_front_motor.get_position(), left_mid_motor.get_position(), left_back_motor.get_position());
//    pros::lcd::print(1, "RF=%.1f, RM=%.1f,, RB=%.1f", right_front_motor.get_position(), right_mid_motor.get_position(), right_back_motor.get_position());
    pros::lcd::print(4, "N1=%d, N2=%d, N3=%d", num1, num2, num3);
    pros::lcd::print(6, "Sum+:%.1f, Sum-:%.1f", sum_x_Encoder_positive, sum_x_Encoder_nagetive);
    pros::lcd::print(7, "X: %.1f, Y: %.1f, Q: %.1f", sys_coordinates.x, sys_coordinates.y,
                     get_robot_heading_lib(hardwareParameter));
    pros::delay(50);
  }
}

void odometry_fn(void* param){
  double pre_X_Encoder = 0;
	double cur_X_Encoder = 0;
  double pre_Y_Encoder = 0;
	double cur_Y_Encoder = 0;

  double cur_Robot_Heading = get_robot_heading_lib(hardwareParameter);
  double pre_Robot_Heading = cur_Robot_Heading;

  double pre_left_wheel_encoder = 0;
  double pre_right_wheel_encoder = 0;
  double cur_left_wheel_encoder = 0;
  double cur_right_wheel_encoder = 0;
  double left_wheel_encoder_diff = 0;
  double right_wheel_encoder_diff = 0;

	double moveDistance_X;
  double moveDistance_Y;
  double moveDistance;

	long pre_Time = pros::millis();
	long cur_Time = pre_Time;
	long timeIntervalMili;

  double average_Robot_Heading;
  double XY_encoder_moving_angle;
  double angleDegree;
  double angleRadian;
  double motionAngleRadian;

  double pre_cur_X_diff;
  double pre_cur_Y_diff;

	double xx; // single vertical encoder
	double yy;
  int entry = 0;
  double percent, tmp;
//  bool firstTime = true;
	while(true) {
		cur_Time = pros::millis();
		timeIntervalMili = cur_Time - pre_Time;
    // get Gyro Heading Adjusted according to initial heading;
		cur_Robot_Heading = get_robot_heading_lib(hardwareParameter);

//    cur_X_Encoder = hardwareParameter.X_encoderLib.get_value();  //need check sign
//    cur_Y_Encoder = (hardwareParameter.Y_left_encoderLib.get_value() + hardwareParameter.Y_right_encoderLib.get_value())/2;
    pre_cur_X_diff = cur_X_Encoder - pre_X_Encoder;
    pre_cur_Y_diff = cur_Y_Encoder - pre_Y_Encoder;

    if(pre_cur_X_diff > 0){
      sum_x_Encoder_positive = sum_x_Encoder_positive + pre_cur_X_diff;
    } else {
      sum_x_Encoder_nagetive = sum_x_Encoder_nagetive + pre_cur_X_diff;
    }

    cur_left_wheel_encoder = (left_front_motor.get_position() + left_mid_motor.get_position()
                              + left_back_motor.get_position())/3.0;
    cur_right_wheel_encoder = (right_front_motor.get_position() + right_mid_motor.get_position()
                              + right_back_motor.get_position())/3.0;;
    left_wheel_encoder_diff = cur_left_wheel_encoder - pre_left_wheel_encoder;
    right_wheel_encoder_diff = cur_right_wheel_encoder - pre_right_wheel_encoder;

    if((pre_cur_Y_diff != 0)) {
      tmp = fabs(fabs(left_wheel_encoder_diff) - fabs(right_wheel_encoder_diff));
      if(fabs(left_wheel_encoder_diff) > fabs(right_wheel_encoder_diff)){
        percent = tmp / fabs(left_wheel_encoder_diff);
      } else {
        percent = tmp / fabs(right_wheel_encoder_diff);
      }
     if(fabs(left_wheel_encoder_diff) > fabs(right_wheel_encoder_diff)) {
        pre_cur_Y_diff = pre_cur_Y_diff * percent * 0.95 + pre_cur_Y_diff * (1-percent);  // 19/20
        num1++;
      } else if(fabs(left_wheel_encoder_diff) < fabs(right_wheel_encoder_diff) / 1.2) {
        pre_cur_Y_diff = pre_cur_Y_diff * percent * 1.055 + pre_cur_Y_diff * (1-percent);  // 19/20
        num2++;
      } else {
        num3++;
        //does nothing
      }
    }


		if(  (pre_cur_X_diff != 0) || (pre_cur_Y_diff != 0)) {
      if(pre_cur_X_diff == 0){
        if(pre_cur_Y_diff >= 0){
          XY_encoder_moving_angle = 90;
          entry = 90;
        } else {
          XY_encoder_moving_angle = 270;
          entry = 270;
        }
      } else {
        XY_encoder_moving_angle = radiansToDegrees(atan(pre_cur_Y_diff/pre_cur_X_diff));
        if(pre_cur_X_diff > 0 && pre_cur_Y_diff >= 0){ //in the first quadrant
          entry = 1;
          XY_encoder_moving_angle = XY_encoder_moving_angle;
        } else if(pre_cur_X_diff < 0 && pre_cur_Y_diff >= 0){ //in the second quadrant
          entry = 2;
          XY_encoder_moving_angle = XY_encoder_moving_angle + 180;
        } else if(pre_cur_X_diff < 0 && pre_cur_Y_diff < 0){ //in the third quadrant
          entry = 3;
          XY_encoder_moving_angle = XY_encoder_moving_angle + 180;
        } else if(pre_cur_X_diff > 0 && pre_cur_Y_diff < 0){ //in the forth quadrant
          entry = 4;
          XY_encoder_moving_angle = XY_encoder_moving_angle + 360;
        } else {
          entry = 0;
        }
      }

      average_Robot_Heading = (pre_Robot_Heading + cur_Robot_Heading)/2.0;
      angleDegree = average_Robot_Heading;  // + XY_encoder_moving_angle - 90;
      moveDistance_X = degreeToDistanceCM(pre_cur_X_diff, ENCODER_CIRCUMFERENCE_CM);
  		moveDistance_Y = degreeToDistanceCM(pre_cur_Y_diff, ENCODER_CIRCUMFERENCE_CM + 0.5);
//      moveDistance_Y = degreeToDistanceCM((left_wheel_encoder_diff +
//                 right_wheel_encoder_diff)/2.0 * 1.3333, WHEEL_CIRCUMFERENCE_CM);

      angleRadian = degreesToRadians(angleDegree);

      if((left_wheel_encoder_diff > 0 && right_wheel_encoder_diff > 0)
        || (left_wheel_encoder_diff < 0 && right_wheel_encoder_diff < 0)) {
        xx = moveDistance_Y * std::cos(angleRadian);
  			yy = moveDistance_Y * std::sin(angleRadian);

        motionAngleRadian = degreesToRadians(motion_initial_angle);

        //	yy = yy + moveDistance_X * std::sin(angleRadian - 1.57079);
        moveDistance_X = moveDistance_X * std::sin(angleRadian - 1.57079 - (motionAngleRadian - 1.57079));

        xx = xx + moveDistance_X * std::cos(motionAngleRadian);
	  		yy = yy + moveDistance_X * std::sin(motionAngleRadian);
        update_coordinate(COODINATE_UPDATE_INCREMENT, xx, yy);
      }

/*      moveDistance_X = degreeToDistanceCM(pre_cur_X_diff, ENCODER_CIRCUMFERENCE_CM);
  		moveDistance_Y = degreeToDistanceCM(pre_cur_Y_diff, ENCODER_CIRCUMFERENCE_CM);
      moveDistance = sqrt(pow(moveDistance_X, 2) + pow(moveDistance_Y, 2));

      xx = moveDistance * std::cos(degreesToRadians(angle));
			yy = moveDistance * std::sin(degreesToRadians(angle));
      update_coordinate(COODINATE_UPDATE_INCREMENT, xx, yy);
*/
      pre_X_Encoder = cur_X_Encoder;
      pre_Y_Encoder = cur_Y_Encoder;
      pre_Robot_Heading = cur_Robot_Heading;
      pre_left_wheel_encoder = cur_left_wheel_encoder;
      pre_right_wheel_encoder = cur_right_wheel_encoder;
		}
    pros::delay(2);
	} // end of while loop
}


/********************************************************************
* before excution of turnDegreesPID_lib(), need to fill the struct: turnDegreesPID_Parameters
********************************************************************/
void background_execution_turnDegreesPID_lib(void* param){
  turnDegreesPID_lib(turnDegreesPID_Parameter.targetHeadingLib, turnDegreesPID_Parameter.turnType,
                     turnDegreesPID_Parameter.maxPower, turnDegreesPID_Parameter.turnDirection,
                     turnDegreesPID_Parameter.headingKP, turnDegreesPID_Parameter.headingKI,
                     turnDegreesPID_Parameter.headingKD, turnDegreesPID_Parameter.timeoutMili,
                     turnDegreesPID_Parameter.exitConditionExpectedPasses, hardwareParameter);
}
