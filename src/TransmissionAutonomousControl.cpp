#include "main.h"
#include "TransmissionConfiguration.hpp"

#define PRE 2
#define CATCH 1
#define RELEASE 0

#define ON 1
#define OFF -1

// go straight parameters

//	goStraightCmPID_lib(122, 90, 127, MOVE_FORWARD, 5, 0, 3, 0.35, 0, 26, 6000, 3, hardwareParameter);
//	goStraightCmPID_lib(122, 90, 127, MOVE_FORWARD, 5, 0, 3, 0.35, 0, 24, 6000, 2, hardwareParameter);
//	goStraightCmPID_lib(122, 90, 127, MOVE_FORWARD, 5, 0, 3, 0.3, 0, 28, 6000, 1, hardwareParameter);

// goStraightCmPID_lib(60.0, 90, 127, MOVE_BACKWARD, 5, 0, 3, 0.40, 0, 1, 2000, 2, hardwareParameter);

// left is positive, right is negative

using namespace pros;

Point curPos;
double distance;
double angle;

/***********************************************************************************************

AUTONOMOUS PROGRAMS

***********************************************************************************************/

/************************************************************************
// ENODEV; 19 - The port cannot be configured as a vision sensor
// EINVAL; 22 - sig_id is outside the range [1-7]
// EDOM;   33 - size_id is greater than the number of available objects.
// EAGAIN; 11 - Reading the vision sensor failed for an unknown reason.
**************************************************************************/

detected_vision_goal_lib get_goal_object_front_vision(int goal_color_signature)
{
	detected_vision_goal_lib final_goal = {goal_color_signature, 0, 0, 0, 0, 0, 0};
	int e_number1;
	int e_number2;
	vision_object_s_t left_goal;
	vision_object_s_t right_goal;

	errno = 6;
	vision_object_s_t largest_goal = front_vision.get_by_sig(0, goal_color_signature);
	std::cout << "   mid = " << largest_goal.x_middle_coord << "...";
	e_number1 = errno;
	errno = 6;
	vision_object_s_t second_goal = front_vision.get_by_sig(1, goal_color_signature);
	e_number2 = errno;

	if (e_number1 == 33)
	{
		final_goal.width = -1;
		final_goal.height = -1;
		std::cout << "   111   ";
		return final_goal;
	}

	final_goal.left_coord = largest_goal.left_coord;
	final_goal.top_coord = largest_goal.top_coord;
	final_goal.width = largest_goal.width;
	final_goal.height = largest_goal.height;
	final_goal.x_middle_coord = largest_goal.x_middle_coord;
	final_goal.y_middle_coord = largest_goal.y_middle_coord;
	if (e_number2 == 33)
	{ // only one object detected
		std::cout << "   222   ";
		return final_goal;
	}
	else
	{ // two object detected
		if (largest_goal.width < 40)
		{ // object too small to combine
			std::cout << "   333   ";
			return final_goal;
		}
		else
		{ // object big/close enough to combine
			if ((abs(largest_goal.top_coord - second_goal.top_coord) <= 5) && (abs(largest_goal.height - second_goal.height) <= 8))
			{ // two goals with similar height
				if (largest_goal.left_coord < second_goal.left_coord)
				{
					left_goal = largest_goal;
					right_goal = second_goal;
				}
				else
				{
					left_goal = second_goal;
					right_goal = largest_goal;
				}
				if (left_goal.left_coord + left_goal.width < right_goal.left_coord)
				{ // two goals not overlapped
					final_goal.left_coord = left_goal.left_coord;
					final_goal.top_coord = left_goal.top_coord;
					final_goal.width = right_goal.left_coord - left_goal.left_coord + right_goal.width;
					final_goal.height = left_goal.height;
					final_goal.x_middle_coord = final_goal.left_coord + final_goal.width / 2;
					final_goal.y_middle_coord = final_goal.top_coord + final_goal.height / 2;
					std::cout << "   444   ";
					return final_goal;
				}
				else
				{ // two goals overlapped
					std::cout << "   555   ";
					return final_goal;
				}
			}
			else
			{ // two goals are in different height
				std::cout << "   666   ";

				return final_goal;
			}
		}
	}

	std::cout << "   777   ";
	return final_goal;
}

void arm_moving_holding_fn(void *param)
{
	long start_time = pros::millis();

	arm_action_lib = ARM_HOLDING_POSITION;
	arm_motor.move(0);
	delay(200);
	while (true)
	{
		if (fabs(arm_motor.get_actual_velocity()) < 3)
		{
			break;
		}
		if (pros::millis() - start_time > 500)
		{
			break;
		}
		delay(10);
	}
	arm_motor.tare_position();
	arm_motor.move_absolute(10, 127);
	delay(50);
	arm_motor.tare_position();

	int pass_target_count = 0;
	double pre_arm_target_angle = arm_move_target_angle_lib;
	double cur_arm_degree = arm_motor.get_position();
	double pre_arm_degree = cur_arm_degree;
	double Kp = 3, Ki = 0.005, Kd = 0;
	double error_kp = 0;
	double pre_error_kp = 0;
	double error_sum = 0;
	double error_change_rate = 0;
	double total_correction = 0;

	while (true)
	{
		if (pre_arm_target_angle != arm_move_target_angle_lib)
		{
			pre_arm_target_angle = arm_move_target_angle_lib;
			error_sum = 0;
			error_kp = arm_move_target_angle_lib - cur_arm_degree;
			pre_error_kp = error_kp;
			pass_target_count = 0;
		}
		cur_arm_degree = arm_motor.get_position();
		if (pass_target_count == 0)
		{
			if (cur_arm_degree == arm_move_target_angle_lib || (cur_arm_degree > arm_move_target_angle_lib && pre_arm_degree < arm_move_target_angle_lib) || (cur_arm_degree < arm_move_target_angle_lib && pre_arm_degree > arm_move_target_angle_lib))
			{
				pass_target_count++;
				//				pros::lcd::print(1, "AAAA=%.1f", arm_move_target_angle_lib);
			}
			else
			{
				if (cur_arm_degree < arm_move_target_angle_lib)
				{
					arm_motor.move(abs(arm_move_speed_lib));
				}
				else if (cur_arm_degree > arm_move_target_angle_lib)
				{
					arm_motor.move(0 - abs(arm_move_speed_lib));
				}
			}
		}
		else
		{
			error_kp = arm_move_target_angle_lib - cur_arm_degree;
			error_sum = error_sum + error_kp;
			error_change_rate = error_kp - pre_error_kp;
			total_correction = error_kp * Kp + error_sum * Ki + error_change_rate * Kd;
			arm_motor.move((int)total_correction);
			pre_error_kp = error_kp;
		}
		pre_arm_degree = cur_arm_degree;
		//		pros::lcd::print(3, "arm=%.1f", arm_motor.get_position());
		delay(5);
	}
}

void auton_60s_skills_bridge_version()
{
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 180;
	long hook_action_delay_time = 50;
	long claw_action_delay_time = 120;
	long start_time = pros::millis();
	int intake_speed = 110;
	arm_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	//	pros::Task arm_holding_task(arm_moving_holding_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "arm_holding_task");

	// sys_display_info_terminal = DEBUG_DISPLAY_MASSAGE_ON;
	// top_piston.set_value(false);
	// sys_display_info_terminal = DEBUG_DISPLAY_MASSAGE_ON;
	// goStraightCm_Front_Vision(55, 90, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
	// 			              						0.5, 0, 1, 0.3, 0, 10, 0.8, 0, 0, 1000, 1, hardwareParameter);
	// front_piston.set_value(true);
	// pros::lcd::print(3, "bbbbbbbbbbbbbbbbb");
	// waitForTouch();
	// pros::lcd::print(3, "aaaaaaaaaaaaaaaaa");
	// while(true){
	// 	delay(50);
	// }

	// int num1 = 0;
	// int num2 = 0;
	// while(true){
	// 	if(is_limit_switch_pressed() == true){
	// 		num1++;
	// 		pros::lcd::print(1, "pressed = %d", num1);
	// 	} else {
	// 	  num2++;
	// 		pros::lcd::print(2, "not pressed%d", num2);
	//   }
	// 	delay(10);
	// }

	// //	bridge code
	//    sys_initial_robot_heading = 135;
	//
	// clawAction_1 = {0, true, 1};
	// hookAction_1 = {0, true, 1};
	// delay(100);
	// turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	// waitForTouch();
	// 	clawAction_1 = {0, true, 1};
	// 	delay(claw_action_delay_time);
	// 	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	// 	delay(300);
	// 	goStraightCmPID_lib(20, 185, 127, MOVE_FORWARD, 8, 0, 0, 0.2, 0, 5, 800, 1, hardwareParameter);
	// 	delay(100);
	// 	goStraightCmPID_lib(35, 183, 127, MOVE_FORWARD, 1.2, 0, 0, 0.5, 0, 5, 1200, 1, hardwareParameter);
	// 	delay(100);
	// //	turnDegreesPID_lib(185, ON_SPOT_TURN, 70, COUNTER_CLOCKWISE, 6, 0, -20, 900, 2, hardwareParameter);
	// //	goStraightCmPID_lib(30, 185, 127, MOVE_FORWARD, 3, 0, 0, 1, 0, 5, 1000, 1, hardwareParameter);
	// //waitForTouch();
	// 	armAction_1 = {127, 0, 20, 1};
	// 	delay(800);
	// 	left_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// 	left_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// 	left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// 	right_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// 	right_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// 	right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	//
	// 	goStraightCmPID_lib(85, 180, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 2000, 1, hardwareParameter);
	// 	balance_bridge_PID_lib(127, -15, 15, 0, 0, 5000, 1, hardwareParameter);
	// 	goStraightCmPID_lib(3.5, 180, 127, MOVE_BACKWARD, 0, 0, 0, 1, 0, 0, 800, 1, hardwareParameter);
	//
	// 	pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
	// 	waitForTouch();

	// 	hookAction_1 = {0, true, 1};
	// 	clawAction_1 = {0, true, 1};
	// 	delay(claw_action_delay_time);
	// 		left_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// 		left_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// 		left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// 		right_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// 		right_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD) ;
	// 		right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	// 		goStraightCmPID_lib(85, 180, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 3000, 1, hardwareParameter);
	// 		balance_bridge_PID_lib(127, -18, 15, 0, 0, 5000, 1, hardwareParameter);
	// 		goStraightCmPID_lib(1.5, 180, 127, MOVE_BACKWARD, 0, 0, 0, 3, 0, 0, 800, 1, hardwareParameter);

	// pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
	// std::cout << "Time=%d" << pros::millis() - start_time << std::endl;
	// waitForTouch();

	clawAction_1 = {0, false, 1};
	hookAction_1 = {0, false, 1};
	transmission_piston.set_value(false);
	top_piston.set_value(false);
	delay(300);

	// ------------------------------------------------------------------------------------ //
	//// part 1 take first blue goal
	goStraightCmPID_lib(10, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	//	goStraightCmPID_lib(15.5, 180, 80, MOVE_FORWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	delay(hook_action_delay_time);
	//	intakeAction_1 = {100, 500, 0, 100, 1};
	goStraightCmPID_lib(65, 82, 127, MOVE_FORWARD, 1.5, 0, 2.5, 1, 0, 0, 1500, 1, hardwareParameter); // 100,80,127
	goStraightCm_Front_Vision_limit_switch(65, 80, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
							  0.4, 0, 1.5, 0.3, 0, 10, 0.8, 0, 0, 1500, 1, hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	//	goStraightCmPID_lib_limit_switch(20, angle, 60, MOVE_FORWARD, 1.5, 0, 1.5, 0.8, 0, 0, 600, 1, hardwareParameter);

	///////catch first yellow goal
	clawAction_1 = {0, true, 1};
	delay(200);
	armAction_1 = {127, 200, MAX_TOP, 1};


	intakeAction_1 = {intake_speed, 200, 0, intake_speed, 1};
	delay(500);

	// 	pros::lcd::print(2, "Time=%d", pros::millis() - start_time);

	///////carry first yellow goal to bridge
	angle = 64;
	goStraightCmPID_lib(159, angle, 127, MOVE_FORWARD, 3.5, 0, 2.5, 0.2, 0, 5, 1650, 1, hardwareParameter); // 1650
	intakeAction_1 = {0, 0, 0, 0, 1};
	delay(100);

	//	goStraightCmPID_lib(15, angle, 127, MOVE_FORWARD, 2.5, 0, 1.5, 0.5, 0, 10, 400, 1, hardwareParameter); //100,80,127
	///////place the first yellow goal to the bridge
	armAction_1 = {127, 0, PRESS_BRIDGE, 1};
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time + 50);

	angle = 70;
	goStraightCmPID_lib(75, 50, 127, MOVE_BACKWARD, 5, 0, 2.5, 0.5, 0, 4, 1200, 1, hardwareParameter);

	waitForTouch();

	////take first blue goal by front claw
	hookAction_1 = {0, false, 1};
	armAction_1 = {127, 0, 0, 1};
	delay(hook_action_delay_time);
	goStraightCmPID_lib(27, 50, 100, MOVE_FORWARD, 5, 0, 1, 0.5, 0, 10, 700, 1, hardwareParameter);
	turnDegreesPID_lib(230, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
	//		goStraightCmPID_lib_limit_switch(27, 230, 60, MOVE_FORWARD, 3.5, 0, 1, 0.3, 0, 0, 700, 1, hardwareParameter);
	// delay(100);
	clawAction_1 = {0, true, 1};
	armAction_1 = {127, 0, 350, 1};
	delay(100);

	goStraightCmPID_lib(65, 5, 127, MOVE_BACKWARD, 1.0, 0, 1.2, 0.7, 0, 10, 1000, 1, hardwareParameter); // 1500
	double distance = get_distance_back_vision(back_vision, DETECT_BLUE_GOAL_SIG, 10, 10, 60, 120);
	if (distance > 70 || distance < 10)
	{
		distance = 20;
	}
	goStraightCm_Back_Vision(distance + 20, 350, 65, DETECT_BLUE_GOAL_SIG, back_vision,
							 0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 5, 800, 1, hardwareParameter);

	////take first red goal by back claw
	hookAction_1 = {0, true, 1};
	armAction_1 = {127, 5, PRESS_BRIDGE + 300, 1};
	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};
	////catch first five flower ring
	goStraightCmPID_lib(112, 340, 127, MOVE_FORWARD, 3.5, 0, 2.5, 0.3, 0, 5, 1500, 1, hardwareParameter);

	goStraightCmPID_lib(20, 10, 70, MOVE_FORWARD, 2.5, 0, 0, 0.8, 0, 2, 800, 1, hardwareParameter); // 1300
	goStraightCmPID_lib(25, 60, 80, MOVE_FORWARD, 2.5, 0, 0, 0.8, 0, 2, 800, 1, hardwareParameter); // 1300
	goStraightCmPID_lib(43, 80, 80, MOVE_FORWARD, 2.5, 0, 2, 1, 0, 5, 1100, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE - 35, 1};
	delay(100);
	goStraightCmPID_lib(15, 80, 100, MOVE_FORWARD, 2.5, 0, 2, 1, 0, 5, 500, 1, hardwareParameter);
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(5, 95, 127, MOVE_BACKWARD, 3.5, 0, 2.5, 10, 0, 5, 500, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 200, 1};
	delay(100);
	goStraightCmPID_lib(12, 180, 100, MOVE_BACKWARD, 1.5, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);

	armAction_1 = {127, 200, 0, 1};
	turnDegreesPID_lib(260, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};

	////take middle high yellow goal
	delay(100); //
	new_goStraightCm_Front_Vision(20, 230, 70, DETECT_YELLOW_GOAL_SIG, front_vision,
								  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 1, 800, 1, hardwareParameter);
	//	intakeAction_1 = {-10, 0, 0, -10, 1};
	angle = get_robot_heading_lib(hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	//		  goStraightCmPID_lib(32, angle, 50, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 1000, 1, hardwareParameter);
	//		  goStraightCmPID_lib_limit_switch(32, angle, 60, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 1000, 1, hardwareParameter);
	// delay(150);
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	//			armAction_1 = {127, 0, PRESS_BRIDGE + 170, 1};
	armAction_1 = {127, 0, 70, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, -20, 1600, 2, hardwareParameter);
	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};
	armAction_1 = {127, 0, PRESS_BRIDGE + 190, 1};
	delay(350);
	angle = 95;
	goStraightCmPID_lib(90, angle, 85, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 1500, 1, hardwareParameter);
	// move high yellow goal to red bridge
	armAction_1 = {127, 0, PRESS_BRIDGE - 30, 1};
	delay(300);
	goStraightCmPID_lib(15, angle, 80, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 700, 1, hardwareParameter);
	//			armAction_1 = {127, 0, PRESS_BRIDGE - 20, 1};
	armAction_1 = {127, 0, PRESS_BRIDGE - 30, 1};
	delay(500);
	//			turnDegreesPID_lib(80, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 700, 2, hardwareParameter);
	////place high yellow mobile goal
	clawAction_1 = {0, false, 1};
	armAction_1 = {127, 0, PRESS_BRIDGE - 40, 1};
	delay(claw_action_delay_time + 100);
	goStraightCmPID_lib(5, angle, 127, MOVE_BACKWARD, 3.5, 0, 2.5, 4, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	delay(200);

	intakeAction_1 = {0, 0, 0, 0, 1};
	goStraightCmPID_lib(60, 90, 127, MOVE_BACKWARD, 3, 0, 1, 0.5, 0, 2, 1200, 1, hardwareParameter);
	goStraightCmPID_lib(100, 135, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 2, 1300, 1, hardwareParameter);
	goStraightCmPID_lib(110, 145, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 2, 1400, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	goStraightCmPID_lib(25, 145, 80, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 600, 1, hardwareParameter);

	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};
	goStraightCmPID_lib(68, 130, 127, MOVE_FORWARD, 1.5, 0, 1, 0.5, 0, 7, 1100, 1, hardwareParameter);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 700, 2, hardwareParameter);

	intakeAction_1 = {0, 0, 0, 0, 1};
	distance = get_distance_back_vision(back_vision, DETECT_RED_GOAL_SIG, 10, 10, 60, 120);
	if (distance > 70 || distance < 10)
	{
		distance = 50;
	}
	goStraightCm_Back_Vision(distance + 20, 180, 70, DETECT_RED_GOAL_SIG, back_vision,
							 0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 1, 1000, 1, hardwareParameter);
	// take corner blue goal
	hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);
	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};

	armAction_1 = {127, 0, 0, 1};
	goStraightCmPID_lib(50, 180, 90, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 5, 800, 1, hardwareParameter); // 900
	turnDegreesPID_lib(90, ON_SPOT_TURN, 80, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);	   // 900

	goStraightCm_Front_Vision(30, 90, 90, DETECT_YELLOW_GOAL_SIG, front_vision,
							  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 800, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	//      goStraightCmPID_lib_limit_switch(30, angle, 70, MOVE_FORWARD, 1.5, 0, 1.5, 0.8, 0, 0, 800, 1, hardwareParameter);
	// take last yellow goal
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, PRESS_BRIDGE + 200, 1};
	goStraightCmPID_lib(36, 120, 80, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 3, 800, 1, hardwareParameter);	  // 2000
	goStraightCmPID_lib(115, 130, 100, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 3, 1500, 1, hardwareParameter); // 2000

	armAction_1 = {127, 0, PRESS_BRIDGE, 1};
	goStraightCmPID_lib(15, 110, 127, MOVE_FORWARD, 3.5, 0, 1, 0.4, 0, 0, 500, 1, hardwareParameter);
	// drop last yellow goal
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(5, 125, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	hookAction_1 = {200, false, 1};
	goStraightCmPID_lib(55, 120, 100, MOVE_BACKWARD, 3, 0, 1, 0.35, 0, 5, 1000, 1, hardwareParameter);

	// hookAction_1 = {0, false, 1};
	armAction_1 = {127, 0, 0, 1};
	intakeAction_1 = {0, 0, 0, 0, 1};
	// delay(hook_action_delay_time);
	goStraightCmPID_lib(30, 120, 80, MOVE_FORWARD, 5, 0, 1, 0.5, 0, 10, 800, 2, hardwareParameter);
	turnDegreesPID_lib(300, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
	// goStraightCmPID_lib_limit_switch(45, 300, 60, MOVE_FORWARD,	3, 0, 1, 0.3, 0, 0, 1300, 1, hardwareParameter);
	// get last red goal
	// delay(100);
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, PRESS_BRIDGE + 190, 1};
	turnDegreesPID_lib(110, ON_SPOT_TURN, 70, COUNTER_CLOCKWISE, 6, 0, -20, 1300, 2, hardwareParameter);
	goStraightCmPID_lib(77, 110, 100, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 2, 1400, 1, hardwareParameter);

	// release last blue goal to the bridge
	// armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	// delay(100);
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);

	armAction_1 = {127, 500, 0, 1};
	goStraightCmPID_lib(70, 180, 127, MOVE_BACKWARD, 3.5, 0, 1, 1, 0, 2, 1400, 1, hardwareParameter);
	turnDegreesPID_lib(120, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);

	distance = get_distance_front_vision(front_vision, DETECT_BLUE_GOAL_SIG, 10, 10, 60, 120);
	if (distance > 70 || distance < 10)
	{
		distance = 50;
	}
	new_goStraightCm_Front_Vision(15, 120, 70, DETECT_BLUE_GOAL_SIG, front_vision,
								  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 1, 1000, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	armAction_1 = {127, 0, 0, 1};
	//	goStraightCmPID_lib_limit_switch(distance - 10, angle, 50, MOVE_FORWARD,	1, 0, 1, 0.3, 0, 1, 1200, 1, hardwareParameter);
	// catch the goal beside the bridge
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, 60, 1};
	goStraightCmPID_lib(200, 83, 127, MOVE_BACKWARD, 2, 0, 1, 0.3, 0, 5, 2500, 1, hardwareParameter);
	goStraightCmPID_lib(40, 135, 127, MOVE_FORWARD, 1.5, 0, 1, 0.3, 0, 5, 1000, 1, hardwareParameter);
	goStraightCmPID_lib(90, 135, 127, MOVE_BACKWARD, 2, 0, 1, 0.3, 0, 5, 1500, 1, hardwareParameter);

	// turnDegreesPID_lib(30, ON_SPOT_TURN, 80, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);
	// goStraightCmPID_lib(40, 60, 127, MOVE_FORWARD,	1, 0, 1, 0.3, 0, 1, 900, 1, hardwareParameter);
	// turnDegreesPID_lib(135, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	// goStraightCmPID_lib(40, 135, 100, MOVE_BACKWARD,	1, 0, 1, 0.3, 0, 5, 1000, 1, hardwareParameter);
	delay(100);
	distance = get_distance_back_vision(back_vision, DETECT_BLUE_GOAL_SIG, 10, 10, 60, 110);
	armAction_1 = {127, 0, PRESS_BRIDGE + 300, 1};
	goStraightCm_Back_Vision(distance + 8, 135, 100, DETECT_BLUE_GOAL_SIG, back_vision,
							 0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 5, 1200, 1, hardwareParameter);
	delay(100);
	hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);

	// //	bridge code

	//		goStraightCmPID_lib(40, 195, 127, MOVE_FORWARD, 8, 0, 0, 0.3, 0, 5, 1200, 1, hardwareParameter);
	goStraightCmPID_lib(5, 135, 127, MOVE_FORWARD, 2, 0, 0, 1, 0, 0, 500, 1, hardwareParameter);
	turnDegreesPID_lib(215, ON_SPOT_TURN, 60, COUNTER_CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);
	//		delay(100);
	goStraightCmPID_lib(45, 215, 127, MOVE_FORWARD, 1.2, 0, 0, 0.5, 0, 5, 1200, 1, hardwareParameter);
	//		goStraightCmPID_lib(35, 180, 127, MOVE_FORWARD, 1.2, 0, 0, 0.5, 0, 5, 1200, 1, hardwareParameter);
	//		delay(100);
	armAction_1 = {127, 200, 20, 1};
	turnDegreesPID_lib(180, ON_SPOT_TURN, 60, CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);

	//		armAction_1 = {127, 0, 20, 1};
	delay(200);
	left_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	left_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	goStraightCmPID_lib(85, 180, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 3000, 1, hardwareParameter);
	balance_bridge_PID_lib(127, -18, 15, 0, 0, 5000, 1, hardwareParameter);
	goStraightCmPID_lib(1.5, 180, 127, MOVE_BACKWARD, 0, 0, 0, 3, 0, 0, 800, 1, hardwareParameter);

	// goStraightCmPID_lib(85, 180, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 3000, 1, hardwareParameter);
	// balance_bridge_PID_lib(127, -10, 15, 0, 0, 5000, 1, hardwareParameter);
	// goStraightCmPID_lib(3.5, 180, 127, MOVE_BACKWARD, 0, 0, 0, 1, 0, 0, 800, 1, hardwareParameter);

	pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
	std::cout << "Time=%d" << pros::millis() - start_time << std::endl;
	waitForTouch();

	////////////////////////////////////////
	////////////////////////////////////////
	////////////////////////////////////////
	//	armAction_1 = {127, 0, 100, 1};
	///////////////////////////////////////?????
	//	goStraightCmPID_lib(15, 30, 100, MOVE_BACKWARD,	5, 0, 1, 0.3, 0, 0, 600, 1, hardwareParameter);

	////////////////////////////////////////
	////////////////////////////////////////
	////////////////////////////////////////

	// bridge code
	//  clawAction_1 = {0, true, 1};
	//  delay(claw_action_delay_time);
	//  armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	//  goStraightCmPID_lib(5.5, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	//  hookAction_1 = {0, true, 1};
	//  delay(hook_action_delay_time);
	//  delay(1000);
	//  goStraightCmPID_lib(35, 180, 127, MOVE_FORWARD, 3, 0, 0, 1, 0, 5, 1000, 1, hardwareParameter);
	//  armAction_1 = {127, 0, 20, 1};
	//  delay(1000);
	//  //waitForTouch();
	//  left_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	//  left_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	//  left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	//  right_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	//  right_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	//  right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	//
	//  goStraightCmPID_lib(85, 180, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 2000, 1, hardwareParameter);
	//  balance_bridge_PID_lib(127, -15, 15, 0, 0, 5000, 1, hardwareParameter);
	//  goStraightCmPID_lib(3, 180, 127, MOVE_BACKWARD, 0, 0, 0, 1, 0, 0, 800, 1, hardwareParameter);
	//

	pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
	waitForTouch();
}

double get_distance_front_vision(Vision vision_sensor, int goal_color_signature,
								 long samplingNumber, long sensingTimeInterval_Millis,
								 int min_width, long timeoutMillis)
{
	double distance = 0;
	double width1;
	vision_object_s_t closest_goal;
	double sum = 0;
	long i = 0;
	long startTime = pros::millis();

	while (i < samplingNumber)
	{
		if (pros::millis() - startTime >= timeoutMillis)
		{
			break;
		}
		closest_goal = vision_sensor.get_by_sig(0, goal_color_signature);
		width1 = (double)(closest_goal.width);
		if (width1 >= min_width)
		{
			sum = sum + width1;
			i++;
		}
		delay(sensingTimeInterval_Millis);
	}
	if (i == 0)
	{
		return -1;
	}

	width1 = sum / ((double)i);
	distance = 224.17 * powf(2.71828, -0.012 * width1);
	//  distance = -2 * 0.00005 * width1 * width1 * width1 + 0.0125 * width1 * width1
	//	           - 2.7984 * width1 + 242.41;
	return distance;
}

double get_distance_back_vision(Vision vision_sensor, int goal_color_signature,
								long samplingNumber, long sensingTimeInterval_Millis,
								int min_width, long timeoutMillis)
{
	double distance = 0;
	double width1;
	vision_object_s_t closest_goal;
	double sum = 0;
	long i = 0;
	long startTime = pros::millis();

	// 	for(i = 0; i < samplingNumber;){
	// 			closest_goal= vision_sensor.get_by_sig(0, goal_color_signature);
	// 			if(closest_goal.width < 60){
	// 				sum = sum + (double)(closest_goal.width);
	// 				i++;
	// 			}
	// 			pros::lcd::print(0, "Width=%d,  M=%d", closest_goal.width, closest_goal.x_middle_coord);
	// 			delay(sensingTimeInterval_Millis);
	// 		}
	// 	pros::lcd::print(1, "avg=%.1f", sum/((double)i));
	// waitForTouch();

	while (i < samplingNumber)
	{
		if (pros::millis() - startTime >= timeoutMillis)
		{
			break;
		}
		closest_goal = vision_sensor.get_by_sig(0, goal_color_signature);
		width1 = (double)(closest_goal.width);
		if (width1 >= min_width)
		{
			sum = sum + width1;
			i++;
		}
		delay(sensingTimeInterval_Millis);
	}
	if (i == 0)
	{
		return -1;
	}

	width1 = sum / ((double)i);
	//	pros::lcd::print(1, "Width=%.1f", width1);
	//	distance = 0 - (0.00002*width1*width1*width1) + (0.014*width1*width1) - (2.9598*width1) + 242.31;
	distance = 217.84 * powf(2.71828, -0.012 * width1);

	return distance;
}

void test()
{
	arm_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	while (true)
	{
		pros::lcd::print(5, "arm: %f", arm_motor.get_position());
		delay(10);
	}

	//	goStraightCmPID_lib(50, 90, 127, MOVE_FORWARD, 0, 0, 0, 1, 0, 10, 1000, 1, hardwareParameter);
	//	turnDegreesPID_lib(180, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, 20, 1000, 2, hardwareParameter);
	turnDegreesPID_lib(270, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, 25, 1000, 2, hardwareParameter);

	while (true)
	{
		//	 std::cout << Y_encoder.get_value() << std::endl;
		delay(50);
	}
}

void test_vision()
{

	long startTime = pros::millis();

	vision_object_s_t first_goal;
	vision_object_s_t second_goal;
	int e_number1 = errno;
	int e_number2 = errno;

	int exposure = 30;
	int flag1 = 0;
	int flag2 = 0;
	std::cout << "exposure = " << exposure << std::endl;
	front_vision.set_exposure(exposure);
	while (true)
	{
		first_goal.width = 0;
		first_goal.height = 0;
		first_goal.left_coord = 0;
		second_goal.width = 0;
		second_goal.height = 0;
		second_goal.left_coord = 0;
		first_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
		e_number1 = errno;
		errno = 0;
		second_goal = front_vision.get_by_sig(1, DETECT_YELLOW_GOAL_SIG);
		e_number2 = errno;
		errno = 0;
		// ENODEV; 19 - The port cannot be configured as a vision sensor
		// EINVAL; 22 - sig_id is outside the range [1-7]
		// EDOM; 33 - size_id is greater than the number of available objects.
		// EAGAIN; 11 - Reading the vision sensor failed for an unknown reason.

		//  if(pros::millis()-startTime > 5000 && flag1 == 0){
		// 		flag1++;
		// 		exposure = exposure + 30;
		// 		front_vision.set_exposure(exposure);
		// 	} else if(pros::millis()-startTime > 10000 && flag2 == 0){
		// 		flag2++;
		// 		exposure = exposure + 30;
		// 		front_vision.set_exposure(exposure);
		// 	}

		std::cout << pros::millis() - startTime << "    " << first_goal.width << "  " << first_goal.height << "  "
				  << first_goal.left_coord << "  " << e_number1;
		std::cout << "            " << second_goal.width << "  " << second_goal.height << "  "
				  << second_goal.left_coord << "  " << e_number2 << "        " << exposure << std::endl;

		// pros::lcd::print(1, "L=%d, W=%d", closest_goal.left_coord, closest_goal.width);
		delay(50);
	}

	// pros::vision_object_s_t closest_goal;
	// while(true){
	//   closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
	// 	 pros::lcd::print(0, "Left=%d, Wid=%d, mid=", closest_goal.left_coord, closest_goal.width, closest_goal.x_middle_coord);
	// 	 delay(100);
	// }

	// goStraightCm_Back_Vision(55, 0, 80, DETECT_RED_GOAL_SIG, back_vision,
	// 												 0.5, 0, 1, 0.3, 0, 5, 0.5, 0, 1, 1200, 1, hardwareParameter);

	// goStraightCm_Front_Vision(35, 0, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
	// 										0.5, 0, 0, 0.3, 0, 7, 1, 0, 1, 1000, 1, hardwareParameter);
	// waitForTouch();
	// sys_initial_robot_heading = 180;
	////turnDegreesPID_lib(0, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.2, 0, -1, 1200, 1, hardwareParameter);
	// waitForTouch();
}

void right_side()
{
	long start_time = pros::millis();
	int intake_speed = 120;
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 90;

	clawAction_1 = {0, false, 1};
	hookAction_1 = {0, false, 1};
	transmission_piston.set_value(false);
	top_piston.set_value(true);

	clawAction_1 = {1050, true, 1};
	goStraightCmPID_lib(100, 90, 127, MOVE_FORWARD, 1.8, 0, 2.5, 1, 0, 0, 1500, 1, hardwareParameter);
	goStraightCmPID_lib(70, 70, 127, MOVE_BACKWARD, 1.8, 0, 2.5, 1, 0, 0, 15000, 1, hardwareParameter);
	turnDegreesPID_lib(310, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, 0.1, 1200, 2, hardwareParameter);
	clawAction_1 = {0, false, 1};
	goStraightCmPID_lib(20, 310, 127, MOVE_FORWARD, 1.8, 0, 2.5, 1, 0, 0, 1500, 1, hardwareParameter);
	goStraightCmPID_lib(20, 310, 127, MOVE_BACKWARD, 1.8, 0, 2.5, 1, 0, 0, 1500, 1, hardwareParameter);
	top_piston.set_value(false);
	turnDegreesPID_lib(132, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, 0.1, 1200, 2, hardwareParameter);
	goStraightCmPID_lib(20, 132, 127, MOVE_FORWARD, 1.8, 0, 2.5, 1, 0, 0, 1500, 1, hardwareParameter);
	goStraightCm_Front_Vision(50, 135, 90, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.8, 0, 0, 1500, 1, hardwareParameter);
	double angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(30, angle, 70, MOVE_FORWARD, 1.8, 0, 2.5, 1, 0, 0, 1500, 1, hardwareParameter);

	clawAction_1 = {0, true, 1};
	goStraightCmPID_lib(40, 180, 127, MOVE_BACKWARD, 0.9, 0, 0.5, 1, 0, 0, 1500, 1, hardwareParameter);
	std::cout << "\n--------------------------------------------\n";
	waitForTouch();
	goStraightCm_Back_Vision(70, 180, 100, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 5, 1200, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(30, angle, 127, MOVE_BACKWARD, 0.9, 0, 0.5, 1, 0, 0, 1500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(30, 180, 127, MOVE_FORWARD, 0.9, 0, 0.5, 1, 0, 0, 1500, 1, hardwareParameter);
}

void left_side()
{
	long start_time = pros::millis();
	int intake_speed = 120;
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 85;

	clawAction_1 = {0, false, 1};
	hookAction_1 = {0, false, 1};
	transmission_piston.set_value(false);
	top_piston.set_value(true);

	clawAction_1 = {1150, true, 1};
	goStraightCmPID_lib(105, 85, 127, MOVE_FORWARD, 1.8, 0, 2.5, 1, 0, 0, 1500, 1, hardwareParameter);
	goStraightCmPID_lib(90, 75, 127, MOVE_BACKWARD, 1.8, 0, 2.5, 1, 0, 0, 15000, 1, hardwareParameter);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, 0.1, 1200, 2, hardwareParameter);
	goStraightCmPID_lib(30, 180, 70, MOVE_BACKWARD, 1.8, 0, 2.5, 1, 0, 0, 15000, 1, hardwareParameter);
}

/**************************
AUTON
**************************/
void autonomous()
{
	arm_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	intake_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	left_front_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	left_back_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_front_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_back_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);

	// left_front_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	// left_back_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	// left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	// right_front_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	// right_back_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	// right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	left_back_motor.set_zero_position(0.0);
	left_front_motor.set_zero_position(0.0);
	left_mid_motor.set_zero_position(0.0);
	right_back_motor.set_zero_position(0.0);
	right_front_motor.set_zero_position(0.0);
	right_mid_motor.set_zero_position(0.0);
	intake_motor.set_zero_position(0.0);
	arm_motor.set_zero_position(0.0);

	sys_initial_to_auton_drifting = hardwareParameter.inertialSensorLib.get_rotation();

	//   //this line records the inertial sensor drifting between initialization and start of auton.
	//  	sys_initial_robot_heading = 270;
	// 	turnDegreesPID_Parameter = {0, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 1500, 2};
	//   pros::Task background(background_execution_turnDegreesPID_lib, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "turn");

	//   double distance = get_distance_back_vision(back_vision, DETECT_BLUE_GOAL_SIG, 50, 100, 60, 10000);

	// waitForTouch();

	// waitForTouch();
	// //	pros::lcd::print(2, "Time=%d", pros::millis() - start_time);

	/***********************************************

	CHOOSE RUN HERE

	***********************************************/

	// test();
	// test_vision();
	//right_side();
	// right_side_blue();
	// left_side_red();
	// left_side_blue();
	// mid_goal_red();
	// mid_goal_blue();
	 auton_60s_skills_bridge_version();
	// winpoint();
	// finals_left_red();
	// finals_left_blue();
	// finals_right_red();
	// finals_right_blue();
	// finals_mid_red(); // DOESNT WORK
	// finals_mid_blue(); // DOESNT WORK
	// bridgeproof_red();
	// bridgeproof_blue();

	waitForTouch();
}
