#include "main.h"
#include "TransmissionConfiguration.hpp"

using namespace pros;

pros::Task intake_set(intake_set_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "intake set");
pros::Task hook_set(hook_set_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "hook set");
pros::Task claw_set(claw_set_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "claw set");
pros::Task arm_set(arm_set_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "arm set");

void display_vision_error(int line, std::string msg){
	if(errno == 0){
		pros::lcd::print(line, "n=%d  %s", errno, msg);
	} else {
		std::string str = "";
		if(errno == ENODEV){
			str = "port not vision";
		} else if(errno == EINVAL){
			str = "sig_id out of range";
		} else if(errno == EDOM) {
			str = "size_id too large";
		} else if(errno == EAGAIN){
			str = "read vision fail";
		} else {
			str = "unknown";
		}
		pros::lcd::print(line, "n=%d  %s", errno, str);
    waitForTouch();
	}
}

void initialize()
{
	arm_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	intake_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	left_front_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	left_back_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_front_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_back_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);

	arm_motor.tare_position();
	left_front_motor.tare_position();
	left_back_motor.tare_position();
	left_mid_motor.tare_position();
	right_front_motor.tare_position();
	right_back_motor.tare_position();
	right_mid_motor.tare_position();
	intake_motor.tare_position();

	arm_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	left_front_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	left_back_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	left_mid_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	right_front_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	right_back_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	right_mid_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	intake_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

	Y_encoder.reset();

	// pros::vision_signature_s_t red_goal_sig = pros::Vision::signature_from_utility(
	//  	                    DETECT_RED_GOAL_SIG, 7527, 9761, 8644, -1293, -819, -1056, 2.400, 0);
 //
	// pros::vision_signature_s_t yellow_goal_sig = pros::Vision::signature_from_utility(
	//  										  DETECT_YELLOW_GOAL_SIG, 2127, 2595, 2360, -3867, -3397, -3632, 3.500, 0);
 //
 // pros::vision_signature_s_t blue_goal_sig = pros::Vision::signature_from_utility(
 // 										  DETECT_BLUE_GOAL_SIG, -1917, -1095, -1506, 5259, 8681, 6970, 2.200, 0);
 //
	vision_signature_s_t red_goal_sig = pros::Vision::signature_from_utility(
		                    DETECT_RED_GOAL_SIG, 7761, 9095, 8428, -907, -503, -705, 4.400, 0);
	vision_signature_s_t blue_goal_sig = pros::Vision::signature_from_utility(
											  DETECT_BLUE_GOAL_SIG, -2383, -1413, -1898, 8113, 11627, 9870, 3.100, 0);
	vision_signature_s_t yellow_goal_sig = pros::Vision::signature_from_utility(
											  DETECT_YELLOW_GOAL_SIG, 1783, 2259, 2021, -3573, -3215, -3394, 5.500, 0);

	front_vision.clear_led();
	front_vision.set_wifi_mode(0); //disable wifi
	front_vision.set_exposure(48);
    front_vision.set_signature(DETECT_RED_GOAL_SIG, &red_goal_sig);
    front_vision.set_signature(DETECT_BLUE_GOAL_SIG, &blue_goal_sig);
	front_vision.set_signature(DETECT_YELLOW_GOAL_SIG, &yellow_goal_sig);

	back_vision.clear_led();
	back_vision.set_wifi_mode(0); //disable wifi
	back_vision.set_exposure(30);
  back_vision.set_signature(DETECT_RED_GOAL_SIG, &red_goal_sig);
  back_vision.set_signature(DETECT_BLUE_GOAL_SIG, &blue_goal_sig);
	back_vision.set_signature(DETECT_YELLOW_GOAL_SIG, &yellow_goal_sig);

	pros::lcd::initialize();
	inertial_sensor.reset();
  delay(2500);



/*
	pros::lcd::initialize();
	pros::lcd::register_btn0_cb(lcd_left_button);   //red or blue team selection button
	pros::lcd::register_btn1_cb(lcd_center_button); //square or rectangle selection button
	pros::lcd::register_btn2_cb(lcd_right_button);  //confirm button
	pros::delay(500);
	pros::lcd::set_text(0, "         ----16868A----");
	pros::lcd::set_text(1, "Inertial sensor initializing......");
	inertial_sensor.reset();
	pros::delay(3000);
*/

//	pros::Task odometry(odometry_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odometry");
//	pros::Task infoPrint(infoPrint_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "infoprint");
}


void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
