#include "main.h" // IWYU pragma: keep
#include "lemlib/api.hpp"
#include "config.h"
#include "autons.h"
#include "subsystems.h"
#include "mcl.hpp"
int auton_select = 0;
int last_auton_cycle_ms = 0;
pros::Controller master(pros::E_CONTROLLER_MASTER);

void subsystem_loop(void*) {
 	while (true) {
		if (!pros::competition::is_autonomous()) {
			updateClawFromController(master);
		}
 		updateLiftFromController(master);
 		updateLiftPID();
 		pros::delay(20);
 	}
}

void selector_loop() {
 	while (true) {
 		if (pros::competition::is_disabled() && auto_selector.get_new_press()) {
 			if (pros::millis() - last_auton_cycle_ms >= 50) {
 				last_auton_cycle_ms = pros::millis();
 				auton_select = (auton_select + 1) % 6;
 			}
 		}
 		pros::delay(20);
 	}
}

void autonomous(){
 	switch(auton_select){
 		case 0: left_red(); break;
 		case 1: right_red(); break;
 		case 2: left_blue(); break;
 		case 3: right_blue(); break;
 		case 4: sawp(); break;
 		case 5: skills(); break;
 	}
}

void print_brain() {
    while (true) {
		const char* auton_name = "Unknown";

		switch (auton_select) {
		    case 0: auton_name = "Left Red"; break;
		    case 1: auton_name = "Right Red"; break;
		    case 2: auton_name = "Left Blue"; break;
		    case 3: auton_name = "Right Blue"; break;
		    case 4: auton_name = "Sawp"; break;
		    case 5: auton_name = "Skills"; break;
		    default: auton_name = "Unknown"; break;
		}

		pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
		pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
		pros::lcd::print(2, "T: %.2f", chassis.getPose().theta);
		pros::lcd::print(3, "DN: %.2f", dNorth.get() / 25.4);
		pros::lcd::print(4, "DS: %.2f", dSouth.get() / 25.4);
		pros::lcd::print(5, "DE: %.2f", dEast.get() / 25.4);
		pros::lcd::print(6, "DW: %.2f", dWest.get() / 25.4);
		pros::lcd::print(7, "Auto: %s", auton_name);
		pros::delay(100);
    }
}

void initialize() {
	pros::lcd::initialize(); // initialize brain screen
	chassis.calibrate(); // calibrate sensors
	lift_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lift_motors.tare_position();
	lift_target = 0.0;
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	pros::Task print_brain_task(print_brain); // print pose + auton info to the brain screen
	pros::Task selector_task(selector_loop); // watch the physical auton selector while disabled
	pros::Task subsystem_task(subsystem_loop); // keep lift and claw logic updating in the background
}
void opcontrol() {
	setLiftSpeed(127.0);

	while (true) {
		double forward = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		double turn = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		chassis.arcade(forward, turn);
  
		pros::delay(20);
	}
}