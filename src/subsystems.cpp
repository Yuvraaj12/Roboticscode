#include "main.h"
#include "lemlib/api.hpp"
#include "config.h"
#include "subsystems.h"

static const double kLiftStep = 200.0;
static const double kLiftMaxTarget = 3700.0;
static const double kLiftSecondObjectTarget = 300.0;
static const uint32_t kLiftRepeatIntervalMs = 50;

bool intake_on = false;
bool reverse_in_progress = false;
bool claw_hold_active = false;
double lift_target = 0.0;
double lift_output_limit = 127.0;
double lift_error_sum = 0.0;
double last_lift_error = 0.0;

void setLiftTarget(double target) {
    if (target < 0.0) {
        target = 0.0;
    } else if (target > kLiftMaxTarget) {
        target = kLiftMaxTarget;
    }

    lift_target = target;
    lift_error_sum = 0.0;
    last_lift_error = 0.0;
}

void setLiftSpeed(double speed) {
    if (speed < 0.0) {
        speed = 0.0;
    } else if (speed > 127.0) {
        speed = 127.0;
    }

    lift_output_limit = speed;
}

void reverse_claw_task(void*) {
    double next_lift_target = lift_target - 250.0;
    if (next_lift_target < 0.0) {
        next_lift_target = 0.0;
    }
    setLiftTarget(next_lift_target);
    lift_output_limit = 50.0;
     

    pros::delay(200);

    claw.move(-80);
    pros::delay(400);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw.brake();

    lift_output_limit = 127.0;
    claw_hold_active = true;
    reverse_in_progress = false;
}

void updateClawFromController(pros::Controller& controller) {
    if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
       setLiftTarget(0.0);
        intake_on = !intake_on;
        claw_hold_active = false;
    }

    if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_UP) && !reverse_in_progress) {
        reverse_in_progress = true;
        intake_on = false;
        claw_hold_active = false;
        pros::Task reverse_task(reverse_claw_task);
    }

    if (claw_hold_active) {
        claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        claw.brake();
    } else if (intake_on && !reverse_in_progress) {
        claw.move(127);
    } else if (!reverse_in_progress) {
        claw.move(20);
    }
}

void updateLiftFromController(pros::Controller& controller) {
    static bool l1_was_down = false;
    static bool l2_was_down = false;
    static uint32_t last_step_time = 0;

    if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)) {
        setLiftTarget(kLiftSecondObjectTarget);
    }

    bool l1_down = controller.get_digital(E_CONTROLLER_DIGITAL_L1);
    bool l2_down = controller.get_digital(E_CONTROLLER_DIGITAL_L2);
    uint32_t now = pros::millis();

    if (l1_down && (!l1_was_down || now - last_step_time >= kLiftRepeatIntervalMs)) {
        setLiftTarget(lift_target + kLiftStep);
        last_step_time = now;
    } else if (l2_down && (!l2_was_down || now - last_step_time >= kLiftRepeatIntervalMs)) {
        setLiftTarget(lift_target - kLiftStep);
        last_step_time = now;
    }

    l1_was_down = l1_down;
    l2_was_down = l2_down;
}

void updateLiftPID() {
    const double kP = 0.8;
    const double kI = 0.0;
    const double kD = 1.7;
    double max_voltage = lift_output_limit;

    double avg_position = lift_motors.get_position();
    double error = lift_target - avg_position;
    lift_error_sum += error;
    double derivative = error - last_lift_error;
    double output = (error * kP) + (lift_error_sum * kI) + (derivative * kD);

    if (output > max_voltage) output = max_voltage;
    if (output < -max_voltage) output = -max_voltage;

    lift_motors.move(output);
    last_lift_error = error;
}
