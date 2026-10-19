#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>
#include "config.h"
#include "subsystems.h"

static const double kLiftStageTargets[] = {0.0, 600.0, 800.0, 1000.0, 1200.0, 1400.0};
static const int kLiftStageCount = sizeof(kLiftStageTargets) / sizeof(kLiftStageTargets[0]);
static const double kLiftSecondObjectTarget = 300.0;

bool intake_on = false;
bool reverse_in_progress = false;
bool claw_hold_active = false;
int lift_state = 0;
double lift_target = 0.0;
double lift_output_limit = 127.0;
double lift_error_sum = 0.0;
double last_lift_error = 0.0;

void setLiftTarget(double target) {
    lift_target = target;
    lift_error_sum = 0.0;
    last_lift_error = 0.0;
}

void applyLiftState() {
    if (lift_state < 0) {
        lift_state = 0;
    } else if (lift_state >= kLiftStageCount) {
        lift_state = kLiftStageCount - 1;
    }

    setLiftTarget(kLiftStageTargets[lift_state]);
}

void syncLiftStateFromTarget() {
    int closest_state = 0;
    double closest_distance = 1e9;

    for (int i = 0; i < kLiftStageCount; i++) {
        double distance = std::abs(lift_target - kLiftStageTargets[i]);
        if (distance < closest_distance) {
            closest_distance = distance;
            closest_state = i;
        }
    }

    lift_state = closest_state;
}

void reverse_claw_task(void*) {
    double next_lift_target = lift_target - 200.0;
    if (next_lift_target < 0.0) {
        next_lift_target = 0.0;
    }
    setLiftTarget(next_lift_target);
    syncLiftStateFromTarget();
    lift_output_limit = 50.0;

    pros::delay(200);

    claw.move_relative(-450, 80);
    pros::delay(400);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw.brake();

    lift_output_limit = 127.0;
    claw_hold_active = true;
    reverse_in_progress = false;
}

void updateClawFromController(pros::Controller& controller) {
    if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
        lift_state = 0;
        setLiftTarget(kLiftStageTargets[0]);
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
    if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)) {
        lift_state = 1;
        setLiftTarget(kLiftSecondObjectTarget);
        return;
    }

    if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L1) && lift_state < kLiftStageCount - 1) {
        lift_state += 1;
        applyLiftState();
    } else if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L2) && lift_state > 0) {
        lift_state -= 1;
        applyLiftState();
    }
}

void updateLiftPID() {
    const double kP = 0.8;
    const double kI = 0.02;
    const double kD = 0.15;
    double max_voltage = lift_output_limit;

    double avg_position = lift_motors.get_position();
    double error = lift_target - avg_position;
    lift_error_sum += error;
    double derivative = error - last_lift_error;
    double output = (error * kP) + (lift_error_sum * kI) + (derivative * kD);

    if (error < 0.0) {
        if (lift_output_limit < 60.0) {
            max_voltage = lift_output_limit;
        } else {
            max_voltage = 60.0;
        }
    } else {
        max_voltage = lift_output_limit;
    }

    if (output > max_voltage) output = max_voltage;
    if (output < -max_voltage) output = -max_voltage;

    lift_motors.move(output);
    last_lift_error = error;
}
