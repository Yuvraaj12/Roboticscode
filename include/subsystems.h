#ifndef SUBSYSTEMS_H
#define SUBSYSTEMS_H

#include "main.h"
#include "config.h"

extern double lift_target;
extern double lift_error_sum;
extern double last_lift_error;

void setLiftTarget(double target);
void updateClawFromController(pros::Controller& controller);
void updateLiftFromController(pros::Controller& controller);
void updateLiftPID();

#endif
