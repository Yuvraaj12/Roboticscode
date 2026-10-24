#include "main.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
pros::MotorGroup left_motor_group({11, 12}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({-19, -20}, pros::MotorGears::blue);
pros::MotorGroup lift_motors({-18, 13}, pros::MotorGears::green);
pros::Motor claw(9, pros::MotorGears::green);
pros::adi::DigitalIn auto_selector('A');
pros::Imu imu(14);
pros::Distance dNorth(6);
pros::Distance dSouth(17);
pros::Distance dEast(1);
pros::Distance dWest(10);
pros::Rotation vertical_encoder(-16);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);

lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              9.7, // track width
                              lemlib::Omniwheel::NEW_325, // wheel
                              360, // drivetrain rpm
                              2 // horizontal drift
);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2
                            nullptr, // horizontal tracking wheel 1 disabled
                            nullptr, // horizontal tracking wheel 2
                            &imu // inertial sensor
);

    // lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              22, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

    // angular PID controller
lemlib::ControllerSettings angular_controller(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              40, // derivative gain (kD)
                                              1.205 , // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

    // create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);