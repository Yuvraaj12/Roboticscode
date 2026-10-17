#include "main.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
pros::MotorGroup left_motor_group({11, 12}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({-19, -20}, pros::MotorGears::blue);
pros::Motor left_lift(18, pros::MotorGears::green);
pros::Motor right_lift(-13, pros::MotorGears::green);
pros::Motor claw(-9, pros::MotorGears::green);
pros::ADIDigitalIn auto_selector ('A');
pros::Imu imu(10);
pros::Distance dNorth(14);
pros::Distance dSouth(19);
pros::Distance dEast(15);
pros::Distance dWest(17);
pros::Rotation horizontal_encoder(15);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0);
pros::Rotation vertical_encoder(16);
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
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
                            &imu // inertial sensor
);

    // lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

    // angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              12.3, // derivative gain (kD)
                                              3, // anti windup
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