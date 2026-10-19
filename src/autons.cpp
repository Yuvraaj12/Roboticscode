#include "main.h"
#include "lemlib/api.hpp"
#include "subsystems.h"
#include "config.h"
#include "mcl.hpp"
void move_relative(double x, double y, double timeout, bool forwards, float speed=127){
    chassis.moveToPoint(chassis.getPose().x + x,
                              chassis.getPose().y + y,
                              timeout, {forwards, speed});
}
void drive_distance(double distance, double timeout, float speed=127){
    chassis.moveToPoint(chassis.getPose().x + distance * cos((-chassis.getPose().theta+90) * M_PI / 180.0),
                              chassis.getPose().y + distance * sin((-chassis.getPose().theta+90) * M_PI / 180.0),
                              timeout, {(distance>=0), speed});
}
void drive_distance_from_wall(double distance, double timeout, float speed=127, bool from_front=true, bool loop=true){
    double exit_range = 2.0;
    double field_length = 144.0;
    double dist_from_wall = 0;
    double time = pros::millis();
    bool forward = true;
    while(std::abs(dist_from_wall - distance) > exit_range && (pros::millis() - time) < timeout){
        double front_distance = dNorth.get()/25.4;
        double back_distance = dSouth.get()/25.4;
        bool front_valid=false;
        bool back_valid=false;
        if(front_distance < 2000/25.4 && front_distance > 10/25.4) front_valid=true;
        if(back_distance < 2000/25.4 && back_distance > 10/25.4) back_valid=true;
        if(from_front && front_valid){
            dist_from_wall = front_distance + 4.67;
            forward=true;
            
        }else if(back_valid&& !from_front){
            dist_from_wall = back_distance + 5.67;
            forward=false;
        }
        drive_distance((forward?1:-1)*(dist_from_wall-distance), 5000, speed);
        chassis.waitUntilDone();
        if(!loop) break;
        pros::delay(20);
        
    }
}
// TODO
void drive_until_detect_distance(double distance, double timeout, float speed, bool forward){
    double time = pros::millis();
    double distance_from_wall = forward?dNorth.get()/25.4 + 4.67:dSouth.get()/25.4 + 5.67;
    while((pros::millis() - time) < timeout){
    }
}
void test360() {
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, -12, 1000, {.forwards = false});
  chassis.turnToHeading(180, 5000,
                        {.direction = lemlib::AngularDirection::CW_CLOCKWISE,
                         .maxSpeed = 70,
                         .earlyExitRange = .00000000001});
  chassis.turnToHeading(0, 5000,
                        {.direction = lemlib::AngularDirection::CW_CLOCKWISE,
                         .maxSpeed = 100,
                         .earlyExitRange = .00000000001},
                        false);
  chassis.turnToHeading(0, 5000,
                        {.direction = lemlib::AngularDirection::CW_CLOCKWISE,
                         .maxSpeed = 100,
                         .earlyExitRange = .00000000001},
                        false);

  {
    {                                                  // Get sensor readings
      double distNorth = dNorth.get_distance() / 25.4; // Convert to inches
      double distNorthW = dSouth.get_distance() / 25.4;

      // Expected sensor reading at (0,0,0) - when aligned with wall
      double expectedDistance = 9.1;

      // Calculate angle error (in degrees) -  sensors are facing Y axis now
      // If dSouth > dNorth, robot is rotated clockwise, theta error is
      // negative
      double theta_error_rad =
          atan2((distNorthW - distNorth),
                13.75); // Still use horizontal separation for angle
      double theta_error_deg = theta_error_rad * (180 / M_PI);

      // Calculate Y position error (in inches) - sensors facing Y axis
      double averageDistance = (distNorth + distNorthW) / 2.0;
      double y_error = averageDistance - expectedDistance;

      // Get current pose
      lemlib::Pose current_pose = chassis.getPose();

      // Calculate corrected pose - now correcting Y and Theta
      double corrected_theta = current_pose.theta - theta_error_deg;
      double corrected_y = current_pose.y - y_error;
      double corrected_x = current_pose.x; // Keep X unchanged

      // Set corrected pose
      chassis.setPose(corrected_x, corrected_y, corrected_theta);

      std::cout << "dNorth: " << distNorth << " inches, dSouth: " << distNorthW
                << " inches" << std::endl;
      std::cout << "Theta Error: " << theta_error_deg
                << " degrees, Y Error: " << y_error << " inches" << std::endl;
      std::cout << "Corrected Pose: x=" << corrected_x << ", y=" << corrected_y
                << ", theta=" << corrected_theta << std::endl;
    }
  }
}

void left_red() {
    }

void right_red() {

}
void left_blue() {

}
void right_blue() {

}
void sawp() {

}
void skills() {

}