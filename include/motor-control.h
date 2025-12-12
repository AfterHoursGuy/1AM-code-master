#include <string>
#include <cmath>
#include "vex.h"

// --- Global Variables (snake_case) ---
extern bool is_turning;

extern double xpos, ypos;
extern double correct_angle;

// --- Function Declarations (lowerCamelCase) ---
void driveChassis(double left_power, double right_power);

double getInertialHeading();
double normalizeTarget(double angle);

void turnToAngle(double turn_angle, double time_limit_msec, bool exit = true, double max_output = 12);
void driveTo(double distance_in, double time_limit_msec, bool exit = true, double max_output = 12);
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit = true, double max_output = 12);
void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit = true, double max_output = 12);

void stopChassis(vex::brakeType type = vex::brake);
void resetChassis();
double getLeftRotationDegree();
double getRightRotationDegree();
void correctHeading();
void trackNoOdomWheel();
void trackXYOdomWheel();
void trackXOdomWheel();
void trackYOdomWheel();
void turnToPoint(double x, double y, int dir, double time_limit_msec);
void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
void driveToWall2D(double target_in, double time_limit_msec, double hold_time, bool exit, double max_output, vex::distance& sensor, double target_side_in);
void driveToWall(double target_distance_in, double time_limit_msec, double hold_time, bool exit = true, double max_output = 12);
void driveFromWall(double target_in, double time_limit_msec, double hold_time, bool exit, double max_output = 12);
void driveToWallRight(double target_in, double time_limit_msec, double hold_time, bool exit, double max_output = 12);
void correctPositionWithSensors(vex::distance& front_sensor, vex::distance& side_sensor, double front_offset, double side_offset, double estimated_x, double estimated_y, double estimated_heading);
void resetPositionWithSensor(vex::distance& sensor, double sensor_offset_x, double sensor_offset_y, double sensor_angle_offset, double field_half_size);
void resetPositionFront(vex::distance& sensor, double sensor_offset_x, double sensor_offset_y, double field_half_size);
void resetPositionBack(vex::distance& sensor, double sensor_offset_x, double sensor_offset_y, double field_half_size);
void resetPositionLeft(vex::distance& sensor, double sensor_offset_x, double sensor_offset_y, double field_half_size);
void resetPositionRight(vex::distance& sensor, double sensor_offset_x, double sensor_offset_y, double field_half_size);
void setFieldDimensions(double half_width, double half_length);
void initializeFieldPosition(vex::distance& x_sensor, double x_sensor_offset_x, double x_sensor_offset_y, double x_sensor_angle, vex::distance& y_sensor, double y_sensor_offset_x, double y_sensor_offset_y, double y_sensor_angle);
void softarmPID(double arm_target);
void fastarmPID(double arm_target);
void lights(int _red, int _green, int _blue);
void moveToPrevPos();