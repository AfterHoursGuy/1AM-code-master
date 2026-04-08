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
void driveToWall(double target_distance_in, double time_limit_msec, double hold_time, bool exit = true, double max_output = 12);
void driveFromWall(double target_in, double time_limit_msec, double hold_time, bool exit, double max_output = 12);
void driveToWallRight(double target_in, double time_limit_msec, double hold_time, bool exit, double max_output = 12);
void resetPositionWithSensor(vex::distance& sensor, double sensor_offset_x, double sensor_offset_y, double sensor_angle_offset, double wall_angle, bool frontback);
void resetPositionFront(vex::distance& sensor, double sensor_offset_x, double sensor_offset_y, double wallangle);
void resetPositionBack(double wallangle);
void resetPositionLeft(double wallangle);
void resetPositionRight(double wallangle);
void trackdistanceodom(bool left, bool right, bool back, bool front, bool FR);
void lights(int _red, int _green, int _blue);
void moveToPrevPos();