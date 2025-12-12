#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }
void score() {
  fastarmPID(135);
}

void reset() {
  stick.spin(fwd, -7, voltageUnits::volt);
  wait(500, msec);
  stick.stop(hold);
}

void doinker(int t){
  wait(t, msec);
  scraper.set(true);
}

void doinkerup(int t){
  wait(t, msec);
  scraper.set(false);
}

void exampleAuton() {
  // Use this for tuning linear and turn pid
  driveTo(60, 3000);
  turnToAngle(90, 2000);
  turnToAngle(135, 2000);
  turnToAngle(150, 2000);
  turnToAngle(160, 2000);
  turnToAngle(165, 2000);
  turnToAngle(0, 2000);
  driveTo(-60, 3000);
}

void exampleAuton2() {
  driveToWall2D(12, 2000, 500, true, 10, leftSide, 12);
  
}

void sigsoloAWP(){
  mid_goal.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  gate.set(true);
  driveTo(6, 1000, true, 10);
  wait(200, msec);
  moveToPoint(0, -33.5, -1, 1300, true, 10);
  scraper.set(true);
  wait(100, msec);
  turnToAngle(-90, 600);
  driveToWall(7.5, 900, 200, true, 12);
  moveToPoint(22.5, -40, -1, 1200, true, 10);
  gate.set(false);
  wait(50, msec);
  stick.spin(fwd, 10, voltageUnits::volt);
  wait(400, msec);
  scraper.set(false);
  stick.spin(fwd, -4, voltageUnits::volt);
  gate.set(true);
  driveTo(2, 1000, false, 12);
  swing(40, 1, 1000, true, 9);
  driveTo(10, 1000, false, 12);
  boomerang(26, 44, 1, 0, 0.3, 2500, true, 9);
  scraper.set(true);
  mid_goal.set(false);
  wait(400, msec);
  scraper.set(false);
  moveToPoint(34, 22.5, -1, 1200, true, 10);
  gate.set(false);
  wait(100, msec);
  stick.spin(fwd, 11, voltageUnits::volt);
  wait(250, msec);
  stick.spin(fwd, -8, voltageUnits::volt);
  gate.set(true);
  mid_goal.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  boomerang(-2, 55, 1, -90, 0.3, 2000, true, 10);
  scraper.set(true);
  turnToAngle(-90, 600);
  driveToWall(7.5, 900, 200, true, 12);
  moveToPoint(21, 54, -1, 1200, true, 10);
  gate.set(false);
  wait(75, msec);
  stick.spin(fwd, 10, voltageUnits::volt);
  wait(600, msec);
  stick.spin(fwd, -8, voltageUnits::volt);
}

void qualsoloawp(){
  
}

void rightsidequal(){
  
}

void leftsidequal(){
  mid_goal.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  wait(1000, msec);
  driveTo(32.5, 3000, true, 8);
  turnToAngle(-90, 1000);
  driveTo(-15, 2000, true, 8);
  fastarmPID(128);
  wait(500, msec);
  fastarmPID(0);
  driveTo(13, 2000, true, 8);
  turnToAngle(135, 1000);
  driveTo(40, 6000, true, 8);
  turnToAngle(-45, 1000);
  driveTo(-4, 2000, true, 8);
  fastarmPID(130);
  wait(500, msec);
  fastarmPID(0);
  swing(185, 1, 1800, true, 8);
  driveTo(40, 2000, true, 8);
  turnToAngle(45, 1000);
  driveTo(20, 2000, true, 8);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
}

void skills() {
  resetChassis();
  wait(10, msec);
  stick.setStopping(hold);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  wings.set(true);
  gate.set(true);
  mid_goal.set(true);
  boomerang(-20, 12, 1, -90, 0.4, 4000, false, 6); 
  boomerang(-35.5, -8, 1, 180, 0.2, 2000, true, 8);
  turnToAngle(180, 500);
  moveToPoint(-35, 11, -1, 1500, true, 8);
  driveToWall(34.75, 600, 0, true, 12);
  gate.set(false);
  stick.spin(fwd, 60, percent);
  wait(600, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(200, msec);
  thread([]{reset();});
  gate.set(true);
  scraper.set(true);
  initializeFieldPosition(rightSide, 6.375, 0, 90, Rwall_distance_sensor, 4.25, 5, 0);
  moveToPoint(-46, -59, 1, 1200, true, 8);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  driveToWall(6.5, 1000, 250, true, 12);
  driveToWall(10.5, 500, 0, true, 12);
  driveToWall(6.5, 1000, 500, true, 12);
  turnToAngle(180, 500);  
  boomerang(-58, -24, -1, 180, 0.3, 1000, false, 11);
  moveToPoint(-56, 42, -1, 1500, true, 11);
  wait(100, msec);
  turnToAngle(90, 800);
  wait(100, msec);
  driveFromWall(15, 1000, 0, true, 12);
  wait(100, msec);
  turnToAngle(0, 800);
  wait(100, msec);
  driveToWall(34.75, 1000, 0, true, 12);
  gate.set(false);
  stick.spin(fwd, 20, percent);
  wait(600, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(800, msec);
  thread([]{reset();});
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  resetPositionFront(Rwall_distance_sensor, 4.125, 5, 70.25);
  resetPositionLeft(leftSide, 6.375, 0, 70.25);
  correct_angle = 3;
  driveTo(26, 2000, true, 10);
  gate.set(true);
  driveToWall(6.5, 1000, 250, true, 12);
  driveToWall(10.5, 500, 0, true, 12);
  driveToWall(6.5, 1000, 500, true, 12);
  moveToPrevPos();
  gate.set(false);
  stick.spin(fwd, 20, percent);
  wait(600, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(1100, msec);
  thread([]{reset();});
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  scraper.set(false);
  resetPositionFront(Rwall_distance_sensor, 4.125, 5, 70.25);
  resetPositionLeft(leftSide, 6.375, 0, 70.25);
  boomerang(-16, 69, 1, 90, 0.3, 1200, true, 10);
  mid_goal.set(false);
  stick.spin(fwd, 100, percent);
  wait(100, msec);
  turnToAngle(90, 800);
  thread([]{reset();});
  wait(100, msec);
  gate.set(true);
  driveChassis(8, 8);
  scraper.set(true);
  wait(600, msec);
  scraper.set(false);
  wait(400, msec);
  driveChassis(0, 0);
  wait(200, msec);
  driveChassis(6, 6);
  wait(1000, msec);
  driveToWallRight(30, 800, 0, true, 8);
  driveToWallRight(42, 800, 0, true, 8);
  swing(180, 1, 1500, true, 12);
  turnToAngle(180, 300);
  wait(50, msec);
  resetPositionBack(backSide, 0, 9, 70.25);
  resetPositionLeft(leftSide, 6.375, 0, 70.25);
  wait(50, msec);
  turnToAngle(215, 800);
  wait(50, msec);
  boomerang(19, 20, 1, 135, 0.3, 2000, true, 10);
  wait(50, msec);
  turnToAngle(45, 800);
  wait(50, msec);
  moveToPoint(3.5, 18.5, -1, 1200, true, 8);
  wait(50, msec);
  turnToAngle(45, 800);
  gate.set(false);
  wait(100, msec);
  stick.spin(fwd, 20, percent);
  wait(600, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(1100, msec);
  thread([]{reset();});
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  scraper.set(true);
  thread([]{
    stick.spin(fwd, 100, percent);
    wait(600, msec);
    reset();
  });
  boomerang(36, 59, 1, 0, 0.4, 1500, true, 10);
  mid_goal.set(true);
  gate.set(true);
  driveToWall(6.5, 1000, 250, true, 12);
  driveToWall(10.5, 500, 0, true, 12);
  driveToWall(6.5, 1000, 500, true, 12);
  driveTo(-4, 1000, true, 12);
  wait(50, msec);
  resetPositionFront(Rwall_distance_sensor, 4.125, 5, 70.25);
  resetPositionRight(rightSide, 6.375, 0, 70.25);
  wait(50, msec);
  boomerang(58, 20, -1, 0, 0.3, 1200, false, 11);
  moveToPoint(57, -48, -1, 2200, true, 11);
  wait(100, msec);
  turnToAngle(270, 800);
  wait(100, msec);
  driveFromWall(15, 1200, 0, true, 12);
  wait(100, msec);
  turnToAngle(180, 800);
  wait(100, msec);
  driveToWall(34.75, 1000, 0, true, 12);
  gate.set(false);
  stick.spin(fwd, 20, percent);
  wait(600, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(800, msec);
  thread([]{reset();});
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  resetPositionFront(Rwall_distance_sensor, 4.125, 5, 70.25);
  resetPositionLeft(leftSide, 6.375, 0, 70.25);
  correct_angle = 183;
  driveTo(26, 2000, true, 10);
  gate.set(true);
  driveToWall(6.5, 1000, 250, true, 12);
  driveToWall(10.5, 500, 0, true, 12);
  driveToWall(6.5, 1000, 500, true, 12);
  turnToAngle(180, 500);  
  moveToPrevPos();
  gate.set(false);
  stick.spin(fwd, 20, percent);
  wait(600, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(800, msec);
  scraper.set(false);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  resetPositionFront(Rwall_distance_sensor, 4.125, 5, 70.25);
  resetPositionLeft(leftSide, 6.375, 0, 70.25);
  driveTo(8, 1000, true, 10);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  mid_goal.set(false);
  boomerang(12, -8, 1, -90, 0.4, 1600, true, 10);
  driveFromWall(65, 5000, 0, true, 12);

}

void elimleft(){
  mid_goal.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  gate.set(true);
  boomerang(-20, 18, 1, -90, 0.4, 1500, false, 10); 
  scraper.set(true);
  boomerang(-38, -8, 1, 180, 0.2, 2000, true, 9);
  turnToAngle(180, 300);
  driveToWall(7.5, 900, 300, true, 12);
  thread([]{
    lower_intake.spin(fwd, -6, voltageUnits::volt);
    wait(200, msec);
    lower_intake.spin(fwd, 12, voltageUnits::volt);
  });
  moveToPoint(-36.5, 15, -1, 1200, true, 10);
  gate.set(false);
  wait(50, msec);
  stick.spin(fwd, 12, voltageUnits::volt);
  wait(600, msec);
  lower_intake.stop();
  scraper.set(false);
  driveTo(4, 1000, false, 12);
  stick.spin(fwd, -1, voltageUnits::volt);
  turnToAngle(-90, 600);
  driveToWall(28.5, 800, 0, true, 12);
  turnToAngle(180, 600);
  driveTo(-27, 10000, true, 12);
  left_chassis.setStopping(hold);
  right_chassis.setStopping(hold);
  lower_intake.spin(forward, 1, voltageUnits::volt);
  stick.spin(reverse, 1, voltageUnits::volt);
  lower_intake.stop();

}

void elimright(){
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  gate.set(true);
  thread([]{
    wait(500, msec);
    mid_goal.set(true);
  });
  boomerang(27, 26, 1, 90, 0.4, 1500, false, 10); 
  scraper.set(true);
  boomerang(55, -8, 1, 180, 0.2, 2000, true, 9);
  turnToAngle(180, 300);
  driveToWall(7, 900, 300, true, 12);
  scraper.set(false);
  thread([]{
    lower_intake.spin(fwd, -6, voltageUnits::volt);
    wait(200, msec);
    lower_intake.spin(fwd, 6, voltageUnits::volt);
  });
  moveToPoint(55, 15.5, -1, 1200, true, 10);
  gate.set(false);
  wait(50, msec);
  //stick.spin(fwd, 12, voltageUnits::volt);
  //wait(600, msec);
  stick.spin(fwd, 70, percent);
  wait(1200, msec);
  lower_intake.stop();
  scraper.set(false);
  driveTo(4, 1000, false, 12);
  stick.spin(fwd, -1, voltageUnits::volt);
  turnToAngle(90, 600);
  driveToWall(7, 800, 0, true, 12);
  turnToAngle(180, 600);
  left_chassis.setStopping(hold);
  right_chassis.setStopping(hold);
  wait(1000, msec);
  driveTo(-27, 10000, true, 11);//12
  lower_intake.spin(forward, 1, voltageUnits::volt);
  stick.spin(reverse, 12, voltageUnits::volt);
  lower_intake.stop();

}