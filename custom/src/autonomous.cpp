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
  driveToWall(7, 3000, 0);
  
}

void sigsoloAWP(){
  resetChassis();
  initializeFieldPosition(backSide, 9, 0, 180, leftSide, 0, 6.375, -90);
  mid_goal.set(true);
  wings.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  gate.set(true);
  correct_angle = -90;
  driveTo(6, 1000, true, 12);
  resetPositionBack(backSide, 9, 0, 70.25);
  resetPositionLeft(leftSide, 0, 6.375, 70.25);
  moveToPoint(43, -48, -1, 1300, true, 10);
  turnToAngle(180, 600);
  scraper.set(true);
  driveToWall(6.5, 1000, 0, true, 12);
  resetPositionFront(Rwall_distance_sensor, 4.25, 5, 70.25);
  resetPositionLeft(leftSide, 6.375, 0, 70.25);
  moveToPoint(47.5, -29, -1, 1300, true, 10);
  driveToWall(34.75, 500, 0, true, 12);
  gate.set(false);
  stick.spin(fwd, 40, percent);
  wait(900, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(100, msec);
  thread([]{reset();});
  gate.set(true);
  scraper.set(false);
  resetPositionFront(Lwall_distance_sensor, 4.25, 5, 70.25);
  resetPositionLeft(leftSide, 6.375, 0, 70.25);
  turnToPoint(-24, -24, 1, 1800);
  wait(50, msec);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  resetPositionLeft(leftSide, 0, 6.375, 70.25);
  resetPositionBack(backSide, 9, 0, 70.25);
  thread([]{doinker(1100);});
  moveToPoint(-26, -22, 1, 2000, true, 10);
  wait(50, msec);
  resetPositionFront(Lwall_distance_sensor, 5, 4.25, 70.25);
  resetPositionLeft(leftSide, 0, 6.375, 70.25);
  turnToAngle(225, 600);
  scraper.set(false);
  wait(50, msec);
  mid_goal.set(false);
  moveToPoint(-8, -11, -1, 1300, true, 10);
  wait(50, msec);
  turnToAngle(225, 600);
  gate.set(false);
  stick.spin(fwd, 30, percent);
  wait(500, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(100, msec);
  thread([]{reset();});
  gate.set(true);
  mid_goal.set(true);
  boomerang(-40, -43, 1, 180, 0.1, 2500, true, 9);
  scraper.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  resetPositionFront(Rwall_distance_sensor, 4.25, 5, 70.25);
  resetPositionRight(rightSide, 6.375, 0, 70.25);
  boomerang(-43, -53, 1, 180, 0.1, 2000, true, 8);
  driveToWall(6.5, 600, 0, true, 12);
  resetPositionFront(Rwall_distance_sensor, 4.25, 5, 70.25);
  resetPositionRight(rightSide, 6.375, 0, 70.25);
  moveToPoint(-45.75, -29, -1, 1500, true, 10);
  driveToWall(34.75, 700, 0, true, 12);
  gate.set(false);
  stick.spin(fwd, 60, percent);
  wait(600, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(100, msec);
  thread([]{reset();});
  gate.set(true);

}

void qualsoloawp(){

}

void rightsidequal(){
  resetChassis();
  initializeFieldPosition(backSide, 9, 0, 180, leftSide, 0, 6.375, -90);
  mid_goal.set(true);
  wings.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  gate.set(true);
  moveToPoint(43, -48, -1, 1300, true, 10);
  turnToAngle(180, 600);
  scraper.set(true);
  driveToWall(6.5, 1000, 0, true, 12);
  resetPositionFront(Rwall_distance_sensor, 4.25, 5, 70.25);
  resetPositionLeft(leftSide, 6.375, 0, 70.25);
  moveToPoint(47.5, -29, -1, 1300, true, 10);
  driveToWall(34.75, 500, 0, true, 12);
  gate.set(false);
  stick.spin(fwd, 40, percent);
  wait(900, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(100, msec);
  thread([]{reset();});
  gate.set(true);
  scraper.set(false);
  resetPositionFront(Lwall_distance_sensor, 4.25, 5, 70.25);
  resetPositionLeft(leftSide, 6.375, 0, 70.25);
  turnToPoint(-24, -24, 1, 1800);
  wait(50, msec);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  resetPositionLeft(leftSide, 0, 6.375, 70.25);
  resetPositionBack(backSide, 9, 0, 70.25);
  thread([]{doinker(1100);});
  moveToPoint(-26, -22, 1, 2000, true, 10);
  wait(50, msec);
  resetPositionFront(Lwall_distance_sensor, 5, 4.25, 70.25);
  resetPositionLeft(leftSide, 0, 6.375, 70.25);
  turnToAngle(225, 600);
  scraper.set(false);
  wait(50, msec);
  mid_goal.set(false);
  moveToPoint(-8, -11, -1, 1300, true, 10);
  wait(50, msec);
  turnToAngle(225, 600);
  gate.set(false);
  stick.spin(fwd, 20, percent);
  wait(1400, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(100, msec);
  thread([]{reset();});
  gate.set(true);
}

void leftsidequal(){
  
}

void skills() { 
  resetChassis();
  initializeFieldPosition(rightSide, 6.375, 0, 90, backSide, 0, 9, 180);
  wait(10, msec);
  stick.setStopping(hold);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  wings.set(true);
  gate.set(true);
  boomerang(-21, -28, 1, 315, 0.1, 2000, true, 8);
  wait(50, msec);
  lower_intake.stop();
  turnToAngle(225, 500);
  wait(50, msec);
  boomerang(-5, -17, -1, 45, 0.3, 2000, true, 8);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  turnToAngle(225, 500);
  gate.set(false);
  stick.spin(fwd, 33, percent);
  wait(400, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(300, msec);
  thread([]{reset();});
  scraper.set(true);
  moveToPoint(-40, -45, 1, 2000, true, 9);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  gate.set(true);
  turnToAngle(180, 500);
  resetPositionFront(Rwall_distance_sensor, 4.125, 5, 70.25);
  resetPositionRight(rightSide, 6.375, 0, 70.25);
  boomerang(-46.25, -61, 1, 180, 0.1, 2000, true, 6);
  mid_goal.set(true);
  driveToWall(7, 1000, 800, true, 12);
  turnToAngle(180, 500);  
  thread([]{
    doinkerup(0);
    doinker(167);
  });
  boomerang(-58, -24, -1, 180, 0.3, 1000, false, 11);
  moveToPoint(-57, 42, -1, 1500, true, 11);
  wait(100, msec);
  turnToAngle(90, 800);
  wait(100, msec);
  driveFromWall(15, 1100, 0, true, 12);
  wait(100, msec);
  turnToAngle(0, 800);
  wait(100, msec);
  driveToWall(34.75, 600, 0, true, 12);
  gate.set(false);
  stick.spin(fwd, 35, percent);
  wait(600, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(500, msec);
  thread([]{reset();});
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  turnToAngle(0, 500);
  resetPositionFront(Rwall_distance_sensor, 4.125, 5, 70.25);
  resetPositionLeft(leftSide, 6.375, 0, 70.25);
  correct_angle = 2;
  driveTo(30, 3000, true, 6.5);
  gate.set(true);
  driveToWall(7, 1000, 750, true, 12);
  thread([]{
    doinkerup(0);
    doinker(167);
  });
  moveToPrevPos();
  driveToWall(34.75, 200, 0, true, 12);
  gate.set(false);
  stick.spin(fwd, 20, percent);
  wait(600, msec);
  stick.spin(fwd, 10, percent);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(1100, msec);
  thread([]{reset();});
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  scraper.set(false);
  wait(50, msec);
  resetPositionFront(Rwall_distance_sensor, 4.125, 5, 70.25);
  resetPositionLeft(leftSide, 6.375, 0, 70.25);
  wait(50, msec);
  boomerang(-15, 70, 1, 90, 0.3, 1200, true, 10);
  mid_goal.set(false);
  stick.spin(fwd, 100, percent);
  wait(100, msec);
  turnToAngle(90, 800);
  thread([]{reset();});
  wait(100, msec);
  gate.set(true);
  driveChassis(8, 9);
  scraper.set(true);
  wait(600, msec);
  scraper.set(false);
  wait(400, msec);
  driveChassis(0, 0);
  wait(200, msec);
  driveChassis(6, 7);
  wait(1000, msec);
  driveToWallRight(30, 800, 0, true, 8);
  driveToWallRight(42, 700, 0, true, 8);
  swing(180, 1, 1500, true, 12);
  turnToAngle(180, 300);
  wait(50, msec);
  resetPositionBack(backSide, 0, 9, 70.25);
  resetPositionRight(rightSide, 6.375, 0, 70.25);
  turnToAngle(215, 800);
  wait(50, msec);
  boomerang(21, 25, 1, 135, 0.3, 2000, true, 10);
  lower_intake.stop();
  driveTo(7, 1000, true);
  wait(50, msec);
  turnToAngle(45, 900);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  wait(50, msec);
  boomerang(7, 13.25, -1, 45, 0.2, 1500, true, 9);
  wait(50, msec);
  turnToAngle(45, 800);
  gate.set(false);
  wait(100, msec);
  stick.spin(fwd, 17, percent);
  wait(700, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(1100, msec);
  thread([]{reset();});
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  scraper.set(true);
  mid_goal.set(true);
  boomerang(40.75, 50, 1, 45, 0.4, 1500, true, 11);
  turnToAngle(0, 700);
  wait(50, msec);
  resetPositionFront(Rwall_distance_sensor, 4.125, 5, 70.25);
  resetPositionRight(rightSide, 6.375, 0, 70.25);
  wait(50, msec);
  boomerang(46, 60, 1, 0, 0.1, 1500, true, 7);
  gate.set(true);
  driveToWall(7, 1000, 1000, true, 12);
  thread([]{
    doinkerup(0);
    doinker(167);
  });
  driveTo(-4, 1000, true, 12);
  wait(50, msec);
  resetPositionFront(Rwall_distance_sensor, 4.125, 5, 70.25);
  resetPositionRight(rightSide, 6.375, 0, 70.25);
  wait(50, msec);
  boomerang(58, 20, -1, 0, 0.3, 1100, false, 11);
  moveToPoint(57, -48, -1, 2200, true, 12);
  wait(100, msec);
  turnToAngle(270, 800);
  wait(100, msec);
  driveFromWall(15, 1200, 0, true, 12);
  wait(100, msec);
  turnToAngle(180, 800);
  wait(100, msec);
  driveToWall(34.75, 1000, 0, true, 12);
  gate.set(false);
  stick.spin(fwd, 35, percent);
  wait(600, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(600, msec);
  thread([]{reset();});
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  resetPositionFront(Rwall_distance_sensor, 4.125, 5, 70.25);
  resetPositionLeft(leftSide, 6.375, 0, 70.25);
  correct_angle = 182;
  driveTo(28, 2000, true, 6);
  gate.set(true);
  driveToWall(7, 1000, 750, true, 12);
  turnToAngle(180, 500);  
  thread([]{
    doinkerup(0);
    doinker(167);
  });
  moveToPrevPos();
  driveToWall(34.75, 500, 0, true, 12);
  gate.set(false);
  stick.spin(fwd, 20, percent);
  wait(700, msec);
  stick.spin(fwd, 10, percent);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(1050, msec);
  scraper.set(false);
  thread([]{reset();});
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  driveTo(18, 1500, true, 10);
  turnToAngle(-90, 800);
  driveTo(6, 1500, true, 10);
  wait(50, msec);
  resetPositionBack(backSide, 9, 0, 70.25);
  resetPositionLeft(leftSide, 0, 6.375, 70.25);
  wait(50, msec);
  thread([]{
    doinker(1100);
    doinkerup(300);
  });
  boomerang(7, -9, 1, 315, 0.4, 2000, true, 6);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(1000, msec);
  mid_goal.set(false);
  boomerang(40, -50, -1, 270, 0.3, 2500, true, 12);
  turnToAngle(270, 400);
  resetPositionBack(backSide, 9, 0, 70.25);
  resetPositionLeft(leftSide, 0, 6.375, 70.25);
  boomerang(19, -63, 1, -90, 0.2, 2000, false, 12);
  turnToAngle(270, 300);
  resetPositionLeft(leftSide, 0, 6.375, 70.25);
  scraper.set(true);
  wait(50, msec);
  thread([]{doinkerup(700);});
  boomerang(-24.67420, -65, 1, -90, 0.1, 5000, true, 7);
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
  driveTo(-27, 10000, true, 7);
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
  boomerang(27, 25, 1, 90, 0.4, 1500, false, 10); 
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
  stick.spin(fwd, 30, percent);
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
  wait(1, msec);
  driveTo(-27, 10000, true, 7);//12
  lower_intake.spin(forward, 1, voltageUnits::volt);
  stick.spin(reverse, 12, voltageUnits::volt);
  lower_intake.stop();

}