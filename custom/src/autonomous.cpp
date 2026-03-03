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
  inertial_sensor.setRotation(-90, degrees);
  correct_angle = -90;
  wings.set(true);
  intake_lift.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  driveTo(6, 700);
  wait(50, msec);
  resetPositionBack(backSide, 3.375, 1, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(50, msec);
  moveToPoint(44.5, -50, -1, 800, false, 11);
  scraper.set(true);
  turnToAngle(180, 800);
  driveToWall(7, 900, 0, true, 12);
  wait(50, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(50, msec);
  moveToPoint(50, -22, -1, 800, true, 11);
  wings.set(false);
  scraper.set(false);
  wait(1100, msec);
  upper_intake.stop();
  driveTo(3, 300, false, 12);
  wings.set(true);
  swing(330, 1, 900, true, 10);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  thread([]{
    doinker(400);
    doinkerup(200);
  });
  driveTo(7, 550, false, 12);
  boomerang(-15, -21, 1, -90, 0.45, 1500, false, 9);
  scraper.set(true);
  boomerang(-50, -45, 1, 180, 0.3, 1000, true, 9);
  turnToAngle(180, 500, true, 12);
  wait(50, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionRight(rightSide, 3.375, 1, 72);
  wait(50, msec);
  moveToPoint(-49, -56, 1, 800, false, 12);
  driveToWall(7, 800, 0, true, 12);
  wait(50, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionRight(rightSide, 3.375, 1, 72);
  wait(50, msec);
  moveToPoint(-50, -22, -1, 800, true, 11);
  wings.set(false);
  scraper.set(false);
  wait(750, msec);
  upper_intake.stop();
  driveTo(3, 200, false, 12);
  wings.set(true);
  swing(-120, 1, 1000, false, 10);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  boomerang(-7, 1, -1, 225, 0.3, 1200, 12);
  mid_goal.set(true);

}

void qualsoloawp(){

}

void rightsidequal(){
  resetChassis();
  wings.set(true);
  intake_lift.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  thread([]{
    doinker(400);
    doinkerup(200);
    doinker(400);
  });
  boomerang(34, 43.50, 1, 90, 0.44, 2000, false, 10.5);
  swing(90, 1, 100, true, 12);
  wait(100, msec);
  swing(0, -1, 700, true, 10);
  driveTo(-8, 800, false, 12);
  curveCircle(181, 11.25, 1000, false, 11);
  driveTo(-1000, 600, true, 12);
  wings.set(false);
  wait(100, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(1000, msec);
  scraper.set(true);
  thread([]{
    wait(70, msec);
    wings.set(true);
  });
  moveToPoint(47.5, -60, 1, 1000, true, 12);
  driveToWall(7, 550, 0, true, 12);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  upper_intake.stop();
  driveTo(-8, 800, true, 12);
  descore.set(true);
  lower_intake.stop();
  turnToAngle(-45, 800);
  descore.set(false);
  scraper.set(false);
  moveToPoint(5.5, -17, 1, 1600, true, 11);
  intake_lift.set(false);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  upper_intake.spin(reverse, 12, voltageUnits::volt);
  wait(900, msec);
  lower_intake.stop();
  upper_intake.stop();
  boomerang(27.5, -40, -1, 0, 0.44, 2000, true, 11);
  intake_lift.set(true);
  turnToAngle(0, 200);
  correct_angle = 0;
  wings.set(false);
  driveTo(25, 10000000000, true, 12);
  
  
}

void leftsidequal(){
  resetChassis();
  wings.set(true);
  intake_lift.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  thread([]{
    doinker(360);
    doinkerup(200);
    doinker(500);
  });
  boomerang(-23.5, 33.5, 1, -90, 0.44, 2000, false, 10.5);
  swing(-90, 1, 100, true, 12);
  wait(100, msec);
  swing(0, -1, 700, true, 10);
  correct_angle = 0;
  driveTo(-8, 800, false, 12);
  curveCircle(179, -11.25, 1000, false, 11);
  driveTo(-1000, 400, true, 12);
  wings.set(false);
  wait(100, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionRight(rightSide, 3.375, 1, 72);
  wait(1000, msec);
  scraper.set(true);
  thread([]{
    wait(70, msec);
    wings.set(true);
  });
  moveToPoint(-49.5, -60, 1, 1000, true, 12);
  driveToWall(7, 550, 0, true, 12);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionRight(rightSide, 3.375, 1, 72);
  boomerang(-8, -12, -1, 225, 0.2, 1600, true, 11);
  scraper.set(false);
  mid_goal.set(true);
  wait(900, msec);
  mid_goal.set(false);
  wings.set(true);
  lower_intake.stop();
  upper_intake.stop();
  boomerang(-37, -42, 1, 180, 0.44, 2000, true, 11);
  turnToAngle(180, 200);
  wings.set(false);
  correct_angle = 180;
  driveTo(-22.5, 10000000000, true, 12);


}

void skills() { 
  resetChassis();
  inertial_sensor.setRotation(180, degrees);
  correct_angle = 180;
  wings.set(true);
  descore.set(true);
  intake_lift.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wait(80, msec);
  driveChassis(8, 8);
  wait(200, msec);
  driveChassis(0, 0);
  wait(500, msec);
  driveTo(-2, 1700);
  wait(500, msec);
  driveChassis(12, 12);
  wait(700, msec);
  driveChassis(0, 0);
  wait(500, msec);
  driveTo(-29, 1700);
  upper_intake.stop();
  wait(100, msec);

  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(100, msec);
  turnToAngle(45, 800);
  descore.set(false);
  moveToPoint(15.5, -19.5, 1, 1500, true, 8);
  descore.set(true);
  turnToAngle(-45, 800);
  descore.set(false);
  driveTo(19, 1000, true, 10);
  intake_lift.set(false);
  lower_intake.spin(fwd, -11, voltageUnits::volt);
  upper_intake.spin(fwd, -9.5, voltageUnits::volt);
  wait(700, msec);
  upper_intake.spin(reverse, 7.2, voltageUnits::volt);
  lower_intake.spin(reverse, 6.2, voltageUnits::volt);
  wait(2000, msec);
  lower_intake.spin(reverse, 3, voltageUnits::volt);
  driveTo(2, 800, true, 2);
  lower_intake.spin(reverse, 3.6, voltageUnits::volt);
  wait(100, msec);
  driveTo(-17, 1600, true, 12);
  turnToAngle(-90, 800);
  /*wait(50, msec);
  resetPositionBack(backSide, 3.375, 1, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(50, msec);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  intake_lift.set(true);
  boomerang(-42.5, -48, 1, 180, 0.1, 2000, true, 10.5);
  scraper.set(true);
  turnToAngle(180, 500);
  wait(100, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionRight(rightSide, 3.375, 1, 72);
  wait(100, msec);
  moveToPoint(-49, -55, 1, 800, true, 8);
  turnToAngle(180, 700);
  driveToWall(7.4, 800, 1100, true, 12);
  driveTo(-3, 900, false, 8);
  boomerang(-62, -26, -1, 180, 0.3, 1000, false, 11);
  scraper.set(false);
  upper_intake.stop();
  lower_intake.stop();
  moveToPoint(-60, 39, -1, 2000, true, 10);
  wait(100, msec);
  turnToAngle(-90, 500);
  driveToWall(17, 800, 0, true, 12);
  turnToAngle(0, 500);
  driveTo(-100, 1100, true, 12);
  wings.set(false);
  wait(100, msec);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wait(2300, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  scraper.set(true);
  upper_intake.stop();
  moveToPoint(-48, 60, 1, 1000, true, 12);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wings.set(true);
  driveToWall(7.4, 900, 1000, true, 12);
  wait(100, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(100, msec);
  correct_angle = 2;
  driveTo(-30, 1000, true, 12);
  wings.set(false);
  wait(200, msec);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wait(2000, msec);
  scraper.set(false);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  intake_lift.set(true);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  boomerang(-17, 71, 1, 90, 0.4, 2000, true, 11);
  turnToAngle(90, 500);
  descore.set(true);
  wings.set(true);
  driveChassis(5.75, 5.75);
  wait(1000, msec);
  driveChassis(0, 0);
  wait(600, msec);
  driveChassis(8, 8);
  wait(800, msec);
  scraper.set(true);
  driveChassis(0, 0);
  wait(100, msec);
  driveToWall(15, 800, 0, true, 12);
  swing(180, 1, 1000, true, 12);
  descore.set(false);
  driveTo(6, 300, true, 12);
  scraper.set(false);
  wait(100, msec);
  resetPositionBack(backSide, 3.375, 1, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(100, msec);


  boomerang(21, 28, 1, 180, 0.2 , 2000, true, 9);
  swing(45, 1, 1000, true, 7);
  lower_intake.stop();
  upper_intake.stop();
  correct_angle = 45;
  driveTo(-19, 1000, true, 10);
  lower_intake.spin(fwd, 11, voltageUnits::volt);
  upper_intake.spin(fwd, 46, pct);
  mid_goal.set(true);
  wait(2500, msec);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  boomerang(39, 52, 1, 0, 0.1, 2000, true, 11);
  mid_goal.set(false);
  turnToAngle(0, 800);
  scraper.set(true);
  wait(50, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionRight(rightSide, 3.375, 1, 72);
  wait(50, msec);
  moveToPoint(49.75, 60, 1, 800, false, 12);
  driveToWall(7.4, 800, 1000, true, 12);
  wait(50, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionRight(rightSide, 3.375, 1, 72);
  wait(50, msec);
  boomerang(62, 26, -1, 0, 0.3, 1200, false, 10);
  scraper.set(false);
  boomerang(64, -36, -1, 190, 0.1, 2000, true, 11);
  wait(100, msec);
  turnToAngle(90, 800);
  driveToWall(17, 800, 0, true, 12);
  turnToAngle(180, 500);
  driveTo(-18, 900, true, 12);
  wings.set(false);
  wait(100, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(2300, msec);
  scraper.set(true);
  upper_intake.stop();
  moveToPoint(48.5, -60, 1, 1000, true, 12);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wings.set(true);
  driveToWall(7.4, 900, 1000, true, 12);
  wait(100, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(100, msec);
  correct_angle = 182;
  driveTo(-30, 1000, true, 12);
  //moveToPoint(50.25, -20, -1, 1000, true, 12);
  wings.set(false);
  wait(100, msec);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wait(2000, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  scraper.set(false);
  boomerang(20, -69, 1, -90, 0.5, 2000, true, 11);
  turnToAngle(-90, 500);
  descore.set(true);
  driveTo(34, 20000, true, 12);*/


}

void elimleft(){

  
}

void elimright(){
  resetChassis();
  wings.set(true);
  intake_lift.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  thread([]{
    doinker(400);
  });
  boomerang(8, 26, 1, 50, 0.44, 2000, false, 10.5);

  //4 Ball 
  /*
  turnToPoint(37.5, 2, 1, 400);
  boomerang(37.5, 2, 1, 180, 0.44, 2000, false, 11);
  turnToAngle(180, 200);
  driveTo(-80, 500, true, 12);
  wings.set(false);
  wait(900, msec);*/

  //7 Ball 
  
  turnToPoint(39, -15, 1, 600);
  boomerang(39, -15, 1, 180, 0.44, 2000, false, 11);
  turnToAngle(180, 200);
  driveToWall(7, 700, 0, true, 12);
  correct_angle = 182;
  driveTo(-35, 900, true, 12);
  wings.set(false);
  wait(1200, msec);


  //Ending
  scraper.set(false);
  correct_angle = 145;
  driveTo(19.5, 1200, true, 12);
  turnToAngle(180, 400);
  correct_angle = 180;
  driveTo(-35, 1000000, true, 12);

}