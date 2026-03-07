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
  moveToPoint(45, -50, -1, 800, false, 11);
  scraper.set(true);
  turnToAngle(180, 800);
  driveToWall(7, 900, 0, true, 12);
  wait(50, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(50, msec);
  moveToPoint(51.5, -21, -1, 800, false, 11);
  wings.set(false);
  scraper.set(false);
  wait(1300, msec);
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
  boomerang(-46, -48, 1, 180, 0.2, 1000, true, 9);
  turnToAngle(180, 500, true, 12);
  wait(50, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionRight(rightSide, 3.375, 1, 72);
  wait(50, msec);
  moveToPoint(-48, -56, 1, 800, false, 12);
  driveToWall(7, 800, 0, true, 12);
  wait(50, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionRight(rightSide, 3.375, 1, 72);
  wait(50, msec);
  moveToPoint(-50, -22, -1, 800, false, 11);
  wings.set(false);
  scraper.set(false);
  wait(750, msec);
  upper_intake.stop();
  driveTo(3, 200, false, 12);
  wings.set(true);
  swing(-120, 1, 1000, false, 10);
  upper_intake.spin(fwd, 9, voltageUnits::volt);
  boomerang(-5, -2, -1, 225, 0.3, 1200, 12);
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
  wait(500, msec);
  driveChassis(8, 8);
  wait(200, msec);
  driveChassis(0, 0);
  wait(500, msec);
  driveTo(-2, 1700);
  wait(400, msec);
  driveChassis(12, 12);
  wait(700, msec);
  driveChassis(0, 0);
  wait(650, msec);
  driveTo(-29, 1700);
  upper_intake.stop();
  wait(100, msec);

  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(100, msec);
  turnToAngle(45, 800);
  descore.set(false);
  moveToPoint(10, -19.5, 1, 1500, true, 8);
  descore.set(true);
  turnToAngle(-45, 800);
  descore.set(false);
  driveTo(15, 1000, true, 10);
  scraper.set(true);
  wait(100, msec);
  intake_lift.set(false);
  lower_intake.spin(fwd, -9, voltageUnits::volt);
  upper_intake.spin(fwd, -10, voltageUnits::volt);
  wait(700, msec);
  upper_intake.spin(reverse, 9, voltageUnits::volt);
  lower_intake.spin(reverse, 6.2, voltageUnits::volt);
  wait(1600, msec);
  lower_intake.spin(reverse, 4, voltageUnits::volt);
  driveTo(2, 800, true, 2);
  scraper.set(false);
  wait(150, msec);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  driveTo(-17, 1600, true, 12);
  turnToAngle(-90, 800);
  wait(10, msec);
  resetPositionBack(backSide, 3.375, 1, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(10, msec);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  intake_lift.set(true);
  boomerang(-38, -48, 1, 180, 0.1, 2000, true, 10.5);
  turnToAngle(180, 200);
  scraper.set(true);
  wait(100, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionRight(rightSide, 3.375, 1, 72);
  wait(100, msec);
  moveToPoint(-49.5, -55, 1, 800, true, 8);
  turnToAngle(180, 300);
  driveToWall(7.1, 1900, 0, true, 12);
  boomerang(-60, -26, -1, 180, 0.3, 1000, true, 11);
  turnToAngle(180, 200);
  scraper.set(false);
  upper_intake.stop();
  lower_intake.stop();
  moveToPoint(-58, 41, -1, 2000, true, 10);
  wait(50, msec);
  turnToAngle(-90, 500);
  driveToWallRight(16.5, 800, 0, true, 12);
  turnToAngle(0, 500);
  thread([]{
    upper_intake.spin(fwd, -12, voltageUnits::volt);
    wait(400, msec);
    upper_intake.stop(hold);
  });
  driveTo(-100, 700, true, 12);
  wings.set(false);
  descore.set(true);
  wait(100, msec);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wait(1700, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  scraper.set(true);
  descore.set(false);
  upper_intake.stop();
  moveToPoint(-48, 60, 1, 1000, true, 12);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wings.set(true);
  driveToWall(8, 1400, 0, true, 12);
  turnToAngle(0, 300);
  wait(100, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(100, msec);
  moveToPoint(-50.25, 22, -1, 1000, false, 11);
  descore.set(true);
  upper_intake.spin(fwd, -12, voltageUnits::volt);
  wings.set(false);
  wait(100, msec);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wait(1500, msec);
  scraper.set(false);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  intake_lift.set(true);
  descore.set(true);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  boomerang(-22, 63, 1, 90, 0.3, 2000, true, 11);
  turnToAngle(90, 300);
  correct_angle = 90;
  driveTo(8, 800, true, 12);
  turnToAngle(90, 500);
  descore.set(true);
  wings.set(true);
  driveChassis(6.5, 6.5);
  wait(800, msec);
  driveChassis(0, 0);
  wait(500, msec);
  driveChassis(8.5, 8.5);
  wait(900, msec);
  scraper.set(true);
  driveChassis(0, 0);
  wait(150, msec);
  driveToWallRight(46, 900, 0, true, 12);
  turnToAngle(183, 800);
  descore.set(false);
  scraper.set(false);
  correct_angle = 180;
  driveTo(5, 500, true, 12);
  wait(10, msec);
  resetPositionBack(backSide, 3.375, 1, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(10, msec);
  driveFromWall(47.5, 1300, 0, true, 8);
  lower_intake.stop();
  descore.set(true);
  turnToAngle(45, 700, true, 10);
  descore.set(false);
  lower_intake.stop();
  upper_intake.stop();
  correct_angle = 45;
  thread([]{
    upper_intake.spin(fwd, -12, voltageUnits::volt);
    wait(400, msec);
    upper_intake.stop(hold);
  });
  driveTo(-17.4, 1000, true, 5);
  lower_intake.spin(fwd, 11, voltageUnits::volt);
  mid_goal.set(true);
  wait(120, msec);
  upper_intake.spin(fwd, 50, pct);
  wait(700, msec);
  upper_intake.spin(fwd, 40, pct);
  wait(800, msec);
  driveTo(1.5, 200, true, 4);
  upper_intake.spin(fwd, 30, pct);
  wait(1700, msec);
  upper_intake.stop();
  driveTo(-2, 400, true, 2.5);
  thread([]{
    wait(200, msec);
    upper_intake.spin(fwd, 12, voltageUnits::volt);
  });
  boomerang(35.5, 51, 1, 0, 0.1, 2000, true, 11);
  mid_goal.set(false);
  scraper.set(true);
  turnToAngle(0, 800);
  wait(50, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionRight(rightSide, 3.375, 1, 72);
  wait(50, msec);
  moveToPoint(50, 61, 1, 800, false, 12);
  driveToWall(8.2, 800, 600, true, 12);
  wait(50, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionRight(rightSide, 3.375, 1, 72);
  wait(50, msec);
  boomerang(60, 26, -1, 0, 0.3, 1200, false, 10);
  scraper.set(false);
  moveToPoint(59, -39, -1, 1400, true, 11.5);
  wait(100, msec);
  turnToAngle(90, 800);
  driveToWallRight(16.4, 800, 0, true, 12);
  turnToAngle(180, 500);
  thread([]{
    upper_intake.spin(fwd, -12, voltageUnits::volt);
    wait(200, msec);
    upper_intake.stop(hold);
  });
  driveTo(-18, 1100, true, 12);
  wings.set(false);
  wait(100, msec);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  wait(1700, msec);
  scraper.set(true);
  upper_intake.stop();
  moveToPoint(49, -60, 1, 1000, true, 12);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wings.set(true);
  driveToWall(8, 800, 700, true, 12);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  thread([]{
    upper_intake.spin(fwd, -12, voltageUnits::volt);
    wait(300, msec);
    upper_intake.stop(hold);
  });
  moveToPoint(49.5, -20, -1, 1000, false, 12);
  wings.set(false);
  wait(100, msec);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wait(1600, msec);
  resetPositionFront(frontsensor, 2.875, 7.5, 72);
  resetPositionLeft(leftSide, 3.375, 1, 72);
  scraper.set(false);
  descore.set(true);
  boomerang(11, -66, 1, -90, 0.3, 2000, false, 12);
  correct_angle = -90;
  driveTo(25, 20000, true, 12);


}

void elimleft(){
  resetChassis();
  wings.set(true);
  intake_lift.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  thread([]{
    doinker(400);
  });
  boomerang(-7, 19, 1, -50, 0.44, 2000, false, 10.5);

  //4 Ball 
  
  turnToPoint(-19, -3, 1, 400);
  boomerang(-19, -3, 1, 180, 0.44, 2000, false, 11);
  turnToAngle(180, 200);
  driveTo(-80, 500, true, 12);
  wings.set(false);
  wait(900, msec);

  //7 Ball 
  /*
  turnToPoint(-16.5, -15, 1, 700);
  boomerang(-16.5, -15, 1, 180, 0.44, 2000, false, 11);
  turnToAngle(180, 200);
  driveToWall(7, 700, 0, true, 12);
  correct_angle = 179;
  driveTo(-35, 900, true, 12);
  wings.set(false);
  wait(1200, msec);*/


  //Ending
  scraper.set(false);
  correct_angle = 145;
  driveTo(19.5, 1200, true, 12);
  turnToAngle(180, 400);
  correct_angle = 180;
  driveTo(-35, 1000000, true, 12);
  
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