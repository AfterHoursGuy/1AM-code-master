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
bool enableLeft = false;
bool enableRight = false;
bool enableBack = false;
bool enableFront = false;
bool enableFR = false;

void doinker(int t){
  wait(t, msec);
  scraper.set(true);
}

void doinkerup(int t){
  wait(t, msec);
  scraper.set(false);
} 

int distanceTrackingTask() {
    while (true) {
        // Run your logic using the global switches
        trackdistanceodom(enableLeft, enableRight, enableBack, enableFront, enableFR);

        // Sleep for 10ms to prevent CPU hogging
        vex::task::sleep(10); 
    }
    return 0;
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
  vex::task t(distanceTrackingTask);
  enableFR = true;
  enableLeft = true;
  
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
  wait(10, msec);
  resetPositionBack(90);
  resetPositionLeft(180);
  wait(10, msec);
  moveToPoint(37.5, -55, -1, 1000, true, 11);
  scraper.set(true);
  turnToAngle(180, 500);
  driveToWall(6.5, 800, 0, true, 12);
  resetPositionFront(frontsensor, -2.875, 7.5, 180);
  resetPositionLeft(90);
  moveToPoint(58.5, -21, -1, 800, false, 11);
  wings.set(false);
  scraper.set(false);
  wait(1000, msec);
  upper_intake.stop();
  correct_angle = 200;
  driveTo(6, 600, true, 12);
  wings.set(true);
  turnToAngle(-60, 700);
  resetPositionBack(90);
  resetPositionLeft(180);
  thread([]{
    doinker(600);
    doinkerup(200);
  });
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  boomerang(24, -36, 1, 274, 0.3, 2000, true, 10);
  resetPositionBack(90);
  resetPositionLeft(180);
  boomerang(-15, -38, 1, 250, 0.3, 2000, false, 10, true);
  scraper.set(true);
  boomerang(-21, -65, 1, 180, 0.3, 2000, true, 10);
  driveToWall(7, 700, 0, true, 12);
  resetPositionFront(frontsensor, -2.875, 7.5, 270);
  resetPositionRight(270);
  wait(50, msec);
  moveToPoint(-48, -19, -1, 800, false, 11);
  resetPositionFront(frontsensor, -2.875, 7.5, 270);
  resetPositionRight(270);
  wings.set(false);
  scraper.set(false);
  wait(800, msec);
  upper_intake.stop();
  correct_angle = 160;
  driveTo(11.5, 800, true, 12);
  turnToAngle(225, 500);
  driveTo(-40, 900, true, 12);
  upper_intake.spin(fwd, 80, pct);
  mid_goal.set(true);




  


  
}

void qualsoloawp(){

}

void rightsidequal(){
  // Early mid, late wing
    /*
  resetChassis();
  wings.set(true);
  intake_lift.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  thread([]{
    doinker(460);
  });
  boomerang(9, 26, 1, 30, 0.4, 2000, true, 10.5);
  turnToAngle(-90, 800);
  scraper.set(false);
  upper_intake.stop();
  boomerang(-4.75, 24, 1, 315, 0.4, 1000, true, 9);
  turnToAngle(315, 500, false, 12);
  driveTo(5, 600, true, 12);
  scraper.set(true);
  intake_lift.set(false);
  upper_intake.spin(fwd, -12, voltageUnits::volt);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(650, msec);
  scraper.set(false);
  intake_lift.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  moveToPoint(25, -6, -1, 1200, false, 10);
  scraper.set(true);
  turnToAngle(180, 800);
  driveToWall(6, 1000, 0, true, 12);
  correct_angle = 183;
  driveTo(-80, 800, false, 12);
  wings.set(false);
  wait(600, msec);
  scraper.set(false);
  correct_angle = 220;
  driveTo(9, 600, true, 10);
  turnToAngle(0, 600);
  correct_angle = 0;
  driveTo(26.5, 100000, true, 12);
  // */

 // early wing, late mid
 // /*
 resetChassis();
  wings.set(true);
  intake_lift.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  thread([]{
    doinker(400);
  });
  boomerang(10, 26, 1, 50, 0.44, 2000, false, 10.5);
  turnToPoint(33, -15, 1, 600);
  boomerang(32.25, -15, 1, 180, 0.4, 2000, true, 11);
  turnToAngle(180, 200);
  driveToWall(6.5, 700, 0, true, 12);
  correct_angle = 182;
  driveTo(-35, 800, false, 12);
  wings.set(false);
  wait(700, msec);
  upper_intake.spin(fwd, -12, voltageUnits::volt);
  wait(80, msec);
  upper_intake.stop();
  scraper.set(false);
  driveTo(3, 600, false, 10);
  swing(359, 1, 1200, false, 9.2);
  correct_angle = 4;
  driveTo(27, 100000, true, 12);
  turnToAngle(-8, 700);
  wait(4000, msec);
  resetPositionBack(180);
  resetPositionRight(90);
  wings.set(true);
  moveToPoint(35, -27, -1, 1000, true, 12);
  resetPositionBack(180);
  resetPositionRight(90);
  turnToAngle(-45, 400);
  boomerang(10, -17, 1, -45, 0.2, 1000, true, 12);
  scraper.set(true);
  intake_lift.set(false);
  upper_intake.spin(fwd, -12, voltageUnits::volt);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(800, msec);
  scraper.set(false);
  wings.set(false);
  boomerang(37, -38, -1, 0, 0.4, 1500, true, 12);
  // */
  

}

void leftsidequal(){
  // early wing late mid
  /*
  resetChassis();
  wings.set(true);
  intake_lift.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  thread([]{
    doinker(500);
  });
  boomerang(-7, 19, 1, -50, 0.44, 2000, true, 10.5);
  turnToPoint(-15, -15, 1, 700);
  boomerang(-15, -15, 1, 180, 0.44, 2000, false, 11);
  turnToAngle(180, 200);
  driveToWall(7.5, 700, 0, true, 12);
  correct_angle = 180;
  driveTo(-35, 800, false, 12);
  wings.set(false);
  wait(700, msec);
  upper_intake.spin(fwd, -12, voltageUnits::volt);
  wait(80, msec);
  upper_intake.stop();
  scraper.set(false);
  correct_angle = 145;
  driveTo(14, 1200, true, 12);
  turnToAngle(180, 400);
  correct_angle = 178;
  driveTo(-26, 1000000, true, 12);
  wait(4000, msec);
  wings.set(true);
  resetPositionFront(frontsensor, -2.875, 7.5, 180);
  resetPositionRight(270);
  moveToPoint(-30, -18, 1, 1000, true, 12);
  resetPositionFront(frontsensor, -2.875, 7.5, 180);
  resetPositionRight(270);
  turnToAngle(225, 400);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  boomerang(-9, -7.5, -1, 45, 0.1, 1400, 12);
  mid_goal.set(true);
  wait(600, msec);
  correct_angle = 225;
  driveTo(8, 600, true, 12);
  descore.set(true);
  driveChassis(-12, -12);
  wait(700, msec);
  driveChassis(0, 0);
  // */

  // early mid, late wing
  resetChassis();
  wings.set(true);
  intake_lift.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  thread([]{
    doinker(500);
  });
  boomerang(-7, 19, 1, -50, 0.44, 2000, true, 10.5);
  turnToAngle(-90, 500);
  boomerang(12, 25, -1, 45, 0.4, 1000, true, 9);
  mid_goal.set(true);
  upper_intake.spin(fwd, 60, pct);
  wait(750, msec);
  boomerang(-18, -14, 1, 180, 0.2, 2000, true, 10.5);
  mid_goal.set(false);
  turnToAngle(180, 200);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  driveToWall(7, 800, 0, true, 12);
  correct_angle = 178;
  driveTo(-35, 800, false, 12);
  wings.set(false);
  wait(900, msec);scraper.set(false);
  correct_angle = 145;
  driveTo(14, 1200, true, 12);
  turnToAngle(180, 400);
  correct_angle = 178;
  driveTo(-26, 1000000, true, 6);


  


}

void skills() { 

  resetChassis();
  inertial_sensor.setRotation(180, degrees);
  correct_angle = 180;

  vex::task t(distanceTrackingTask);


   // /*
  
  wings.set(true);
  descore.set(true);
  intake_lift.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wait(200, msec);
  driveChassis(8, 8);
  wait(200, msec);
  driveChassis(0, 0);
  wait(500, msec);
  driveTo(-2, 1700);
  wait(400, msec);
  driveChassis(12, 12);
  wait(500, msec);
  driveChassis(0, 0);
  wait(650, msec);
  driveTo(-25, 1700);
  upper_intake.stop();
  wait(10, msec);
  turnToAngle(90, 800);
  resetPositionFront(rightfront, 2.875, 7.5, 90);
  resetPositionRight(180);
  descore.set(false);
  moveToPoint(9, -30, 1, 1500, true, 8);
  lower_intake.stop();
  descore.set(true);
  turnToAngle(0, 500, true, 12);
  resetPositionBack(180);
  resetPositionRight(90);
  turnToAngle(-45, 400);
  moveToPoint(12, -18, 1, 1000, true, 10);
  turnToAngle(-45, 800);
  scraper.set(true);
  wait(100, msec);
  intake_lift.set(false);
  lower_intake.spin(fwd, -7, voltageUnits::volt);
  upper_intake.spin(fwd, -8, voltageUnits::volt);
  wait(500, msec);
  upper_intake.spin(reverse, 7, voltageUnits::volt);
  lower_intake.spin(reverse, 4, voltageUnits::volt);
  wait(1700, msec);
  scraper.set(false);
  lower_intake.spin(reverse, 9.5, voltageUnits::volt);
  driveTo(-17, 1600, true, 8);
  intake_lift.set(true);
  turnToAngle(-90, 800);
  wait(10, msec);
  resetPositionBack(90);
  resetPositionLeft(180);
  wait(10, msec);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  boomerang(-12.5, -56, 1, 180, 0.53, 2000, true, 10.5);
  turnToAngle(180, 200);
  scraper.set(true);
  wait(100, msec);
  resetPositionFront(frontsensor, -2.875, 7.5, 180);
  resetPositionRight(270);
  wait(100, msec);
  moveToPoint(-47, -55, 1, 800, true, 8);
  turnToAngle(180, 300);
  driveToWall(7, 1300, 0, true, 12);

  boomerang(-60, -23, -1, 180, 0.3, 1000, true, 11);
  turnToAngle(180, 200);
  scraper.set(false);
  upper_intake.stop();
  lower_intake.stop();
  moveToPoint(-54.5, 20, -1, 2000, true, 10);
  wait(50, msec);
  turnToAngle(-90, 500);
  resetPositionFront(frontsensor, -2.875, 7.5, 270);
  resetPositionRight(0);
  moveToPoint(-48, 28, -1, 1000, true, 10);
  turnToAngle(0, 500);
  thread([]{
    upper_intake.spin(fwd, -12, voltageUnits::volt);
    wait(400, msec);
    upper_intake.stop(hold);
  });
  driveTo(-100, 700, false, 10);
  wings.set(false);
  descore.set(true);
  wait(100, msec);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  turnToAngle(0, 500, true, 12);
  wait(1700, msec);
  resetPositionFront(frontsensor, -2.875, 7.5, 0);
  resetPositionLeft(270);
  scraper.set(true);
  descore.set(false);
  upper_intake.stop();
  moveToPoint(-53, 52, 1, 1000, true, 12);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wings.set(true);
  driveToWall(7.5, 1400, 0, true, 12);

  turnToAngle(0, 300);
  wait(100, msec);
  resetPositionFront(frontsensor, -2.875, 7.5, 0);
  resetPositionLeft(270);
  wait(100, msec);
  moveToPoint(-56.5, 22, -1, 1000, false, 11);
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
  resetPositionFront(frontsensor, -2.875, 7.5, 0);
  resetPositionLeft(270);
  boomerang(-40, 53.5, 1, 85, 0.3, 2000, true, 11);
  boomerang(-24.5, 53.5, 1, 90, 0.1, 2000, true, 11);
  turnToAngle(90, 500);
  wait(75, msec);
  descore.set(false);
  wings.set(true);
  driveChassis(7, 7);
  wait(800, msec);
  driveChassis(0, 0);
  wait(500, msec);
  driveChassis(8.5, 8.5);
  wait(950, msec);
  scraper.set(true);
  driveChassis(0, 0);
  wait(150, msec);
  turnToAngle(90, 300);
  resetPositionLeft(0);
  resetPositionFront(rightfront, 2.875, 7.5, 90);
  moveToPoint(24, 64, -1, 1200, true, 10);
  turnToAngle(180, 800);
  descore.set(false);
  scraper.set(false);
  resetPositionBack(0);
  resetPositionLeft(90);
  moveToPoint(22, 33, 1, 1000, true, 7);
  turnToAngle(180, 300);
  resetPositionBack(0);
  resetPositionLeft(90);
  lower_intake.stop();
  descore.set(true);
  turnToAngle(45, 700, true, 10);

  scraper.set(true);
  descore.set(false);
  lower_intake.stop();
  upper_intake.stop();
  thread([]{
    upper_intake.spin(fwd, -12, voltageUnits::volt);
    wait(400, msec);
    upper_intake.stop(hold);
  });
  moveToPoint(8, 16, -1, 1000, true, 7);
  turnToAngle(45, 500);
  lower_intake.spin(fwd, 11, voltageUnits::volt);
  mid_goal.set(true);
  wait(60, msec);
  upper_intake.spin(fwd, 40, pct);
  wait(1500, msec);
  driveTo(1.5, 200, true, 4);
  upper_intake.spin(fwd, 30, pct);
  wait(1500, msec);
  upper_intake.stop();
  driveTo(-1, 400, true, 2.5);
  thread([]{
    wait(200, msec);
    upper_intake.spin(fwd, 12, voltageUnits::volt);
  });
  boomerang(38, 55, 1, 0, 0.1, 2000, true, 11);
  mid_goal.set(false);
  scraper.set(true);
  turnToAngle(0, 800);
  wait(50, msec);
  resetPositionFront(frontsensor, -2.875, 7.5, 0);
  resetPositionRight(90);
  wait(50, msec);
  moveToPoint(48, 61, 1, 800, false, 12);
  driveToWall(8, 1300, 0, true, 12);


  wait(50, msec);
  resetPositionFront(frontsensor, -2.875, 7.5, 0);
  resetPositionRight(90);
  wait(50, msec);
  boomerang(60, 26, -1, 0, 0.3, 1200, false, 10);
  scraper.set(false);
  moveToPoint(55, -20, -1, 1400, true, 11.5);
  wait(100, msec);
  turnToAngle(90, 500);
  driveToWallRight(18.5, 800, 0, true, 12);
  turnToAngle(180, 500);
  thread([]{
    upper_intake.spin(fwd, -12, voltageUnits::volt);
    wait(200, msec);
    upper_intake.stop(hold);
  });
  driveTo(-18, 1100, false, 12);
  wings.set(false);
  wait(100, msec);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  resetPositionFront(frontsensor, -2.875, 7.5, 180);
  resetPositionLeft(90);
  wait(1700, msec);
  scraper.set(true);
  upper_intake.stop();
  moveToPoint(55, -52, 1, 1000, true, 12);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wings.set(true);
  driveToWall(8, 1300, 0, true, 12);
  resetPositionFront(frontsensor, -2.875, 7.5, 180);
  resetPositionLeft(90);
  thread([]{
    upper_intake.spin(fwd, -12, voltageUnits::volt);
    wait(300, msec);
    upper_intake.stop(hold);
  });
  moveToPoint(54, -24, -1, 1000, false, 12);
  wings.set(false);
  wait(100, msec);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wait(1600, msec);
  resetPositionFront(frontsensor, -2.875, 7.5, 180);
  resetPositionLeft(90);
  scraper.set(false);
  descore.set(true);
  boomerang(27, -56, 1, -90, 0.3, 2000, false, 12);
  correct_angle = -90;
  driveTo(20, 20000, true, 12); 
  // */

  /*resetChassis();
  inertial_sensor.setRotation(180, degrees);
  correct_angle = 180;

  vex::task t(distanceTrackingTask);


   // /*
  
  wings.set(true);
  descore.set(true);
  intake_lift.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wait(200, msec);
  driveChassis(8, 8);
  wait(200, msec);
  driveChassis(0, 0);
  wait(500, msec);
  driveTo(-2, 1700);
  wait(400, msec);
  driveChassis(12, 12);
  wait(500, msec);
  driveChassis(0, 0);
  wait(650, msec);
  driveTo(-29, 1700);
  upper_intake.stop();
  wait(100, msec);
  enableRight = true;
  enableFR = true;
  wait(100, msec);
  turnToAngle(60, 800);
  descore.set(false);
  moveToPoint(20, -30, 1, 1500, true, 8);
  lower_intake.stop();
  descore.set(true);
  enableRight = false;
  enableFR = false;
  turnToAngle(-41, 800);
  driveTo(15.5, 800, true, 9);
  turnToAngle(-45, 800);
  scraper.set(true);
  wait(100, msec);
  intake_lift.set(false);
  lower_intake.spin(fwd, -7, voltageUnits::volt);
  upper_intake.spin(fwd, -8, voltageUnits::volt);
  wait(500, msec);
  upper_intake.spin(reverse, 7, voltageUnits::volt);
  lower_intake.spin(reverse, 4, voltageUnits::volt);
  wait(1700, msec);
  scraper.set(false);
  lower_intake.spin(reverse, 9.5, voltageUnits::volt);
  driveTo(-17, 1600, true, 8);
  intake_lift.set(true);
  turnToAngle(-90, 800);
  wait(10, msec);
  enableFront = true;
  resetPositionLeft(180);
  wait(10, msec);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  boomerang(-47, -38, 1, 220, 0.3, 2000, true, 10.5);
  turnToAngle(180, 200);
  scraper.set(true);
  wait(100, msec);
  enableRight = true;
  enableFront = true;

  wait(100, msec);
  moveToPoint(-47, -55, 1, 800, true, 8);
  turnToAngle(180, 300);
  driveToWall(7, 1300, 0, true, 12);

  boomerang(-60, -23, -1, 180, 0.3, 1000, true, 11);
  turnToAngle(180, 200);
  scraper.set(false);
  upper_intake.stop();
  lower_intake.stop();
  enableFront = false;
  enableBack = true;
  moveToPoint(-54.5, 20, -1, 2000, true, 10);
  wait(50, msec);
  enableBack = false;
  enableFront = true;
  turnToAngle(-90, 500);
  driveToWallRight(17, 950, 0, true, 12);
  enableRight = false;
  enableLeft = true;
  turnToAngle(0, 500);
  thread([]{
    upper_intake.spin(fwd, -12, voltageUnits::volt);
    wait(400, msec);
    upper_intake.stop(hold);
  });
  driveTo(-100, 700, false, 10);
  wings.set(false);
  descore.set(true);
  wait(100, msec);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  turnToAngle(0, 500, true, 12);
  wait(1700, msec);
  resetPositionFront(frontsensor, -2.875, 7.5, 0);
  resetPositionLeft(270);
  scraper.set(true);
  descore.set(false);
  upper_intake.stop();
  moveToPoint(-54, 52, 1, 1000, true, 12);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wings.set(true);
  driveToWall(7.5, 1400, 0, true, 12);

  turnToAngle(0, 300);
  wait(100, msec);
  resetPositionFront(frontsensor, -2.875, 7.5, 0);
  resetPositionLeft(270);
  wait(100, msec);
  moveToPoint(-56.5, 22, -1, 1000, false, 11);
  descore.set(true);
  upper_intake.spin(fwd, -12, voltageUnits::volt);
  wings.set(false);
  wait(100, msec);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wait(1500, msec);
  scraper.set(false);
  // */

   /*
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  intake_lift.set(true);
  descore.set(true);
  resetPositionFront(frontsensor, -2.875, 7.5, 0);
  resetPositionLeft(270);
  boomerang(-40, 53.5, 1, 85, 0.3, 2000, true, 11);
  boomerang(-24.5, 53.5, 1, 90, 0.1, 2000, true, 11);
  turnToAngle(90, 500);
  wait(75, msec);
  descore.set(false);
  wings.set(true);
  driveChassis(7, 7);
  wait(800, msec);
  driveChassis(0, 0);
  wait(500, msec);
  driveChassis(8.5, 8.5);
  wait(950, msec);
  scraper.set(true);
  driveChassis(0, 0);
  wait(150, msec);
  resetPositionLeft(0);
  resetPositionFront(frontsensor, -2.875, 7.5, 90);
  moveToPoint(27, 60, -1, 1200, false, 10);
  turnToAngle(180, 800);
  descore.set(false);
  scraper.set(false);
  correct_angle = 180;
  driveTo(5, 500, true, 12);
  wait(10, msec);
  resetPositionBack(0);
  resetPositionLeft(90);
  wait(10, msec);
  moveToPoint(30, 33, 1, 1000, true, 7);
  lower_intake.stop();
  descore.set(true);
  turnToAngle(45, 700, true, 10);

  scraper.set(true);
  descore.set(false);
  lower_intake.stop();
  upper_intake.stop();
  correct_angle = 45;
  thread([]{
    upper_intake.spin(fwd, -12, voltageUnits::volt);
    wait(400, msec);
    upper_intake.stop(hold);
  });
  driveTo(-11, 1000, true, 5);
  lower_intake.spin(fwd, 11, voltageUnits::volt);
  mid_goal.set(true);
  wait(60, msec);
  upper_intake.spin(fwd, 40, pct);
  wait(1500, msec);
  driveTo(1.5, 200, true, 4);
  upper_intake.spin(fwd, 30, pct);
  wait(1500, msec);
  upper_intake.stop();
  driveTo(-1, 400, true, 2.5);
  thread([]{
    wait(200, msec);
    upper_intake.spin(fwd, 12, voltageUnits::volt);
  });
  boomerang(38, 55, 1, 0, 0.1, 2000, true, 11);
  mid_goal.set(false);
  scraper.set(true);
  turnToAngle(0, 800);
  wait(50, msec);
  resetPositionFront(frontsensor, -2.875, 7.5, 0);
  resetPositionRight(90);
  wait(50, msec);
  moveToPoint(48, 61, 1, 800, false, 12);
  driveToWall(8, 1300, 0, true, 12);


  wait(50, msec);
  resetPositionFront(frontsensor, -2.875, 7.5, 0);
  resetPositionRight(90);
  wait(50, msec);
  boomerang(60, 26, -1, 0, 0.3, 1200, false, 10);
  scraper.set(false);
  moveToPoint(55, -20, -1, 1400, true, 11.5);
  wait(100, msec);
  turnToAngle(90, 500);
  driveToWallRight(18.5, 800, 0, true, 12);
  turnToAngle(180, 500);
  thread([]{
    upper_intake.spin(fwd, -12, voltageUnits::volt);
    wait(200, msec);
    upper_intake.stop(hold);
  });
  driveTo(-18, 1100, false, 12);
  wings.set(false);
  wait(100, msec);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  resetPositionFront(frontsensor, -2.875, 7.5, 180);
  resetPositionLeft(90);
  wait(1700, msec);
  scraper.set(true);
  upper_intake.stop();
  moveToPoint(55, -52, 1, 1000, true, 12);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wings.set(true);
  driveToWall(8, 1300, 0, true, 12);
  resetPositionFront(frontsensor, -2.875, 7.5, 180);
  resetPositionLeft(90);
  thread([]{
    upper_intake.spin(fwd, -12, voltageUnits::volt);
    wait(300, msec);
    upper_intake.stop(hold);
  });
  moveToPoint(54, -24, -1, 1000, false, 12);
  wings.set(false);
  wait(100, msec);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  wait(1600, msec);
  resetPositionFront(frontsensor, -2.875, 7.5, 180);
  resetPositionLeft(90);
  scraper.set(false);
  descore.set(true);
  boomerang(27, -56, 1, -90, 0.3, 2000, false, 12);
  correct_angle = -90;
  driveTo(20, 20000, true, 12); */

}

void elimleft(){
  resetChassis();
  wings.set(true);
  intake_lift.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  upper_intake.spin(fwd, 12, voltageUnits::volt);
  thread([]{
    doinker(500);
  });
  boomerang(-7, 19, 1, -50, 0.44, 2000, true, 10.5);

  //4 Ball 
  /*
  turnToPoint(-16, -3, 1, 400);
  boomerang(-16, -3, 1, 180, 0.1, 2000, false, 11);
  turnToAngle(180, 200);
  driveTo(-80, 600, true, 12);
  wings.set(false);
  wait(900, msec);*/

  //7 Ball 
  
  turnToPoint(-15, -15, 1, 700);
  boomerang(-15, -15, 1, 180, 0.44, 2000, false, 11);
  turnToAngle(180, 200);
  driveToWall(6, 700, 0, true, 12);
  correct_angle = 180;
  driveTo(-35, 800, false, 12);
  wings.set(false);
  wait(1200, msec);


  //Ending
  scraper.set(false);
  correct_angle = 145;
  driveTo(14, 1200, true, 12);
  turnToAngle(180, 400);
  correct_angle = 178;
  driveTo(-26, 1000000, true, 12);
  
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
  boomerang(10, 26, 1, 50, 0.44, 2000, false, 10.5);

  //4 Ball 
  
  turnToPoint(33, 0, 1, 400);
  boomerang(33, 0, 1, 180, 0.1, 2000, false, 11);
  turnToAngle(180, 200);
  driveTo(-80, 600, true, 12);
  wings.set(false);
  wait(800, msec);

  //7 Ball 
  /*
  turnToPoint(33, -15, 1, 600);
  boomerang(33, -15, 1, 180, 0.4, 2000, true, 11);
  turnToAngle(180, 200);
  driveToWall(6.5, 700, 0, true, 12);
  correct_angle = 180;
  driveTo(-35, 800, false, 12);
  wings.set(false);
  wait(1200, msec);
  */

  //Ending
  scraper.set(false);
  driveTo(3, 600, false, 10);
  swing(359, 1, 1200, true, 9.5);
  correct_angle = 3;
  driveTo(28, 100000, true, 12);
}