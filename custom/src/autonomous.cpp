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
  driveTo(24, 2000);
  resetPositionFrontLeft();
}

void sigsoloAWP(){
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  scraper.set(true);
  mid_goal.set(true);
  moveToPoint(0, 27, 1, 1200, false, 12);
  swing(270, 1, 700, true, 12);
  driveToWall(8.1, 900, 100);
  moveToPoint(25, 29, -1, 1200, true, 12);
  hood.spin(reverse, 12, voltageUnits::volt);
  scraper.set(false);
  wait(600, msec);
  driveTo(8, 500, false);
  turnToPoint(36, 11, 1, 700);
  hood.stop(hold);
  thread([](){ 
    doinker(500);
    doinkerup(300);
   });
  moveToPoint(36, 11, 1, 1200, true, 10);
  turnToAngle(-45, 1000, true, 12);
  boomerang(38, -7.5, -1, -45, 0.3, 1200, true, 11);
  mid_goal.set(false);
  wait(95, msec);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(500, msec);
  hood.stop(hold);
  driveTo(6, 500, false);
  turnToPoint(31, -42, 1, 700);
  thread([](){ 
    doinker(800);
    doinkerup(300);
    doinker(700);
   });
  moveToPoint(31, -42, 1, 2000, true, 12);
  boomerang(-3, -67, 1, -90, 0.4, 2100, false, 12);
  mid_goal.set(true);
  driveToWall(8.1, 800, 500);
  moveToPoint(25, -68, -1, 2000, true, 12);
  hood.spin(reverse, 12, voltageUnits::volt);
}

void qualsoloawp(){
  
}

void rightsidequal(){
  mid_goal.set(true);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){ 
    doinker(500);
    doinkerup(500);
   });
  moveToPoint(8, 30, 1, 1200, false, 11);
  moveToPoint(22.5, 46, 1, 1800, true, 11);
  thread([](){ doinker(200); });
  swing(90, 1, 900, true, 10);
  wait(100, msec);
  moveToPoint(4, 37, -1, 1300, false, 12);
  scraper.set(false);
  turnToAngle(-45, 900, true, 12);
  driveTo(4, 500, true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  hood.spin(fwd, 12, voltageUnits::volt);
  wait(1200, msec);
  hood.stop(hold);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  moveToPoint(29, 0, -1, 4000, false, 12);
  scraper.set(true);
  turnToAngle(180, 1000, true, 12);
  driveToWall(8.1, 800, 1200);
  moveToPoint(42, 23, -1, 1200, true, 12);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(800, msec);
  hood.stop(hold);
  moveToPoint(42, 17, 1, 700, true, 12);
  mid_goal.set(false);
  hood.spin(reverse, 7, voltageUnits::volt);
  wait(500, msec);
  hood.stop(hold);
  moveToPoint(42, 23, -1, 700, true, 12);
  wait(300, msec);
  moveToPoint(42, 17, 1, 700, true, 12);
  turnToAngle(-90, 800, true, 10);
  driveChassis(-12,-12);
  

}

void leftsidequal(){
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){ 
    doinker(500);
    doinkerup(500);
   });
  moveToPoint(-5.5, 23, 1, 1200, false, 10);
  moveToPoint(-17, 38, 1, 1500, true, 10);
  thread([](){ doinker(200); });
  swing(270, 1, 900, true, 10);
  wait(100, msec);
  moveToPoint(4, 33.5, -1, 1300, false, 10);
  scraper.set(false);
  turnToAngle(-135, 900, true, 10);
  driveTo(-2, 500, true);
  hood.spin(reverse, 40, percentUnits::pct);
  wait(1000, msec);
  hood.stop(hold);
  moveToPoint(-22, 0, 1, 5000, false, 11);
  scraper.set(true);
  mid_goal.set(true);
  turnToAngle(180, 3000, true, 10);
  driveToWall(8.1, 800, 1000);
  lower_intake.stop(coast);
  moveToPoint(-22, 23, -1, 5000, true, 10);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(800, msec);
  hood.stop(hold);
  moveToPoint(-22, 11, 1, 1000, true, 8);
  mid_goal.set(false);
  hood.spin(reverse, 7, voltageUnits::volt);
  wait(500, msec);
  hood.stop(hold);
  moveToPoint(-22, 25, -1, 1000, true, 12);
  wait(500, msec);
  moveToPoint(-22, 19, 1, 1400, true, 11);
  turnToAngle(90, 800, true, 10);
  driveChassis(-12,-12);


}

void skills() {
  scraper.set(true);
  mid_goal.set(true);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  moveToPoint(0, 29, 1, 2000, false, 10);
  turnToAngle(270, 1500, true, 10);
  driveToWall(8.1, 1000, 1200);
  moveToPoint(24, 40, -1, 2000, false, 10);
  scraper.set(false);
  moveToPoint(85, 40, -1, 4000, true, 10);
  turnToAngle(0, 800, true, 10);
  driveToWall(17, 1000, 0);
  turnToAngle(90, 800, true, 10);
  thread([](){ 
    wait(900, msec);
    hood.spin(reverse, 12, voltageUnits::volt);
   });
  moveToPoint(80, 39.5, -1, 2000, false, 12);
  wait(1700, msec);
  hood.stop(hold);
  scraper.set(true);
  correct_angle = 90;
  driveTo(20, 2000);
  driveToWall(8.1, 2000, 1200);
  thread([](){ 
    wait(1000, msec);
    hood.spin(reverse, 12, voltageUnits::volt);
    scraper.set(false);
   });
  moveToPoint(80, 39.5, -1, 2000, true, 12);
  wait(2000, msec);
  curveCircle(210, 9, 2000, true, 12);
  hood.stop(hold);
  moveToPoint(60, 0, 1, 2000, false, 12);
  moveToPoint(18, 8, 1, 2300, true, 9);
  turnToAngle(-45, 800, true, 10);
  moveToPoint(37.5, -9.5, -1, 2000, true, 10);
  mid_goal.set(false);
  hood.spin(reverse, 30, percentUnits::pct);
  wait(1800, msec);
  swing(270, 1, 1000, false, 12);
  mid_goal.set(true);
  hood.stop(hold);
  moveToPoint(5, -66, 1, 4000, true, 12);
  scraper.set(true);
  turnToAngle(270, 1500, true, 10);
  driveToWall(8.1, 2000, 1200);
  moveToPoint(24, -79, -1, 2000, false, 10);
  scraper.set(false);
  turnToAngle(270, 800, true, 12);
  moveToPoint(89, -75, -1, 5000, true, 10);
  turnToAngle(180, 800, true, 12);
  driveToWall(19.5, 1000, 0);
  turnToAngle(90, 800, true, 10);
  thread([](){ 
    wait(900, msec);
    hood.spin(reverse, 12, voltageUnits::volt);
   });
  moveToPoint(75, -55, -1, 1000, true, 10);
  wait(1500, msec);
  hood.stop(hold);
  scraper.set(true);
  correct_angle = 90;
  driveTo(20, 2000);
  driveToWall(8.1, 2000, 1200);
  
  thread([](){ 
    wait(1100, msec);
    hood.spin(reverse, 12, voltageUnits::volt);
   });
  moveToPoint(75, -55, -1, 1000, true, 10);
  wait(2000, msec);
  hood.stop(hold);
  scraper.set(false);
  driveTo(3, 700);
  swing(-40, 1, 1200, true, 12);
  moveToPoint(67, -40, 1, 3000, false, 8);
  moveToPoint(18, -40, 1, 3000, true, 8);
  turnToAngle(45, 800, true, 10);
  moveToPoint(28, -20, 1, 2000, true, 10);
  turnToAngle(45, 800, true, 12);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  hood.spin(fwd, 12, voltageUnits::volt);
  wait(2000, msec);
  moveToPoint(-5, -45, -1, 2000, true, 10);
  turnToAngle(-40, 1500, true, 10);
  boomerang(-23, 8, 1, 0, 0.4, 15000, true, 12);
}

void elimleft(){

}

void elimright(){
  
}