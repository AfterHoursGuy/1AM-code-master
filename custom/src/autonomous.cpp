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
  
}

void qualsoloawp(){
  
}

void rightsidequal(){

}

void leftsidequal(){
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){ 
    doinker(500);
    doinkerup(500);
   });
  moveToPoint(-5.5, 23, 1, 1200, false, 10);
  moveToPoint(-16, 38, 1, 1200, false, 10);
  thread([](){ doinker(300); });
  swing(270, 1, 900, true, 10);
  wait(400, msec);
  moveToPoint(7.5, 35, -1, 5000, false, 10);
  scraper.set(false);
  turnToAngle(-135, 900, true, 10);
  hood.spin(reverse, 7, voltageUnits::volt);
  wait(1000, msec);
  hood.stop(hold);
  moveToPoint(-22, 0, 1, 5000, false, 11);
  scraper.set(true);
  turnToAngle(180, 3000, true, 10);
  driveToWall(8.1, 800, 1000);
  lower_intake.stop(coast);
  mid_goal.set(true);
  moveToPoint(-22, 23, -1, 5000, true, 10);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(1000, msec);
  hood.stop(hold);
  moveToPoint(-22, 11, 1, 5000, true, 8);
  mid_goal.set(false);
  hood.spin(reverse, 7, voltageUnits::volt);
  wait(500, msec);
  hood.stop(hold);
  moveToPoint(-22, 25, -1, 1000, true, 12);
  wait(500, msec);
  moveToPoint(-22, 19, 1, 1400, true, 11);
  turnToAngle(90, 800, true, 10);
  driveChassis(-12,-12);
  wait(1000, msec);
  driveChassis(0,0);


}

void skills() {
  scraper.set(true);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  moveToPoint(0, 28, 1, 2000, false, 10);
  turnToAngle(270, 1500, true, 10);
  driveToWall(8.1, 1000, 1200);
  moveToPoint(24, 40, -1, 2000, false, 10);
  scraper.set(false);
  moveToPoint(85, 40, -1, 4000, true, 10);
  turnToAngle(0, 800, true, 10);
  driveToWall(17, 1000, 0);
  turnToAngle(90, 800, true, 10);
  thread([](){ 
    mid_goal.set(true);
    wait(900, msec);
    hood.spin(reverse, 12, voltageUnits::volt);
   });
  moveToPoint(80, 38, -1, 2000, false, 12);
  wait(1500, msec);
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
  moveToPoint(80, 38, -1, 2000, true, 12);
  wait(2000, msec);
  curveCircle(210, 9, 2000, true, 12);
  hood.stop(hold);
  moveToPoint(60, 0, 1, 2000, false, 12);
  moveToPoint(18, 8, 1, 2300, true, 9);
  turnToAngle(-45, 800, true, 10);
  moveToPoint(37, -9, -1, 2000, true, 10);
  mid_goal.set(false);
  hood.spin(reverse, 6, voltageUnits::volt);
  wait(1500, msec);
  hood.stop(hold);
  swing(270, 1, 1000, false, 12);
  moveToPoint(5, -65, 1, 4000, true, 12);
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
    mid_goal.set(true);
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
  boomerang(-23, 5, 1, 0, 0.4, 5000, true, 12);
}

void elimleft(){

}

void elimright(){
  
}