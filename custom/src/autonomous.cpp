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
  
}

void elimleft(){
  mid_goal.set(true);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  wings.set(true);
  thread([]{doinker(550);});
  moveToPoint(-5.5, 22, 1, 1000, true, 10);
  turnToPoint(-23, -5, 1, 800);
  moveToPoint(-23, -5, 1, 1000, true, 12);
  turnToAngle(180, 1000);
  driveToWall(7.5, 1200, 400, true, 12);
  scraper.set(false);
  moveToPoint(-23.5, 13.5, -1, 1800, true, 10);
  wings.set(false);
  wait(100, msec);
  thread s = thread(score);
  wait(600, msec);
  lower_intake.stop();
  driveTo(4, 1000, false, 12);
  turnToAngle(-90, 800);
  driveToWall(7.75, 700, 0, true, 12);
  turnToAngle(180, 800);
  driveTo(-35, 2000, true, 12);
  s.interrupt();
  lower_intake.spin(forward, -12, voltageUnits::volt);
  fastarmPID(0);
  



}

void elimright(){
  
}