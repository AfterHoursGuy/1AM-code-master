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
  moveToPoint(-5.5, 23, 1, 5000, false, 10);
  moveToPoint(-16, 38, 1, 5000, false, 10);
  thread([](){ doinker(300); });
  swing(270, 1, 5000, true, 10);
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
  moveToPoint(-22, -7.5, 1, 5000, true, 11);
  wait(1200, msec);
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
  moveToPoint(-22, 25, -1, 5000, true, 12);
  wait(500, msec);
  moveToPoint(-22, 19, 1, 1400, true, 11);
  turnToAngle(90, 800, true, 10);
  driveChassis(-12,-12);
  wait(1000, msec);
  driveChassis(0,0);


}

void skills() {
  

}

void elimleft(){

}

void elimright(){
  
}