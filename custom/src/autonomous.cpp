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

void ballhold(){
  hood.spin(reverse, 2.5, voltageUnits::volt);
  mid_goal.set(true);
  example_optical_sensor.setLightPower(100);
  while (true) {
    if (example_optical_sensor.color() == color::red) {
      mid_goal.set(false);
      example_optical_sensor.setLightPower(0);
      lower_intake.stop(coast);
      hood.stop(coast);
      break;
    }
  }
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
  ballhold();
}

void sigsoloAWP(){
  scraper.set(true);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  boomerang(-16, 23, 1, -95, 0.4, 1700, true, 8);
  wait(330, msec);
  moveToPoint(22.5, 21.5, -1, 1100, true, 10);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(1300, msec);
  scraper.set(false);
  moveToPoint(4, 23, 1, 1500, true, 10);
  hood.stop(coast);
  turnToAngle(135, 800, true, 12);
  thread([](){doinker(580);});
  moveToPoint(28, 4, 1, 1000, true, 12);
  turnToAngle(-45, 600);
  mid_goal.set(true); 
  moveToPoint(33, -16, -1, 1000, true, 10);
  scraper.set(false);
  hood.spin(fwd, 12, voltageUnits::volt);
  wait(1300, msec);
  thread([](){
    wait(400, msec);
    hood.spin(reverse, 12, voltageUnits::volt);
    wait(500, msec);
    hood.stop();
  });
  moveToPoint(20, -4, 1, 1000, false, 12);
  mid_goal.set(false);
  turnToAngle(180, 700);
  thread([](){doinker(1000);});
  boomerang(24, -46, 1, 210, 0.45, 3000, false, 12, true);
  moveToPoint(-5, -74, 1, 1000, false, 11); 
  moveToPoint(23, -74, -1, 700, true, 12);
  scraper.set(false);
  hood.spin(reverse, 12, voltageUnits::volt);


  
  
  

  /*scraper.set(true);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  //moveToPoint(0, 15, 1, 1500, false, 9);
  //turnToAngle(90, 800, true, 12);
  //moveToPoint(20.5, 37, 1, 1500, true, 9);
  boomerang(24, 31, 1, 90, 0.6, 3000, true, 8);
  wait(700, msec);
  moveToPoint(-11, 30, -1, 1000, true, 10);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(1100, msec);
  hood.stop();
  scraper.set(false);
  moveToPoint(-3, 32, 1, 1500, true, 10);
  turnToAngle(225, 800, true, 12);
  hood.stop();
  thread([](){
    doinker(1700);
    lower_intake.stop(coast);
  });
  moveToPoint(-9, 4, 1, 800, false, 12);
  boomerang(-9, -50, 1, 180, 0.2, 1700, true, 12);
  wait(100, msec);
  scraper.set(false);
  turnToAngle(135, 600);
  wait(100, msec);
  moveToPoint(-22, -31, -1, 1100, true, 8);
  mid_goal.set(true); 
  hood.spin(reverse, 12, voltageUnits::volt);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  hood.spin(fwd, 12, voltageUnits::volt);
  wait(1500, msec);
  hood.stop();
  mid_goal.set(false);
  thread([](){doinker(1000);});
  boomerang(30, -59, 1, 87, 0.5, 2000, true, 9);
  wait(500, msec);
  moveToPoint(-14, -60, -1, 1500, true, 12);
  hood.spin(reverse, 12, voltageUnits::volt);*/
 


  /*lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){doinker(500);}); 
  moveToPoint(-5.3, 20.5, 1, 600, true, 9);
  wait(50, msec);
  turnToAngle(229, 600);
  mid_goal.set(true);
  hood_limiter.set(true);
  phood.set(true);
  moveToPoint(15, 31, -1, 1100, true, 12);
  hood.spin(fwd, 12, voltageUnits::volt);
  turnToAngle(225, 650);
  wait(350, msec);
  scraper.set(false);
  hood.stop();
  correct_angle = 225;
  driveTo(13, 500, true, 12);
  mid_goal.set(false);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  turnToAngle(90, 600);
  thread([](){
    doinker(800);
  });
  moveToPoint(44, 25, 1, 900, true, 12);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  wait(100, msec);
  turnToAngle(60, 600);
  thread([](){doinkerup(200);});
  thread([](){doinker(650);});
  correct_angle = 60;
  driveTo(27, 900, true, 10);
  wait(250, msec);
  moveToPoint(45, 24, -1, 1000, true, 12);
  turnToAngle(137, 700);
  correct_angle = 137;
  driveTo(-13, 600, true, 12);
  mid_goal.set(true);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  wait(100, msec);
  hood.spin(fwd, 12, voltageUnits::volt);
  wait(500, msec); 
  hood.stop();
  mid_goal.set(false);
  hood_limiter.set(false);
  phood.set(false);
  scraper.set(false);
  moveToPoint(73.5, -5, 1, 3000, true, 12);
  wait(100, msec);
  scraper.set(true);
  turnToAngle(183, 930);
  driveTo(11, 600, true, 12);
  wait(500, msec);
  moveToPoint(74.5, 27, -1, 1000, true, 12);
  hood.spin(reverse, 12, voltageUnits::volt);*/

}

void qualsoloawp(){
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){doinker(500);}); 
  moveToPoint(-5.3, 24, 1, 700, true, 9);
  wait(100, msec);
  turnToAngle(229, 600);
  mid_goal.set(true);
  hood_limiter.set(true);
  phood.set(true);
  wait(100, msec);
  moveToPoint(17, 35, -1, 1100, true, 12);
  hood.spin(fwd, 12, voltageUnits::volt);
  turnToAngle(225, 650);
  wait(600, msec);
  scraper.set(false);
  hood.stop();
  correct_angle = 225;
  driveTo(13, 500, true, 12);
  mid_goal.set(false);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  turnToAngle(90, 600);
  wait(100, msec);
  thread([](){
    doinker(800);
  });
  moveToPoint(50, 25, 1, 1100, true, 12);
  wait(150, msec);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  turnToAngle(135, 700);
  moveToPoint(41, 36, -1, 1000, true, 9);
  mid_goal.set(true);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  wait(100, msec);
  hood.spin(fwd, 12, voltageUnits::volt);
  wait(500, msec); 
  hood.stop();
  mid_goal.set(false);
  hood_limiter.set(false);
  phood.set(false);
  scraper.set(false);
  moveToPoint(73, -4, 1, 3000, true, 12);
  wait(100, msec);
  scraper.set(true);
  turnToAngle(183, 930);
  driveTo(11.5, 600, true, 12);
  wait(500, msec);
  moveToPoint(74.5, 27, -1, 1000, true, 12);
  hood.spin(reverse, 12, voltageUnits::volt);

}

void rightsidequal(){
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){
    doinker(467);
    doinkerup(350);
    doinker(600);
    lower_intake.stop();
  });
  moveToPoint(8, 28, 1, 1000, false, 11); 
  moveToPoint(34, 42.5, 1, 1000, false, 11.5);
  turnToAngle(90, 600, true);
  wait(200, msec);
  thread([](){
    wait(700, msec);
    lower_intake.spin(reverse, 12, voltageUnits::volt);
  });
  moveToPoint(15, 28, -1, 1000, true, 9); 
  turnToAngle(135, 500);
  moveToPoint(11.3, 35, -1, 800, true, 8);
  mid_goal.set(true);
  hood.spin(fwd, 12, voltageUnits::volt);
  wait(800, msec);
  hood.stop();
  mid_goal.set(false);
  scraper.set(false);
  moveToPoint(44, -1, 1, 1400, true, 9);
  scraper.set(true);
  turnToAngle(180, 600);
  boomerang(45, -20, 1, 180, 0.1, 1000, true, 12); 
  wait(600, msec);
  moveToPoint(46, 26, -1, 900, true, 12);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(2500, msec);
  moveToPoint(45, -12, 1, 1000, true, 10);
  wait(400, msec); 
  scraper.set(false);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  moveToPoint(46, 27, -1, 900, true, 12);

}

void leftsidequal(){
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){doinker(500);}); 
  moveToPoint(-5.3, 20.5, 1, 600, true, 9);
  turnToAngle(225, 600);
  thread([](){
    wait(200, msec);
    hood_limiter.set(true);
    phood.set(true);
    mid_goal.set(true);

  });
  moveToPoint(15, 30.5, -1, 1400, true, 12);
  hood.spin(fwd, 12, voltageUnits::volt);
  turnToAngle(225, 600, true, 12);
  wait(750, msec);
  scraper.set(false);
  hood.stop(coast);
  driveTo(4, 300, true, 8);
  mid_goal.set(false);
  hood_limiter.set(false);
  phood.set(false);
  turnToAngle(284, 600, true, 12);
  correct_angle = normalizeTarget(284);
  thread([](){doinker(590);}); 
  driveTo(25.5, 700, true, 11);
  wait(250, msec);
  correct_angle = normalizeTarget(284);
  driveTo(-16.5, 700, true, 11);
  scraper.set(false);
  turnToAngle(195, 700);
  moveToPoint(-23, -5, 1, 1500, true, 9);
  turnToAngle(180, 700);
  scraper.set(true);
  wait(150, msec);
  driveTo(11.5, 525);
  wait(750, msec);
  moveToPoint(-23, 27, -1, 1000, true, 12);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(2000, msec);
  moveToPoint(-23, -9, 1, 1500, false, 12);

}

void skills() {
  /*scraper.set(true);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  moveToPoint(0, 29, 1, 1500, false, 9);
  turnToAngle(-90, 800, true, 12);
  moveToPoint(-14, 29, 1, 1700, true, 9);
  wait(1700, msec);
  boomerang(22.5, 40, -1, -90, 0.2, 1100, false, 8);
  scraper.set(false);
  moveToPoint(85, 38, -1, 1500, true, 10);
  turnToAngle(180, 800, true, 12);

  moveToPoint(94, 37, 1, 1600, true, 8);
  turnToAngle(90, 800, true, 12);
  moveToPoint(69, 37, -1, 1000, true, 8);
  hood.spin(reverse, 12, voltageUnits::volt);
  turnToAngle(90, 600, true, 12);
  wait(4000, msec);
  hood.stop();
  scraper.set(true);

  moveToPoint(105.75, 36, 1, 2000, true, 6);
  wait(1800, msec);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  wait(100, msec);
  lower_intake.stop(coast);
  moveToPoint(70, 32.5, -1, 2200, true, 6);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  hood.spin(reverse, 12, voltageUnits::volt);
  turnToAngle(90, 600, true, 12);
  wait(4000, msec); //2200 for reg scoring
  hood.stop();
  scraper.set(false); 

  moveToPoint(86, 29, 1, 2000, true, 8);

  turnToAngle(180, 1000);
  moveToPoint(86, -68, 1, 3000, true, 9);
  scraper.set(true);
  turnToAngle(90, 1000);
  boomerang(102, -66, 1, 90, 0.3, 3500, true, 6);
  wait(1700, msec);

  
  moveToPoint(86, -66, -1, 2000, true, 9);
  turnToAngle(40, 1000);
  scraper.set(false);
  moveToPoint(-0, -70, -1, 3000, true, 10);
  turnToAngle(180, 800);
  moveToPoint(-0, -68, -1, 1500, true, 8);
  turnToAngle(-90, 800);
  boomerang(20, -69, -1, 270, 0.5, 1500, true, 8);
  hood.spin(reverse, 12, voltageUnits::volt);
  scraper.set(true);
  wait(4000, msec);
  hood.stop();
  moveToPoint(-21.5, -67, 1, 3400, true, 6);
  turnToAngle(-90, 800);
  wait(1700, msec);
  moveToPoint(20, -69, -1, 1500, true, 7);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(5000, msec);
  hood.stop();
  scraper.set(false);
  moveToPoint(5, -65, 1, 1500, false, 8);
  boomerang(-22, -32, 1, 0, 0.3, 4600, false, 9);
  hood_limiter.set(true);
  phood.set(true);
  moveToPoint(-22, 5, 1, 3000, true, 7);*/

  scraper.set(true);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  boomerang(-16.5, 23, 1, -95, 0.4, 1700, true, 8);
  wait(1700, msec);
  boomerang(22.5, 38, -1, -90, 0.2, 1100, false, 8);
  scraper.set(false);
  moveToPoint(85, 30, -1, 1500, true, 10);
  turnToAngle(180, 800, true, 12);

  moveToPoint(94, 28, 1, 1600, true, 8);
  turnToAngle(90, 800, true, 12);
  moveToPoint(69, 32, -1, 1000, true, 8);
  hood.spin(reverse, 12, voltageUnits::volt);
  turnToAngle(90, 600, true, 12);
  wait(4000, msec);
  hood.stop();
  scraper.set(true);

  moveToPoint(105.75, 29, 1, 2000, true, 6);
  wait(1800, msec);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  wait(100, msec);
  lower_intake.stop(coast);
  moveToPoint(70, 32.5, -1, 2200, true, 6);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  hood.spin(reverse, 12, voltageUnits::volt);
  turnToAngle(90, 600, true, 12);
  wait(4000, msec); //2200 for reg scoring
  hood.stop();
  scraper.set(false); 

  moveToPoint(86, 29, 1, 2000, true, 8);

  turnToAngle(180, 1000);
  moveToPoint(86, -68, 1, 3000, true, 9);
  scraper.set(true);
  turnToAngle(90, 1000);
  boomerang(102, -66, 1, 90, 0.3, 3500, true, 6);
  wait(1700, msec);

  
  moveToPoint(86, -66, -1, 2000, true, 9);
  turnToAngle(40, 1000);
  scraper.set(false);
  moveToPoint(-0, -70, -1, 3000, true, 10);
  turnToAngle(180, 800);
  moveToPoint(-0, -68, -1, 1500, true, 8);
  turnToAngle(-90, 800);
  boomerang(20, -69, -1, 270, 0.5, 1500, true, 8);
  hood.spin(reverse, 12, voltageUnits::volt);
  scraper.set(true);
  wait(4000, msec);
  hood.stop();
  moveToPoint(-21.5, -67, 1, 3400, true, 6);
  turnToAngle(-90, 800);
  wait(1700, msec);
  moveToPoint(20, -69, -1, 1500, true, 7);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(5000, msec);
  hood.stop();
  scraper.set(false);
  moveToPoint(5, -65, 1, 1500, false, 8);
  boomerang(-22, -32, 1, 0, 0.3, 4600, false, 9);
  hood_limiter.set(true);
  phood.set(true);
  moveToPoint(-22, 5, 1, 3000, true, 7);

}

void elimleft(){
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){doinker(350);}); 
  moveToPoint(-5.3, 25, 1, 1000, true, 9);
  //turnToAngle(-50, 600);
  thread([](){
    doinkerup(350);
    doinker(650);
  });
  boomerang(-25, 40, 1, -74, 0.3, 3000, true, 8);
  //moveToPoint(-25, 32.5, 1, 1400, false, 9);
  wait(300, msec); 
  moveToPoint(-5.3, 20.5, -1, 600, true, 9);
  turnToAngle(225, 600);
  scraper.set(false);
  moveToPoint(-24.5, -5, 1, 1400, true, 9);
  scraper.set(true);
  turnToAngle(180, 700);
  moveToPoint(-24.5, -12, 1, 1400, true, 9);
  wait(500,msec);
  moveToPoint(-25, 23, -1, 1400, true, 10);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(3000, msec);
  moveToPoint(-24.5, -11, 1, 1400, true, 9);
  wait(600,msec);
  thread([](){
    doinkerup(100);
    lower_intake.spin(fwd, 12, voltageUnits::volt);
  });
  moveToPoint(-25, 23, -1, 1400, true, 12);

}