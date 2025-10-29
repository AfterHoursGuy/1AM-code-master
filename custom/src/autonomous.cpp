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
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  hood.spin(reverse, 12, voltageUnits::volt);
  correct_angle = 0;
  driveToWall(8.8, 5000, true, 12); 
  moveToPoint(0, 0, -1, 4000, true, 8);

  
}

void sigsoloAWP(){
  scraper.set(true);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  boomerang(-16.5, 23, 1, -95, 0.4, 1900, true, 8);
  wait(330, msec);
  moveToPoint(21, 21.5, -1, 1100, true, 10);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(1300, msec);
  scraper.set(false);
  moveToPoint(4, 23, 1, 1500, true, 10);
  hood.stop(coast);
  turnToAngle(135, 800, true, 12);
  thread([](){doinker(670);});
  moveToPoint(33, 4.5, 1, 1000, true, 12);
  turnToAngle(-45, 600);
  mid_goal.set(true); 
  moveToPoint(31, -16, -1, 1000, true, 10);
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
  thread([](){doinker(900);});
  boomerang(24, -46, 1, 210, 0.1, 3000, false, 12, true);
  moveToPoint(-5, -76, 1, 1000, false, 11); 
  moveToPoint(23, -70, -1, 700, true, 12);
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
    doinker(620);
    doinkerup(400);
    doinker(1400);
  }); 
  moveToPoint(5.3, 20.5, 1, 1500, true, 9); 
  moveToPoint(10, 40, 1, 1500, true, 9);
  turnToAngle(90, 600);
  moveToPoint(35, 47, 1, 1500, true, 9);
  wait(200, msec);
  boomerang(14, 27, -1, 30, 0.15, 1900, true, 9);
  wait(100, msec);
  /*lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){
    doinker(580);
    doinkerup(350);
    doinker(1100);
  });
  moveToPoint(8, 28, 1, 1500, false, 9); 
  turnToAngle(45, 600, true);
  moveToPoint(35, 44, 1, 1500, true, 9);
  wait(200, msec);
  thread([](){
    wait(700, msec);
    lower_intake.spin(reverse, 12, voltageUnits::volt);
  });
  moveToPoint(14, 27, -1, 1000, true, 9); 
  turnToAngle(135, 500);*/
  turnToAngle(135, 500);
  moveToPoint(8, 39, -1, 1500, true, 9);
  mid_goal.set(true);
  hood.spin(fwd, 12, voltageUnits::volt);
  turnToAngle(135, 600);
  wait(350, msec);
  hood.stop();
  mid_goal.set(false);
  scraper.set(false);
  wait(100, msec);
  mid_goal.set(true);
  mid_goal.set(false);
  scraper.set(false);
  moveToPoint(43.5, 0, 1, 1800, true, 9);
  scraper.set(true);
  turnToAngle(180, 600);
  moveToPoint(44, -12, 1, 1000, true, 9); 
  wait(600, msec);
  moveToPoint(44, 19, -1, 900, true, 9);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(2500, msec);
  moveToPoint(44.5, -12, 1, 1000, true, 10);

}

void leftsidequal(){
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){
    doinker(500);
    doinkerup(400);
    doinker(1300);
  }); 
  moveToPoint(-5.3, 20.5, 1, 900, true, 9);
  moveToPoint(-10, 37, 1, 900, true, 9);
  turnToAngle(-90, 600);
  moveToPoint(-24, 37, 1, 900, true, 9);
  wait(200, msec);
  boomerang(-5, 18, -1, -30, 0.25, 1900, true, 9);
  wait(200, msec);
  turnToAngle(-135, 700);
  wait(200, msec);
  moveToPoint(12, 34, -1, 1400, true, 9);
  hood.spin(fwd, 12, voltageUnits::volt);
  mid_goal.set(true);
  turnToAngle(225, 600, true, 12);
  wait(600, msec);
  hood.stop(coast);
  scraper.set(false);
  lower_intake.stop(coast);
  thread([](){
    wait(167, msec);
    mid_goal.set(false);
    lower_intake.spin(reverse, 12, voltageUnits::volt);
    doinker(800);
  });
  moveToPoint(-26, -4, 1, 1900, true, 8);
  turnToAngle(180, 500);
  moveToPoint(-26, -13.5, 1, 1900, true, 8);
  wait(600, msec);
  lower_intake.stop(coast);
  moveToPoint(-24, 18, -1, 1900, true, 8);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(2500, msec);
  moveToPoint(-25, -13.5, 1, 1900, true, 8);

}

void skills() {
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

  moveToPoint(105.75, 30.5, 1, 2000, true, 6);
  wait(1800, msec);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  wait(100, msec);
  lower_intake.stop(coast);
  moveToPoint(70, 32.5, -1, 2200, true, 6);
  turnToAngle(90, 600, true, 12);
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
  moveToPoint(105, -66, 1, 3500, true, 6);
  wait(1900, msec);

  
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
  turnToAngle(-90, 600);
  wait(4000, msec);
  hood.stop();
  moveToPoint(-20.5, -67.8, 1, 3400, true, 6);
  turnToAngle(-90, 800);
  wait(1700, msec);
  moveToPoint(20, -69, -1, 1500, true, 7);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(5000, msec);
  scraper.set(false);
  turnToAngle(-90, 600);
  moveToPoint(5, -65, 1, 1500, false, 8);
  boomerang(-22, -32, 1, 0, 0.3, 4600, false, 9);
  hood_limiter.set(true);
  phood.set(true);
  moveToPoint(-22, 0, 1, 3000, true, 7);

}

void elimleft(){
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){
    doinker(500);
    doinkerup(400);
    doinker(1300);
  }); 
  moveToPoint(-5.3, 20.5, 1, 900, true, 9);
  moveToPoint(-10, 37, 1, 900, true, 9);
  turnToAngle(-90, 600);
  moveToPoint(-24, 37, 1, 900, true, 9);
  wait(200, msec);
  boomerang(-5, 20, -1, -30, 0.25, 1500, true, 9);
  wait(100, msec);
  turnToAngle(-135, 500);
  lower_intake.stop(coast);
  wait(100, msec);
  scraper.set(false);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  moveToPoint(-31, 4, 1, 1900, true, 8);
  wait(100, msec);
  turnToAngle(180, 500);
  scraper.set(true);
  wait(100, msec);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  moveToPoint(-26.5, 20, -1, 1500, true, 9);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(2367, msec);
  hood.stop();
  moveToPoint(-27, -16, 1, 1700, true, 7);
  wait(1000, msec);
  moveToPoint(-26, 18, -1, 1200, true, 10);
  scraper.set(false);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(1100, msec);
  hood.stop();

}

void elimright(){
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){
    doinker(580);
    doinkerup(400);
    doinker(1400);
  }); 
  moveToPoint(5.3, 20.5, 1, 1500, true, 9); 
  moveToPoint(10, 40, 1, 1500, true, 9);
  turnToAngle(90, 600);
  moveToPoint(35, 47, 1, 1500, true, 9);
  wait(200, msec);
  boomerang(14, 27, -1, 30, 0.15, 1500, true, 9);
  wait(200, msec);
  turnToAngle(135, 500);
  wait(200, msec);
  scraper.set(false);
  moveToPoint(43, 12, 1, 1800, true, 9);
  turnToAngle(180, 600);
  moveToPoint(46, 18, -1, 1200, true, 9);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(2367, msec);
  hood.stop();
  scraper.set(true);
  turnToAngle(180, 600);
  moveToPoint(45.5, -12, 1, 1000, true, 9); 
  wait(1000, msec);
  moveToPoint(46, 23, -1, 900, true, 9);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(1400, msec);
  hood.stop();
  scraper.set(true);

}