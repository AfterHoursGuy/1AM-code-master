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
  /*Sig solo autonomous, Gets 3 blocks out of the left matchloader, and scores them in the long goal,
  then gets the ball bundle on the left side and scores all 3 in the upper middle goal, then gets the
  bundle from the right side, and scores all 3 in the long goal. This auton gets us the all important 
  autonomous winpoint alone.*/

  //Runs an arc into the first matchloader, picking up 3 blocks
  scraper.set(true);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  boomerang(-16.5, 23, 1, -95, 0.4, 1900, true, 8);
  wait(330, msec);

  //Moves to the long goal and scores the 3 blocks from the matchloader
  moveToPoint(21, 21.5, -1, 1100, true, 10);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(1300, msec);
  scraper.set(false);

  //Moves away from the goal, threads a ball control command, and moves to the first block bundle
  moveToPoint(4, 23, 1, 1500, true, 10);
  hood.stop(coast);
  turnToAngle(135, 800, true, 12);
  thread([](){doinker(670);});
  moveToPoint(33, 4.5, 1, 1000, true, 12);

  //Turns toward the middle goal, moves to it with heading corrections, and scores 3 blocks
  turnToAngle(-45, 600);
  mid_goal.set(true); 
  moveToPoint(31, -16, -1, 1000, true, 10);
  scraper.set(false);
  hood.spin(fwd, 12, voltageUnits::volt);
  wait(1300, msec);

  //Threads antijam for block collection
  thread([](){
    wait(400, msec);
    hood.spin(reverse, 12, voltageUnits::volt);
    wait(500, msec);
    hood.stop();
  });

  //Moves away from the middle goal and turns towards the next bundle
  moveToPoint(20, -4, 1, 1000, false, 12);
  mid_goal.set(false);
  turnToAngle(180, 700);

  //Threads ball control, moves and collects the block bundle, and scores 3 blocks in the other long goal
  thread([](){doinker(900);});
  boomerang(24, -46, 1, 210, 0.1, 3000, false, 12, true);
  moveToPoint(-5, -76, 1, 1000, false, 11); 
  moveToPoint(23, -70, -1, 700, true, 12);
  scraper.set(false);
  hood.spin(reverse, 12, voltageUnits::volt);

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
  /*Qualification left side autonomous, picks up 8 balls, scores three in the upper middle goal, 
  5 in the long goal, and empties the matchloader*/

  //Runs lower intake and threads ball control commands for the beggining sequence
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){
    doinker(650);
    doinkerup(400);
    doinker(1400);
  }); 

  //Runs movements to pick up the first 5 balls in the sequence, uses the previous thread for ball control
  moveToPoint(5.3, 20.5, 1, 1500, true, 9); 
  moveToPoint(10, 40, 1, 1500, true, 9);
  turnToAngle(90, 600);
  moveToPoint(35, 47, 1, 1500, true, 9);
  wait(200, msec);

  //Runs an arc away from the auto line to prime for the middle goal approach
  boomerang(14, 27, -1, 30, 0.15, 1900, true, 9);
  wait(100, msec);
  turnToAngle(135, 500);

  //Moves into position with heading corrections to align with the middle goal, and scores 3 blocks
  moveToPoint(8, 39, -1, 1500, true, 9);
  mid_goal.set(true);
  hood.spin(fwd, 12, voltageUnits::volt);
  turnToAngle(135, 600);
  hood.stop(coast);
  scraper.set(false);
  lower_intake.stop(coast);

  //Threads commands for antijam to prepare to get blocks out of the matchloader
  thread([](){
    wait(167, msec);
    mid_goal.set(false);
    lower_intake.spin(reverse, 12, voltageUnits::volt);
    doinker(800);
  });

  //Moves into position to get blocks from the matchloader
  moveToPoint(42, 0, 1, 1800, true, 9);
  turnToAngle(180, 600);

  //Gets the first three blocks out of the matchloader
  correct_angle = 180;
  driveToWall(8.8, 800, true, 12);

  /*Moves into position to score in the long goal, and scores all 5 blocks, 
  then moves back to the matchloader to empty it*/
  moveToPoint(44, 19, -1, 900, true, 9);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(2500, msec);
  correct_angle = 180;
  driveToWall(8.8, 2000, 2000, true, 12);

}

void leftsidequal(){
  /*Qualification left side autonomous, picks up 8 balls, scores three in the upper middle goal, 
  5 in the long goal, and empties the matchloader*/

  //Runs lower intake and threads ball control commands for the beggining sequence
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){
    doinker(500);
    doinkerup(400);
    doinker(1300);
  }); 

  //Runs movements to pick up the first 5 balls in the sequence, uses the previous thread for ball control
  moveToPoint(-5.3, 20.5, 1, 900, true, 9);
  moveToPoint(-10, 37, 1, 900, true, 9);
  turnToAngle(-90, 600);
  moveToPoint(-24, 37, 1, 900, true, 9);
  wait(200, msec);

  //Runs an arc away from the auto line to prime for the middle goal approach
  boomerang(-5, 18, -1, -30, 0.25, 1900, true, 9);
  wait(200, msec);
  turnToAngle(-135, 700);
  wait(200, msec);

  //Moves into position with heading corrections to align with the middle goal, and scores 3 blocks
  moveToPoint(12, 32.5, -1, 1400, true, 9);
  hood.spin(fwd, 12, voltageUnits::volt);
  mid_goal.set(true);
  turnToAngle(225, 600, true, 12);
  wait(600, msec);
  hood.stop(coast);
  scraper.set(false);
  lower_intake.stop(coast);

  //Threads commands for antijam to prepare to get blocks out of the matchloader
  thread([](){
    wait(167, msec);
    mid_goal.set(false);
    lower_intake.spin(reverse, 12, voltageUnits::volt);
    doinker(800);
  });

  //Moves into position to get blocks from the matchloader
  moveToPoint(-25, -4, 1, 1900, true, 8);
  turnToAngle(180, 500);

  //Gets the first three blocks out of the matchloader
  correct_angle = 180;
  driveToWall(8.8, 800, 700, true, 12);
  lower_intake.stop(coast);

  /*Moves into position to score in the long goal, and scores all 5 blocks, 
  then moves back to the matchloader to empty it*/
  moveToPoint(-25, 19.5, -1, 1900, true, 8);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(2500, msec);
  correct_angle = 180;
  driveToWall(8.8, 2000, 2000, true, 12);

}

void skills() {
  /*Skills autonomous, this code is ran during autonomous skills, it scores 75 points by 
  emptying all 4 matchloaders, and scoring all 24 blocks and the preload, getting both 
  long goal control zones, and at the end it clears the park zones and parks.
  */

  //Runs an arc and empties the first matchloader
  scraper.set(true);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  boomerang(-16.5, 23, 1, -95, 0.4, 1700, true, 8);
  wait(1700, msec);

  //Runs a motionchain with an arc toward the channel, and a movement to the other side of the goal
  boomerang(22.5, 38, -1, -90, 0.2, 1100, false, 8);
  scraper.set(false);
  moveToPoint(85, 30, -1, 1500, true, 10);

  //Runs a sequence of movements to the long goal, and scores 7 blocks
  turnToAngle(180, 800, true, 12);
  moveToPoint(94, 28, 1, 1600, true, 8);
  turnToAngle(90, 800, true, 12);
  moveToPoint(69, 32, -1, 1000, true, 8);
  hood.spin(reverse, 12, voltageUnits::volt);
  turnToAngle(90, 600, true, 12);
  wait(4000, msec);
  hood.stop();

  //Runs a motionchain with heading correction to the second matchloader, and empties it
  scraper.set(true);
  moveToPoint(98, 30.5, 1, 2000, true, 6);
  correct_angle = 90;
  driveToWall(8.8, 900, 1700, true, 12);
  lower_intake.spin(fwd, 12, voltageUnits::volt);
  wait(100, msec);

  /*Runs a motion back to the goal with an antijam sequence, and scores the 6 matchload 
  blocks for a total of 13
  */
  lower_intake.stop(coast);
  moveToPoint(74, 32.5, -1, 2200, true, 6);
  turnToAngle(90, 600, true, 12);
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  hood.spin(reverse, 12, voltageUnits::volt);
  turnToAngle(90, 600, true, 12);
  wait(4000, msec);
  hood.stop();
  scraper.set(false); 

  //Moves away from the first long goal and moves across the feild to the third matchloader
  moveToPoint(86, 29, 1, 2000, true, 8);
  turnToAngle(180, 1000);
  moveToPoint(90, -66, 1, 3000, true, 9);

  //Runs a motionchain with heading correction into the third matchloader, and empties it
  scraper.set(true);
  turnToAngle(90, 1000);
  moveToPoint(98, -64, 1, 3500, true, 6);
  correct_angle = 90;
  driveToWall(8.8, 900, 1700, true, 12);

  //Runs a motionchain through the channel to the other side of the goal
  moveToPoint(86, -66, -1, 2000, true, 9);
  turnToAngle(40, 1000);
  scraper.set(false);
  moveToPoint(-0, -70, -1, 3000, true, 10);

  //Runs a sequence of movements to the long goal, and scores 6 blocks, and corrects our heading
  turnToAngle(180, 800);
  moveToPoint(-0, -68, -1, 1500, true, 8);
  turnToAngle(-90, 800);
  boomerang(20, -69, -1, 270, 0.5, 1500, true, 8);
  hood.spin(reverse, 12, voltageUnits::volt);
  scraper.set(true);
  turnToAngle(-90, 600);
  wait(4000, msec);
  hood.stop();

  //Runs a motionchain to the final matchloader and picks up 6 blocks
  moveToPoint(-13, -67.8, 1, 3400, true, 6);
  correct_angle = -90;
  driveToWall(8.8, 900, 1700, true, 12);
  
  //Moves back to the long goal and scores 6 blocks, and corrects the heading for park
  moveToPoint(20, -69, -1, 1500, true, 7);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(5000, msec);
  scraper.set(false);
  turnToAngle(-90, 600);
  hood.stop();

  //Runs a motionchain into the park zone, and clears the blocks
  moveToPoint(5, -65, 1, 1500, false, 8);
  boomerang(-21, -32, 1, 0, 0.3, 4600, false, 9);
  moveToPoint(-21, 0, 1, 3000, true, 7);

}

void elimleft(){
  /*Elimintaion left side autonomous, picks up 8 balls and scores them all in the long goal, 
  amd empties the matchloader
  */
  //Runs lower intake and threads ball control commands for the beggining sequence
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){
    doinker(500);
    doinkerup(400);
    doinker(1300);
  }); 

  //Runs movements to pick up the first 5 balls in the sequence, uses the previous thread for ball control
  moveToPoint(-5.3, 20.5, 1, 900, true, 9);
  moveToPoint(-10, 37, 1, 900, true, 9);
  turnToAngle(-90, 600);
  moveToPoint(-24, 37, 1, 900, true, 9);
  wait(200, msec);

  //Runs an arc away from the auto line to prime for the move to the long goal
  boomerang(-5, 20, -1, -30, 0.25, 1500, true, 9);
  wait(100, msec);
  turnToAngle(-135, 500);

  //Runs anti jam before moving to the long goal
  lower_intake.stop(coast);
  wait(100, msec);
  scraper.set(false);
  lower_intake.spin(reverse, 12, voltageUnits::volt);

  //moves into position with heading corrections to align with the long goal
  moveToPoint(-31, 4, 1, 1900, true, 8);
  wait(100, msec);
  turnToAngle(180, 500);

  //Runs another anti jam before the scoring movement
  scraper.set(true);
  lower_intake.stop(coast);
  wait(100, msec);
  lower_intake.spin(reverse, 12, voltageUnits::volt);

  //Final approach to the long goal, and scores the 5 blocks weve picked up so far
  moveToPoint(-28, 20, -1, 1500, true, 9);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(2367, msec);
  hood.stop();

  //Moves to pick up the next 3 blocks from the match loader
  moveToPoint(-27.5, -4, 1, 1000, false, 11);
  correct_angle = 180;
  driveToWall(8.8, 1500, 900, true, 12);

  //Final scoring move, moves into position and scores the last 3 balls
  moveToPoint(-28.5, 21, -1, 1200, false, 10);
  scraper.set(false);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(1100, msec);
  hood.stop();

}

void elimright(){
  /*Elimintaion right side autonomous, picks up 8 balls and scores them all in the long goal,
  empties the matchloader
  */

  //Runs lower intake and threads ball control commands for the beggining sequence
  lower_intake.spin(reverse, 12, voltageUnits::volt);
  thread([](){
    doinker(600);
    doinkerup(400);
    doinker(1400);
  }); 

  //Runs movements to pick up the first 5 balls in the sequence, uses the previous thread for ball control
  moveToPoint(5.3, 23.5, 1, 1500, true, 9); 
  moveToPoint(10, 40, 1, 1500, true, 9);
  turnToAngle(90, 600);
  moveToPoint(35, 47, 1, 1500, true, 9);
  wait(200, msec);

  //Runs an arc away from the auto line to prime for the move to the long goal
  boomerang(14, 22, -1, 30, 0.15, 1900, true, 9);
  wait(200, msec);
  turnToAngle(135, 500);

  //Runs anti jam before moving to the long goal
  lower_intake.stop(coast);
  wait(100, msec);
  scraper.set(false);
  lower_intake.spin(reverse, 12, voltageUnits::volt);

  //moves into position with heading corrections to align with the long goal
  moveToPoint(48, 8, 1, 2000, true, 9);
  wait(100, msec);
  turnToAngle(180, 600);

  //Runs another anti jam before the scoring movement
  scraper.set(true);
  lower_intake.stop(coast);
  wait(100, msec);
  lower_intake.spin(reverse, 12, voltageUnits::volt);

  //Final approach to the long goal, and scores the 5 blocks weve picked up so far
  moveToPoint(48, 18, -1, 1200, true, 9);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(2367, msec);
  hood.stop();

  //Moves to pick up the next 3 blocks from the match loader
  moveToPoint(48.5, -1, 1, 1000, true, 9); 
  correct_angle = 180;
  driveToWall(8.8, 1500, 900, true, 12);

  //Final scoring move, moves into position and scores the last 3 balls
  moveToPoint(48, 20, -1, 900, true, 9);
  hood.spin(reverse, 12, voltageUnits::volt);
  wait(1400, msec);
  hood.stop();
  scraper.set(true);

}