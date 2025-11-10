#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"

// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  int auton_selected = 3; // change this to select different autonomous routines
  switch(auton_selected) {
    case 1:
      exampleAuton();
      break;
    case 2:
      exampleAuton2();
      break;  
    case 3:
      leftsidequal();
      break;
    case 4:
      rightsidequal();
      break; 
    case 5:
      sigsoloAWP();
      break;
    case 6:
      qualsoloawp();
      break;
    case 7:
      skills();
      break;
    case 8:
      elimleft();
      break;
    case 9:
    elimright();
      break;
  }
}

// Slight exponential scaling for joystick input
double expoDrive(int input, double expo = 1.3) {
  // normalize to [-1, 1]
  double x = input / 100.0;  
  // apply exponential curve (expo > 1 means softer near center)
  double scaled = std::copysign(std::pow(std::fabs(x), expo), x);
  // return back to [-100, 100]
  return scaled * 100.0;
}

// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;

// pneumatic control
bool scraperState = false;
bool middleGoalState = false;
bool parkPistonState = false;
bool hoodLimiterState = false;
bool phoodState = false;

// loading toggle L1
static bool l1Prev = false;
static bool intakeToggle = false;

const int BLOCK_COUNT_TARGET = 7;
const int DEBOUNCE_TIME_MS = 250; // delay to prevent double-counts
int blockCount = 0;

void block_count() {
  block_counter.setLightPower(100); // turn on LED for block counter
  bool blockPreviouslyDetected = false;

  while (true) {
    // detect a new block when sensor sees one
    if (block_counter.isNearObject() && !blockPreviouslyDetected) {
      blockPreviouslyDetected = true;

      if (blockCount < BLOCK_COUNT_TARGET) {
        ++blockCount;

        if (blockCount >= BLOCK_COUNT_TARGET) {
          // reached target — take action and reset counter
          hood.stop(coast);
          wait(2000,msec);        
          blockCount = 0;         
        }
      }

      wait(DEBOUNCE_TIME_MS, msec);
    }

    // reset detection when block leaves sensor view
    if (!block_counter.isNearObject()) {
      blockPreviouslyDetected = false;
    }
  }
}


void runDriver() {
  stopChassis(coast);
  heading_correction = false;
  thread([](){block_count();});

  while (true) {
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // true/false for controller button presses
    l1 = controller_1.ButtonL1.pressing();
    l2 = controller_1.ButtonL2.pressing();
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_x = controller_1.ButtonX.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();

    // default tank drive or replace it with your preferred driver code here: 
    //driveChassis(ch3 * 0.12, ch2 * 0.12);
    // Apply slight exponential scaling to joysticks with expoDrive
    double leftPower  = expoDrive(ch3, 1.4) * 0.12;  
    double rightPower = expoDrive(ch2, 1.4) * 0.12;  
    driveChassis(leftPower, rightPower);

      if (l1 && !l1Prev) {  // rising edge detection
        intakeToggle = !intakeToggle;
      }
      l1Prev = l1;

    // Intake & hood control with middle goal condition
    if (r1) {
        lower_intake.spin(reverse, 12, voltageUnits::volt);
        hood.spin(reverse, 12, voltageUnits::volt);
      
    } else if (r2) {
      // R2 → reverse both
      lower_intake.spin(fwd, 12, voltageUnits::volt);
      hood.spin(fwd, 12, voltageUnits::volt);
    } else if (intakeToggle) {
      // L1 toggle ON → forward lower intake only
      lower_intake.spin(reverse, 12, voltageUnits::volt);
      hood.stop(hold);
    } else {
      // Nothing pressed → stop both
      lower_intake.stop(coast);
      hood.stop(hold);
    }

    // Scraper toggle on L2
    static bool l2Prev = false;
    if (l2 && !l2Prev) {   // button press event
      scraperState = !scraperState;
      scraper.set(scraperState);
    }
    l2Prev = l2;

    // Park toggle on A
    static bool aPrev = false;
    if (button_a && !aPrev) {
      parkPistonState = !parkPistonState;
      park.set(parkPistonState);
    }
    aPrev = button_a;

    // Right arrow toggle for Middle goal
    static bool rightPrev = false;
    if (button_right_arrow && !rightPrev) {
    middleGoalState = !middleGoalState;
    mid_goal.set(middleGoalState);
    blockCount = 0;
    }
    rightPrev = button_right_arrow;

    wait(10, msec); 

  }
}


void runPreAutonomous() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }

  double current_heading = inertial_sensor.heading();
  Brain.Screen.print(current_heading);
  
  // odom tracking
  resetChassis();
  if(using_horizontal_tracker && using_vertical_tracker) {
    thread odom = thread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    thread odom = thread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    thread odom = thread(trackYOdomWheel);
  } else {
    thread odom = thread(trackNoOdomWheel);
  }

}