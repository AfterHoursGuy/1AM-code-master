#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include "pid.h"

// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  int auton_selected = 7; // change this to select different autonomous routines
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

bool intaken = false;

void intaker() {
  intaken = true;
  lower_intake.spin(forward, 12, voltageUnits::volt);
  wait(500, msec);
  lower_intake.stop(coast);
  intaken = false;
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
bool wingState = false;

static bool intakeToggle = false;
bool btnPrev = false;
int pressStart = 0;
bool longPress = false;


void runDriver() {
  stopChassis(coast);
  heading_correction = false;

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

    bool btn = controller_1.ButtonL1.pressing();

    // Button pressed this moment
    if (btn && !btnPrev) {
        pressStart = Brain.timer(msec);
        longPress = false;
    }

    // If held long enough → reverse
    if (btn && !longPress && Brain.timer(msec) - pressStart > 200) {
        longPress = true;
    }

    // Handle release
    if (!btn && btnPrev) {
        if (!longPress) {
            // short press → toggle
            intakeToggle = !intakeToggle;
        }
        // if longPress: do nothing → return to toggle state
    }

    // Apply motor behavior
    if (longPress && btn) {
        // reverse while holding long press
        lower_intake.spin(reverse, 12, voltageUnits::volt);
    }
    else if (intakeToggle) {
        // normal toggle state
        lower_intake.spin(forward, 12, voltageUnits::volt);
    }
    else if (!intaken) {
        lower_intake.stop(coast);
    }

    btnPrev = btn;

    // Intake & hood control with middle goal condition
    if (r1) {
      gate.set(false);
      wait(50, msec);
      thread([]{intaker();});
      thread([]{
        fastarmPID(135);
        fastarmPID(1);
        gate.set(true);
      });
      
      
    } else if (r2) {
      if (middleGoalState) {
        thread([]{lower_intake.spin(fwd, 12, voltageUnits::volt);});
        gate.set(false);
    stick.spin(fwd, 20, percent);
      wait(600, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(1100, msec);
  thread([]{fastarmPID(1);
    gate.set(true);
    lower_intake.spin(fwd, 12, voltageUnits::volt);
  });
        /*gate.set(false);
        wait(50, msec);
        thread([]{intaker();});
        thread([]{
          softarmPID(135);
          fastarmPID(1);
          gate.set(true);*/

      } else {
        lower_intake.spin(fwd, 12, voltageUnits::volt);
        gate.set(false);
    stick.spin(fwd, 20, percent);
      wait(600, msec);
  lower_intake.spin(fwd, -12, voltageUnits::volt);
  wait(1100, msec);
  thread([]{fastarmPID(1);
    
    gate.set(true);
    lower_intake.spin(fwd, 12, voltageUnits::volt);
        });
      }
      
    } else {
      stick.stop(hold);
      
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
    }
    rightPrev = button_right_arrow;

    static bool yPrev = false;
    if (button_y && !yPrev) {
    wingState = !wingState;
    wings.set(wingState);
    }
    yPrev = button_y;

    if (button_x) {
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      
      
      
    }

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
  //inertial_sensor.setHeading(starting_heading, degrees);
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

  thread([]{lights(0, 0, 0);});

}