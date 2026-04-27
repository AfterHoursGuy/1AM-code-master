#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include "pid.h"

// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  int auton_selected = 9; // change this to select different autonomous routines
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
      elimleft(); // 7 Ball Active
      break;
    case 9:
      elimright(); // 7 Ball Active
      break;
  }
}

bool intaken = false;


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
bool wingFState = false;
static bool swapPrev = false;

//static bool intakeToggle = false;
bool btnPrev = false;
int pressStart = 0;
bool longPress = false;

bool r1Prev = false;
uint32_t lastR1ReleaseTime = 0;
bool isDoublePress = false;
//const int doubleClickWindow = 250; // Milliseconds to wait for second click

// --- L1 Toggle & Long Press Variables ---
bool intakeToggle = false;    // Keeps track of whether the intake is "on"
bool btnPrevL1 = false;      // Stores the state of L1 from the previous loop
uint32_t pressStartL1 = 0;   // The timestamp when L1 was first pressed
bool longPressL1 = false;    // Flag to indicate if L1 has been held long enough


void runDriver() {
  //runAutonomous();

  stopChassis(coast);
  heading_correction = false;
  intake_lift.set(true);
  descore.set(true);

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



/*bool btnL1 = controller_1.ButtonL1.pressing();
bool btnR1 = controller_1.ButtonR1.pressing();

// --- L1 Logic (Toggle/Reverse) ---
if (btnL1 && !btnPrevL1) {
    pressStartL1 = Brain.timer(msec);
    longPressL1 = false;
}
if (btnL1 && !longPressL1 && Brain.timer(msec) - pressStartL1 > 200) {
    longPressL1 = true;
}
if (!btnL1 && btnPrevL1) {
    if (!longPressL1) intakeToggle = !intakeToggle;
}
btnPrevL1 = btnL1;

// --- R1 Double Press Logic ---
if (btnR1 && !r1Prev) {
    // If we press again within the window, it's a double press
    if (Brain.timer(msec) - lastR1ReleaseTime < doubleClickWindow) {
        isDoublePress = true;
    } else {
        isDoublePress = false;
    }
}
if (!btnR1 && r1Prev) {
    lastR1ReleaseTime = Brain.timer(msec);
}
r1Prev = btnR1;

// --- Combined Motor Behavior ---

if (longPressL1 && btnL1) {
    // REVERSE (L1 Hold)
    intake_lift.set(false);
    lower_intake.spin(reverse, 5, volt); 
    upper_intake.spin(reverse, 7, volt);
} 
else if (btnR1 && isDoublePress) {
    // R2 COMMANDS (Triggered by R1 Double Press + Hold)
    hood.set(false);
    mid_goal.set(true);
    lower_intake.spin(forward, 10, volt); 
    upper_intake.spin(forward, 35, pct);
}
else if (btnR1 && !isDoublePress) {
    // R1 COMMANDS (Triggered by R1 Single Press + Hold)
    hood.set(true);
    mid_goal.set(false);
    lower_intake.spin(forward, 12, volt);
    upper_intake.spin(forward, 12, volt);
}
else if (intakeToggle) {
    // NORMAL TOGGLE (L1 Short Press)
    mid_goal.set(false);
    hood.set(false);
    intake_lift.set(true);
    lower_intake.spin(forward, 12, volt);
    upper_intake.spin(forward, 12, volt);
} 
else {
    // DEFAULT IDLE (Nothing pressed/toggled)
    mid_goal.set(false);
    hood.set(false);
    intake_lift.set(true);
    lower_intake.stop(coast);
    upper_intake.stop(coast);
} */

bool btnL1 = controller_1.ButtonL1.pressing();
bool btnR1 = controller_1.ButtonR1.pressing();
bool btnR2 = controller_1.ButtonR2.pressing(); // Added R2 check

// --- L1 Logic (Toggle/Reverse) ---
if (btnL1 && !btnPrevL1) {
    pressStartL1 = Brain.timer(msec);
    longPressL1 = false;
}
if (btnL1 && !longPressL1 && Brain.timer(msec) - pressStartL1 > 200) {
    longPressL1 = true;
}
if (!btnL1 && btnPrevL1) {
    if (!longPressL1) intakeToggle = !intakeToggle;
}
btnPrevL1 = btnL1;

// --- Combined Motor Behavior ---

if (longPressL1 && btnL1) {
    // REVERSE (L1 Hold)
    intake_lift.set(false);
    lower_intake.spin(reverse, 12, volt); //5
    upper_intake.spin(reverse, 12, volt); //7
} 
else if (btnR2) {
    // R2 COMMANDS (Now triggered by a simple R2 hold)
    hood.set(false);
    mid_goal.set(true);
    lower_intake.spin(forward, 12, volt); // 10
    upper_intake.spin(forward, 100, pct); // 35
}
else if (btnR1) {
    // R1 COMMANDS (Triggered by R1 Hold)
    hood.set(true);
    mid_goal.set(false);
    lower_intake.spin(forward, 12, volt);
    upper_intake.spin(forward, 12, volt);
}
else if (intakeToggle) {
    // NORMAL TOGGLE (L1 Short Press)
    mid_goal.set(false);
    hood.set(false);
    intake_lift.set(true);
    lower_intake.spin(forward, 12, volt);
    upper_intake.spin(forward, 12, volt);
} 
else {
    // DEFAULT IDLE
    mid_goal.set(false);
    hood.set(false);
    intake_lift.set(true);
    lower_intake.stop(coast);
    upper_intake.stop(coast);
}
    
    
    // Scraper toggle on L2
    static bool yPrev = false;
    if (button_y && !yPrev) {   // button press event
      scraperState = !scraperState;
      scraper.set(scraperState);
    }
    yPrev = button_y;

    

    // Right arrow toggle for Middle goal
    static bool rightPrev = false;
    if (button_right_arrow && !rightPrev) {
    middleGoalState = !middleGoalState;
    descore.set(middleGoalState);
    }
    rightPrev = button_right_arrow;

    static bool downPrev = false;
    if (button_down_arrow && !downPrev) {
    wingFState = !wingFState;
    wingF.set(wingFState);
    }
    downPrev = button_down_arrow;

    static bool bPrev = false;
    if (button_b && !bPrev) {
    wingState = !wingState;
    wings.set(wingState);
    }
    bPrev = button_b;

    if (l2 && !swapPrev) {
    // Flip both states
    wingFState = !wingFState;
    wingState = !wingFState; // Set wingState to the opposite of the new wingFState

    // Apply to the solenoids/motors
    wingF.set(wingFState);
    wings.set(wingState);
}
swapPrev = l2;

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

  thread([]{lights(255, 255, 0);});

}