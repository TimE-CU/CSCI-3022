#include <sparki.h>

#define STATE_FIND_OBJ 0
#define STATE_DRIVE_TO_OBJ 1
#define STATE_GRAB_OBJ 2
#define STATE_FIND_LINE 3
#define STATE_FOLLOW_LINE 4
#define STATE_FOUND_START 5

// Set up some global variables with default values to be replaced during operation
int current_state = STATE_FIND_OBJ;
const int threshold = 700; // IR reading threshold to detect whether there's a black line under the sensor
int cm_distance = 1000;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;


void setup() {
  // put your setup code here, to run once:
  sparki.RGB(RGB_RED); // Turn on the red LED
  sparki.servo(SERVO_CENTER); // Center the ultrasonic sensor
  delay(1000); // Give the motor time to turn
  sparki.gripperOpen(); // Open the gripper
  delay(5000); // Give the motor time to open the griper
  sparki.gripperStop(); // 5 seconds should be long enough
  sparki.RGB(RGB_GREEN); // Change LED to green so we know the robot's setup is done!
}

void readSensors() {
  cm_distance = 0; // Replace with code to read the distance sensor
  line_left = 0; // Replace with code to read the left IR sensor
  line_right = 0; // Replace with code to read the right IR sensor
  line_center = 0; // Replace with code to read the center IR sensor
}

void loop() {
  // put your main code here, to run repeatedly:
  readSensors(); // Read sensors once per loop() call

  sparki.clearLCD();
  sparki.print("STATE: ");
  sparki.println(current_state);

  // Your state machine code goes here

  switch(current_state) {
    case STATE_FIND_OBJ:
      // Rotate until object is found
      break;
    case STATE_DRIVE_TO_OBJ:
      // Drive Within 7cm of object
      break;
    case STATE_GRAB_OBJ:
      // Grab object w pinchers
      break;
    case STATE_FIND_LINE:
      // Turn 180 and drive until a line is found
      break;
    case STATE_FOLLOW_LINE:
      // Follow the line until "start"
      break;
    case STATE_FOUND_START:
      // beep and drop object
      break;
    default:
      break;
  }

  sparki.updateLCD();
  delay(100); // Only run controller at 10Hz
}
