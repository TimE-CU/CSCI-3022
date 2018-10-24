// UPDATED 10/10
#include <sparki.h>

#define ROBOT_SPEED 0.0278
#define MIN_CYCLE_TIME .100
#define AXLE_DIAMETER 0.0865
#define FWD 1
#define NONE 0
#define BCK -1

// Screen size
#define SCREEN_X_RES 128.
#define SCREEN_Y_RES 64.

// Map size
#define NUM_X_CELLS 12
#define NUM_Y_CELLS 6

// Start line is 18", 2" from bottom left corner
#define START_LINE_X .4572 
#define START_LINE_Y .0508 

#define SERVO_POS_DEG 45

int current_state = 1;
const int threshold = 800;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;
float distance = 0.;
unsigned long last_cycle_time = 0;

float pose_x = 0., pose_y = 0., pose_theta = 0., pose_servo = 0.;
int left_wheel_rotating = 0, right_wheel_rotating = 0;

// TODO: Define world_map multi-dimensional array

int disp_arr[NUM_X_CELLS][NUM_Y_CELLS];

// TODO: Figure out how many meters of space are in each grid cell
const float CELL_RESOLUTION_X = 0.60 / NUM_X_CELLS;  // Line following map is ~60cm x ~42cm
const float CELL_RESOLUTION_Y = 0.42 / NUM_Y_CELLS; // Line following map is ~60cm x ~42cm


void setup() {
  pose_x = START_LINE_X;
  pose_y = START_LINE_Y;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
  pose_servo = to_radians(SERVO_POS_DEG);

  sparki.servo(-to_degrees(pose_servo)); // Servo axis points down instead of up, so we need to negate it to be consistent with the robot's rotation axis

  // TODO: Initialize world_map

  sparki.clearLCD();
  displayMap();
  delay(1000);  
  last_cycle_time = millis();
  for(int i = 0; i<NUM_X_CELLS; i++){
    for(int j = 0; j<NUM_Y_CELLS; j++){
      disp_arr[i][j] = 0;
    }
  }
}

float to_radians(float deg) {
  return deg * 3.14159 / 180.;
}

float to_degrees(float rad) {
  return rad * 180. / 3.14159;
}

// Ultrasonic Sensor Readings -> Robot coordinates
void transform_us_to_robot_coords(float dist, float theta, float *rx, float *ry) {
  // TODO
  *rx = cos(theta) * dist;
  *ry = sin(theta) * dist;
}

// Robot coordinates -> World frame coordinates
void transform_robot_to_world_coords(float x, float y, float *gx, float *gy) {
  // TODO
  *gx = x + pose_x;
  *gy = y + pose_y;
}

bool transform_xy_to_grid_coords(float x, float y, int *i, int *j) {
  // TODO: Set *i and *j to their corresponding grid coords  
  *i = float(x / .6 * (NUM_X_CELLS-1));
  *j = float(y / .42 * (NUM_Y_CELLS-1));

  // TODO: Return 0 if the X,Y coordinates were out of bounds
  if(*i >= NUM_X_CELLS || *j >= NUM_Y_CELLS || *i < 0 || *j < 0)
  {
    return 0;
  }
  return 1;
}

// Turns grid coordinates into world coordinates (grid centers)
bool transform_grid_coords_to_xy(int i, int j, float *x, float *y) {
  // TODO: Return 0 if the I,J coordinates were out of bounds
  if(i >= NUM_X_CELLS || j >= NUM_Y_CELLS || i < 0 || j < 0)
  {
    return 0;
  }

  // TODO: Set *x and *y
  *x = float(i) *.6 / float(NUM_X_CELLS-1);
  *y = float(j) *.42 / float(NUM_Y_CELLS-1);
  return 1;
}

bool transform_world_to_screen(float x, float y, float *sx, float *sy)
{
  *sx = (x / 0.60) * SCREEN_X_RES;
  *sy = (y / 0.42) * SCREEN_Y_RES;
  if(*sx < 0 || *sx >= SCREEN_X_RES || *sy < 0 || *sy >= SCREEN_Y_RES)
  {
    return 0;
  }
  return 1;
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  distance = float(sparki.ping()) / 100.;
}

void moveRight() {
  left_wheel_rotating = FWD;
  right_wheel_rotating = BCK;
  sparki.moveRight();
}

void moveLeft() {
  left_wheel_rotating = BCK;
  right_wheel_rotating = FWD;
  sparki.moveLeft();
}

void moveForward() {
  left_wheel_rotating = FWD;
  right_wheel_rotating = FWD;
  sparki.moveForward();
}

void moveBackward() {
  left_wheel_rotating = BCK;
  right_wheel_rotating = BCK;
  sparki.moveBackward();
}

void moveStop() {
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
  sparki.moveStop();
}

void updateOdometry(float cycle_time) {
  pose_x += cos(pose_theta) * cycle_time * ROBOT_SPEED * (left_wheel_rotating + right_wheel_rotating)/2.;
  pose_y += sin(pose_theta) * cycle_time * ROBOT_SPEED * (left_wheel_rotating + right_wheel_rotating)/2.;
  pose_theta += (right_wheel_rotating - left_wheel_rotating) * cycle_time * ROBOT_SPEED / AXLE_DIAMETER;
}

void displayMap() {
  // TODO: Measure how many pixels will be taken by each grid cell
  const int PIXELS_PER_X_CELL = 0;
  const int PIXELS_PER_Y_CELL = 0; 
  int cur_cell_x=-1, cur_cell_y=-1;

  // TODO: Make sure that if the robot is "off-grid", e.g., at a negative grid position or somewhere outside your grid's max x or y position that you don't try to plot the robot's position!
  float sx, sy;
  if(transform_world_to_screen(pose_x, pose_y, &sx, &sy))
  {
//    sparki.print(sx);
    sparki.drawCircleFilled(sx, sy, 2);
  }
  // TODO: Draw Map
  for(int i = 0; i<NUM_X_CELLS; i++){
    for(int j = 0; j<NUM_Y_CELLS; j++){
      transform_grid_coords_to_xy(i, j, &sx, &sy);
      transform_world_to_screen(sx, sy, &sx, &sy);
      if(disp_arr[i][j] == 1){
        sparki.drawCircleFilled(sx, sy, 5);
      }
      else{
        sparki.drawCircle(sx, sy, 5);
      }
    }
  }
}

void serialPrintOdometry() {
  Serial.print("\n\n\nPose: ");
  Serial.print("\nX: ");
  Serial.print(pose_x);
  Serial.print("\nY: ");
  Serial.print(pose_y);
  Serial.print("\nT: ");
  Serial.print(pose_theta * 180. / 3.14159);
  Serial.print("\n");
}

void displayOdometry() {
  sparki.println("Pose: ");
  sparki.print("X: ");
  sparki.println(pose_x);
  sparki.print("Y: ");
  sparki.println(pose_y);
  sparki.print("T: ");
  sparki.println(pose_theta * 180. / 3.14159);
}

int get_cell_index(int x, int y)
{
  return (x * NUM_X_CELLS) + y;
}

void get_index_pos(int idx, int* x, int* y)
{
  *x = idx / NUM_X_CELLS;
  *y = idx % NUM_X_CELLS;
}

// Cost to move from idx1 to idx2
int get_cost(int idx1, int idx2)
{
  int x1, y1, x2, y2;
  get_index_pos(idx1, &x1, &y1);
  get_index_pos(idx2, &x2, &y2);
  if(disp_arr[x2][y2])
  {
    return 999;
  }
  
  int dist = abs(x1-x2) + abs(y1+y2);
  if(dist != 1)
  {
    return 999;
  }

  return 1;
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long begin_movement_time = 0;
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
  float elapsed_time = 0.;
  bool found_object = 0;
  readSensors();
  
  sparki.clearLCD();
  last_cycle_time = (millis() - last_cycle_time);
  elapsed_time = last_cycle_time / 1000.;
  updateOdometry(elapsed_time);
  last_cycle_time = millis(); // Start timer for last motor command to determine cycle time
  serialPrintOdometry();

  // Mapping Code
  sparki.servo(-to_degrees(pose_servo));

  // TODO: Check if sensors found an object

  float cm_distance = sparki.ping(); 
  float sx, sy;
  int px, py;
  
  // TODO: Adjust Map to accommodate new object

  if(cm_distance <= 30 && cm_distance != -1) {
    transform_us_to_robot_coords(cm_distance/100, pose_theta+to_radians(SERVO_POS_DEG), &sx, &sy);
    transform_robot_to_world_coords(sx, sy, &sx, &sy);
    transform_xy_to_grid_coords(sx, sy, &px, &py);
    disp_arr[px][py] = 1;
  }

//  displayOdometry();
  displayMap();

  if (line_center < threshold) {
    moveForward();
  } else if (line_left < threshold) {
    moveLeft();
  } else if (line_right < threshold) {
    moveRight();
  } else {
    moveStop();
  }
  
  // Check for start line, use as loop closure
  // NOTE: Assumes robot is moving counter-clockwise around the map (by setting pose_theta = 0)!
  //       If your robot is moving clockwise, set pose_theta to pi radians (i.e., pointing left).
  if (line_left < threshold && line_right < threshold && line_center < threshold) {
    pose_x = START_LINE_X;
    pose_y = START_LINE_Y;
    pose_theta = 0.;
  } 

  sparki.updateLCD();
  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*MIN_CYCLE_TIME)
    delay(1000*MIN_CYCLE_TIME - delay_time); // make sure each loop takes at least MIN_CYCLE_TIME ms
  else
    delay(10);
}
