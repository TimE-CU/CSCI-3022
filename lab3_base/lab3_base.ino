#include <sparki.h>

#define M_PI 3.14159
#define ROBOT_SPEED 0.0275 // meters/second 
#define CYCLE_TIME .050 // Default 50ms cycle time
#define AXLE_DIAMETER 0.0857 // meters
#define WHEEL_RADIUS 0.03 // meters
#define CONTROLLER_FOLLOW_LINE 1 
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3

#define FWD 1
#define NONE 0
#define BCK -1


// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

// Controller and dTheta update rule settings
int current_state = CONTROLLER_GOTO_POSITION_PART3;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (rad), heading error (rad)
float phi_l = 0., phi_r = 0.; // Wheel rotation (radians)


// Wheel rotation vars
float left_speed_pct = 0.;
float right_speed_pct = 0.;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;
int left_wheel_rotating = NONE;
int right_wheel_rotating = NONE;

// X and Theta Updates (global for debug output purposes)
// and their respective feedback controller gains
const float distance_gain = 0.1;
const float alpha_gain  = 0.1;
const float beta_gain = 0.001;
float dX  = 0., dTheta = 0.;

float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180 / 3.1415;
}

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;

  // Set test cases here!
  set_pose_destination(10, 10, to_radians(-90));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Degrees
  orig_dist_to_goal = sqrt(sq(dest_pose_x - pose_x) + sq(dest_pose_y - pose_y));
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta < -M_PI) dest_pose_theta += 2*M_PI;
  orig_dist_to_goal = 0; // TODO
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}


void updateOdometry() {
  phi_l = left_speed_pct * (ROBOT_SPEED * CYCLE_TIME);
  phi_r = right_speed_pct * (ROBOT_SPEED * CYCLE_TIME);

  dX = ((phi_l + phi_r)/2) * 100; // convert to cm
  dTheta = (phi_r - phi_l)/(AXLE_DIAMETER);

  pose_theta += dTheta;
  pose_x += dX*cos(pose_theta);
  pose_y += dX*sin(pose_theta);

  d_err = sqrt(sq(dest_pose_x - pose_x) + sq(dest_pose_y - pose_y)) / orig_dist_to_goal;
  b_err = atan2((dest_pose_y - pose_y),(dest_pose_x - pose_x)) - pose_theta;
  h_err = dest_pose_theta - pose_theta;

  if(abs(d_err) < 0.05)
  {
    d_err = 0;
    b_err = 0;
  }
  if(abs(h_err) < 0.1)
  {
    h_err = 0;
  }
  if(d_err == 0 && b_err == 0 && h_err == 0)
  {
    current_state = -1;
  }
  
}


void displayOdometry() {
  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y); 
  sparki.print("T: ");
  sparki.print(pose_theta);
  sparki.print(" Tg: ");
  sparki.println(dest_pose_theta);

  sparki.print("dX : ");
  sparki.print(dX );
  sparki.print("   dT: ");
  sparki.println(dTheta);
  sparki.print("phl: "); sparki.print(phi_l); sparki.print(" phr: "); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.println(b_err);
  sparki.print("h: "); sparki.println(h_err);  
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
   
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Useful for testing odometry updates
      readSensors();
      if (line_center < threshold) {
        // TODO: Fill in odometry code
        sparki.moveForward();
      } else if (line_left < threshold) {
        // TODO: Fill in odometry code
        sparki.moveLeft();
      } else if (line_right < threshold) {
        // TODO: Fill in odometry code
        sparki.moveRight();
      } else {
        sparki.moveStop();
      }

      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold) {
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = 0.;
      } 
      break;
    case CONTROLLER_GOTO_POSITION_PART2:
      // TODO: Implement solution using moveLeft, moveForward, moveRight functions
      // This case should arrest control of the program's control flow (taking as long as it needs to, ignoring the 100ms loop time)
      // and move the robot to its final destination
      
      updateOdometry();
      
      if(b_err >= 0) {
        sparki.moveLeft(to_degrees(abs(b_err)));
      } else {
        sparki.moveRight(to_degrees(abs(b_err)));
      }
      
      sparki.moveForward(d_err);
      
      if(h_err >= 0) {
        sparki.moveLeft(to_degrees(abs(h_err)));
      } else {
        sparki.moveRight(to_degrees(abs(h_err)));
      }
      
      break;
    case CONTROLLER_GOTO_POSITION_PART3:      
      updateOdometry();

      dX = distance_gain*d_err;
      dTheta  = (alpha_gain*b_err)+(beta_gain*h_err);

      left_speed_pct = dX - dTheta;
      right_speed_pct = dX + dTheta;

      if(abs(left_speed_pct) > abs(right_speed_pct))
      {
         left_speed_pct = left_speed_pct / abs(left_speed_pct);
         right_speed_pct = right_speed_pct / abs(left_speed_pct);
      }
      else{
         left_speed_pct = left_speed_pct / abs(right_speed_pct);
         right_speed_pct = right_speed_pct / abs(right_speed_pct);
      }

      if(left_speed_pct > 0) {
        left_dir = DIR_CCW;
      } else {
        left_dir = DIR_CW;
      }

      if(right_speed_pct > 0) {
        right_dir = DIR_CW;
      } else {
        right_dir = DIR_CCW;
      }

      sparki.motorRotate(0, left_dir, int(abs(left_speed_pct * 100)));
      sparki.motorRotate(1, right_dir, int(abs(right_speed_pct * 100)));
      
      break;
  }

  sparki.clearLCD();
  displayOdometry();
  sparki.updateLCD();

  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
}
