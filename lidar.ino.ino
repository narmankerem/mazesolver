#include <Encoder.h>
#include <SharpIR.h>
#include <ros.h>
#include <ArduinoHardware.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>


//ROS
ros::NodeHandle nh;

geometry_msgs::Pose2D pose;
std_msgs::Float32 frontRange;
std_msgs::Float32 rightRange;
std_msgs::Float32 leftRange;
std_msgs::Float32 odom_x;
std_msgs::Float32 odom_y;

ros::Publisher front_dist_pub("/front_distance", &frontRange);
ros::Publisher right_dist_pub("/right_distance", &rightRange);
ros::Publisher left_dist_pub("/left_distance", &leftRange);
ros::Publisher x_pub("/odom_x", &odom_x);
ros::Publisher y_pub("/odom_y", &odom_y);

float linear_vel, angular_vel;
void cmdVelCallback(const geometry_msgs::Twist& msg) {
     linear_vel = msg.linear.x;
     angular_vel = msg.angular.z;
}


ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmdVelCallback);





/*************************************************************************************************************************/
// Define the pins for left and right encoder interrupts
#define LEFT_ENCODER_PIN_A 19
#define LEFT_ENCODER_PIN_B 18
#define RIGHT_ENCODER_PIN_A 2
#define RIGHT_ENCODER_PIN_B 3

// Define the range sensor pins
#define LEFT_RANGE_SENSOR_PIN A2
#define RIGHT_RANGE_SENSOR_PIN A4
#define FRONT_RANGE_SENSOR_PIN A3

// Define the motor pin numbers
#define LEFT_MOTOR_PIN_1 6
#define LEFT_MOTOR_PIN_2 9
#define RIGHT_MOTOR_PIN_1 5
#define RIGHT_MOTOR_PIN_2 4

// Define wheel parameters
#define rad 0.02
#define axisL 0.09
#define Cpr 8300 // 1 wheel turn encoder approximativelly 

// Create encoder objects for left and right wheels
Encoder encL(LEFT_ENCODER_PIN_A, LEFT_ENCODER_PIN_B);
Encoder encR(RIGHT_ENCODER_PIN_A, RIGHT_ENCODER_PIN_B);

// Define the sensor types and models
SharpIR sensorL(SharpIR::GP2Y0A21YK0F, LEFT_RANGE_SENSOR_PIN);
SharpIR sensorR(SharpIR::GP2Y0A21YK0F, RIGHT_RANGE_SENSOR_PIN);
SharpIR sensorF(SharpIR::GP2Y0A21YK0F, FRONT_RANGE_SENSOR_PIN);

// Variables to store previous encoder counts
long prevEncoderCountL = 0;
long prevEncoderCountR = 0;

// Variables for PID
float Kp=20;
float Kd=0;
float Ki=100;

// Variables for timing
unsigned long previousTime = 0;
const unsigned long interval = 100; // Interval of 100ms for 10Hz frequency


/*************************************************************************************************************************/
// Function to read the left range sensor and convert measurements to millimeters
float sensL() {
  int sensorValue = sensorL.getDistance();
  // Convert sensor reading to millimeters using appropriate conversion formula
  float distance_mm = sensorValue * 10; // Assuming the sensor output is in cm
  return distance_mm;
}

// Function to read the right range sensor and convert measurements to millimeters
float sensR() {
  int sensorValue = sensorR.getDistance();
  // Convert sensor reading to millimeters using appropriate conversion formula
  float distance_mm = sensorValue * 10; // Assuming the sensor output is in cm
  return distance_mm;
}

// Function to read the front range sensor and convert measurements to millimeters
float sensF() {
  int sensorValue = sensorF.getDistance();
  // Convert sensor reading to millimeters using appropriate conversion formula
  float distance_mm = sensorValue * 10; // Assuming the sensor output is in cm
  return distance_mm;
}
/*************************************************************************************************************************/


// Function to move the robot forward
void moveForward(int leftSpeed, int rightSpeed) {

  if(leftSpeed<0){
  digitalWrite(LEFT_MOTOR_PIN_1, HIGH);
  analogWrite(LEFT_MOTOR_PIN_2, abs(leftSpeed));
  }
  else{
  digitalWrite(LEFT_MOTOR_PIN_1, LOW);
  analogWrite(LEFT_MOTOR_PIN_2, leftSpeed);
  }

  if(rightSpeed<0){
  digitalWrite(RIGHT_MOTOR_PIN_1, HIGH);
  analogWrite(RIGHT_MOTOR_PIN_2, abs(rightSpeed));
  }
  else{
  digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
  analogWrite(RIGHT_MOTOR_PIN_2, rightSpeed);
  }
}


/*************************************************************************************************************************/
// Function to read the absolute values of encoders
void readEncoders(long& encoderCountL, long& encoderCountR) {
  encoderCountL = abs(encL.read());
  encoderCountR = abs(encR.read());
}

// Function to calculate the difference in encoder counts since the last request
float *getEncoderDifference() {
  long currentEncoderCountL, currentEncoderCountR;
  readEncoders(currentEncoderCountL, currentEncoderCountR);
  
  float differenceL = currentEncoderCountL - prevEncoderCountL;
  float differenceR = currentEncoderCountR - prevEncoderCountR;
  
  // Update the previous encoder counts
  prevEncoderCountL = currentEncoderCountL;
  prevEncoderCountR = currentEncoderCountR;

  // Calculate and return the average difference in encoder counts
  static float Difference[2];//={differenceL,differenceR}
  Difference[0]=differenceL; 
  Difference[1]=differenceR;
  return Difference;
}

/*************************************************************************************************************************/
float *poseUpdate(int Nl, int Nr, float r, float b, int C)
{
  static float robPose[5] = {0.0, 0.0, 0.0 ,0.0 ,0.0};
  float Dr = (2.0*PI*r*Nr/C);
  float Dl = (2.0*PI*r*Nl/C);
  float V = (2.0*PI*r/C ) * ((Nr+Nl)/2) * (1/0.1);
  float W = (2.0*PI*r/C ) * ((Nr-Nl)/b) * (1/0.1);
  float D = (Dr + Dl)/2.0;
  robPose[2] = atan2(sin(robPose[2] + W*0.1), cos(robPose[2] + W*0.1));//theta angle
  robPose[0] = robPose[0] + D*V*cos(robPose[2])*0.1;//x
  robPose[1] = robPose[1] + D*V*sin(robPose[2])*0.1;//y
  robPose[3]= V;
  robPose[4]= W;
  
  return robPose;
}

float *cmd_vel(float r, float b){
  //float V = 0.102; // max = 0.102
  //float V = 0.056; // half = 0.102
  //float W = 0.0 ; 

  float wd_l = (linear_vel-(b/2.0)*angular_vel)/r;
  float wd_r = (linear_vel+(b/2.0)*angular_vel)/r;

  static float Wd_l_r[2] ;
  Wd_l_r[0] = wd_l;
  Wd_l_r[1] = wd_r;

  return Wd_l_r;  
}

float *cmd_vel2wheels(float V, float W,float r, float b){

  float wl = (V-(b/2.0)*W)/r;
  float wr = (V+(b/2.0)*W)/r;

  static float W_l_r[2];
  W_l_r[0] = wl;
  W_l_r[1] = wr;

  return W_l_r;
}

float *pid_controller (float *Wd_l_r, float *W_l_r){

  static float prop_error[2]= {0,0} ;
  static float deri_error[2]= {0,0} ;
  static float int_error[2] = {0,0};
  static float G_l_r[2] = {0,0};

 
  
  deri_error[0] = (Wd_l_r[0] - W_l_r[0]) - prop_error[0] ; //deri_l_error+1
  deri_error[1] = (Wd_l_r[1] - W_l_r[1]) - prop_error[1]; //deri_r_error+1

  prop_error[0] = Wd_l_r[0] - W_l_r[0] ; //prop_l_error+1
  prop_error[1] = Wd_l_r[1] - W_l_r[1]; //prop_r_error+1

  int_error[0] = int_error[0] + prop_error[0] ; //int_l_error+1
  int_error[1] = int_error[1] + prop_error[1]; //int_r_error+1

  if(Wd_l_r[0]==0  ){
    deri_error[0]=0;
    prop_error[0]=0;
    int_error[0]=0;
  }

  if(Wd_l_r[1]==0  ){
    deri_error[1]=0;
    prop_error[1]=0;
    int_error[1]=0;
  }

  G_l_r[0] = Kp * prop_error[0] + Ki * int_error[0] * 0.1 + Kd * (deri_error[0]/0.1); //G_l+1
  G_l_r[1] = Kp * prop_error[1] + Ki * int_error[1] * 0.1 + Kd * (deri_error[1]/0.1); //G_r+1

  
  // Protection  for value > abs(255)
  if(G_l_r[0]>=255) G_l_r[0]=255;
  if(G_l_r[0]<=-255) G_l_r[0]=-255;

  if(G_l_r[1]>=255) G_l_r[1]=255;
  if(G_l_r[1]<=-255) G_l_r[1]=-255;
  
  return G_l_r;
}

/*************************************************************************************************************************/

void setup() {
  // Set up the encoder pins as inputs with internal pull-up resistors
  pinMode(LEFT_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN_B, INPUT_PULLUP);
  
  // Set up motor control pins as outputs
  pinMode(LEFT_MOTOR_PIN_1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN_2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_2, OUTPUT);
  
  // Set up serial communication if needed
  
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(front_dist_pub);
  nh.advertise(right_dist_pub);
  nh.advertise(left_dist_pub);
  nh.advertise(x_pub);
  nh.advertise(y_pub);
  
}

void loop() {
  // Check if the desired interval has passed
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= interval) {

    // Update the previous time
    previousTime = currentTime;
    
    // Read the range sensor values
    leftRange.data= sensL();
    rightRange.data = sensR();
    frontRange.data = sensF();
    
    // Read the encoder difference and print it
    float *encoderDifference = getEncoderDifference();
    
    //pose_pub.publish(&pose);
    front_dist_pub.publish(&frontRange);
    right_dist_pub.publish(&rightRange);
    left_dist_pub.publish(&leftRange);
    
    
    float *robPose = poseUpdate(encoderDifference[0], encoderDifference[1], rad, axisL, Cpr);
    float *cmd= cmd_vel(rad, axisL);
    float *vel2wheels = cmd_vel2wheels(robPose[3],robPose[4],rad, axisL);
    float *G =pid_controller (cmd, vel2wheels);

    odom_x.data = robPose[0];
    odom_y.data = robPose[1];
    x_pub.publish(&odom_x);
    y_pub.publish(&odom_y);
    
    moveForward((int) G[0], (int) G[1]);
    
  }
  nh.spinOnce();
}

