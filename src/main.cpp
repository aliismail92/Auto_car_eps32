#include <Arduino.h>
#include <NewPing.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>

Servo servo;
ESP32Encoder encoder_r;
ESP32Encoder encoder_l;

//Defining UltraSonic Sensors
const int frontSen_trig = 12;
const int frontSen_echo = 14;

NewPing sonar_front(frontSen_trig, frontSen_echo, 100);

const int rightSen_trig = 27;
const int rightSen_echo = 26;

NewPing sonar_right(rightSen_trig, rightSen_echo, 100);

const int leftSen_trig = 2;
const int leftSen_echo = 15;

NewPing sonar_left(leftSen_trig, leftSen_echo, 100);

//Defining Servo signal
const int Servo_sig = 18;

//Defining Motors
const int PWM_A = 4;
const int PWM_B = 16;

const int DIR_A = 17;
const int DIR_B = 5;

uint32_t freq = 4000;
uint32_t resolution = 8;
uint32_t MotorA = 0;
uint32_t MotorB = 1;


//Defining Morotr Encoders
const int encoderRight_A = 34;
const int encoderRight_B = 35;

const int encoderLeft_A = 22;
const int encoderLeft_B = 23;


//Encoder counts
int counter_R = 0;
int counter_L = 0;
int prev_counter_r = 0;
int prev_counter_l = 0;

//distance to obstacles measured by ultrasonic sensors.
int dist_r;
int dist_l;
int dist_f;
int dist_f_r;
int dist_f_l;

//distance moved by each wheel
int motion_r = 0;
int motion_l = 0;
int Dc;

// Command Velocity to each wheel

int vel_r = 0;
int vel_l = 0;

//robot geometry
const int wheel_base = 205.5;
const int wheel_radius = 67;

int phi_current;
int phi_des;

// State estimate from odometry
int x_p = 0;
int y_p = 0;
int phi_p = 0;

//Target Location

int x_g = 5;
int y_g = 1;


//PID terms

const int Kp_phi = 2;
const int Kd_phi = .4;
const int Ki_phi = .1;

int prev_d_phi = 0;
int e_d_phi;
int e_i_phi;
int sum_i_phi;
const int limit_i_phi = 10;

const int Kp_v = 2;
const int Kd_v = .6;
const int Ki_v = .1;

int prev_d_v = 0;
int e_d_v;
int e_i_v;
int sum_i_v;
const int limit_i_v = 20;

const int pi = 3.141592653989;
long dt;


//Defining Custom Functions:
void Move(int vel, int ang);
void sense();
void encode();
void odom();
void gotogoal();
void avoidObstacle();
void followWall();



void setup() {

  Serial.begin(9600);

  servo.attach(Servo_sig);

  ESP32Encoder::useInternalWeakPullResistors=UP;
	encoder_l.attachFullQuad(23, 22);
	encoder_r.attachFullQuad(34, 35);
  encoder_l.clearCount();
  encoder_r.clearCount();
  
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  ledcSetup(MotorA, freq, resolution);
  ledcSetup(MotorB, freq, resolution);
  ledcAttachPin(PWM_A, MotorA);
  ledcAttachPin(PWM_B, MotorB);

  pinMode(encoderLeft_A, INPUT);
  pinMode(encoderLeft_B, INPUT);
  pinMode(encoderRight_A, INPUT);
  pinMode(encoderRight_B, INPUT);

  pinMode(frontSen_echo, INPUT);
  pinMode(frontSen_trig, OUTPUT);

  pinMode(rightSen_echo, INPUT);
  pinMode(rightSen_trig, OUTPUT);

  pinMode(leftSen_echo, INPUT);
  pinMode(leftSen_trig, OUTPUT);

  // Set initial Encoder Value
  counter_R = encoder_r.getCount();
  counter_L = encoder_l.getCount();


  // Set Motor to be off

  /*digitalWrite(PWM_A, 0);
  digitalWrite(PWM_B, 0);
  delay(100);*/

  
}

void loop() {

  
  Move(500, 1);
  encode();
  Serial.print(counter_R);
  Serial.print(" ");
  Serial.println(counter_L);


 
}


void sense(){
  dist_f_r = sonar_front.ping_cm();
  servo.write(80);
  delay(50);
  dist_l = sonar_left.ping_cm();
  delay(150);
  dist_f = sonar_front.ping_cm();
  delay(50);
  servo.write(130);
  dist_r = sonar_right.ping_cm();
  delay(150);
  dist_f_l = sonar_front.ping_cm();
  servo.write(30);
}


void encode(){

  counter_R =  encoder_r.getCount();
  counter_L = encoder_l.getCount();

  prev_counter_r = counter_R;
  prev_counter_l = counter_L;
}



void Move(int vel_r, int vel_l){

  if (vel_r >= 0 ){
    digitalWrite(DIR_A, HIGH);
  } else {
    digitalWrite(DIR_A, LOW);
  }
  
  if (vel_l >= 0){
    digitalWrite(DIR_B, HIGH);
  } else {
    digitalWrite(DIR_B, LOW);
  }
  

  digitalWrite(PWM_A, vel_r);
  digitalWrite(PWM_B, vel_l);

  //ledcWrite(PWM_A, vel_r);
  //ledcWrite(PWM_B, vel_l);
}



void odom(){
  
  //Odometry calculations

  motion_r = 2*pi*wheel_radius*(counter_R - prev_counter_r)/823.0;
  motion_l = 2*pi*wheel_radius*(counter_L - prev_counter_l)/823.0;
  Dc = (motion_r + motion_l)/2;

  x_p = x_p + Dc * cos(phi_p);
  y_p = y_p + Dc* sin(phi_p);
  phi_p = phi_p + (motion_r - motion_l)/wheel_base;

}

void gotogoal(){

// Update position estimate
 odom();

//Angle Errors
 int E_x = x_g - x_p;
 int E_y = y_g - y_p;

 int phi_g = atan2(E_y, E_x);

 int E_g = phi_g - phi_p;
 E_g = atan2(sin(E_g), cos(E_g));

 e_i_phi = e_i_phi + E_g*dt;
 e_d_phi = (E_g - prev_d_phi)/dt;

 if (e_i_phi > limit_i_phi){
   e_i_phi = limit_i_phi;
 }
 
 int u_phi = E_g * Kp_phi + e_i_phi * Ki_phi + e_d_phi * Kd_phi;
 
//Distance Errors 
 
 int E_d = sqrt(E_x^2 +E_y^2);
 e_i_v = e_i_v + E_d *dt;
 e_d_v = (E_d - prev_d_v)/dt;

 if (e_i_v > limit_i_v){
   e_i_v = limit_i_v;;
 }

 int u_v = E_d * Kp_v + e_i_v * Ki_v + e_d_v * Kd_v;

//limiting V to ensure phi


// Decomposing to indivicual wheel vel

 vel_r = (2*u_v + u_phi*wheel_base)/(2*wheel_radius);
 vel_l = (2*u_v - u_phi*wheel_base)/(2*wheel_radius);

 Move(vel_r, vel_l);


}