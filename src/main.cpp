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

uint32_t freq = 5000;
uint32_t resolution = 13;
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
const float wheel_base = 205.5;
const float wheel_radius = 33.5;

int phi_current;
int phi_des;

// State estimate from odometry
int x_p = 0;
int y_p = 0;
int phi_p = 0;

//Target Location

int x_g = 5000;
int y_g = 5000;


//PID terms for phi

const int Kp_phi = 2.5;
const int Kd_phi = 0.5;
const int Ki_phi = 0.2;

float prev_d_phi = 0;
float e_d_phi=0;
float e_i_phi = 0;
int sum_i_phi;
const int limit_i_phi = 1;

// PID terms for v
const int Kp_v = 3;
const int Kd_v = 10;
const int Ki_v = 1.5;

float prev_d_v = 0;
float e_d_v=0;
float e_i_v = 0;
int sum_i_v_r = 0;
int sum_i_v_l = 0;
const int limit_i_v = 500;

int counter_r_pid = 0;
int counter_l_pid = 0;
int corrected_v_r = 0;
int corrected_v_l = 0;

unsigned long PID_millis_v = 0;
unsigned long PID_millis_phi = 0;

const int pi = 3.141592653989;
float dt= 0.01;


//Defining Custom Functions:
void Move(float vel_r, float vel_l);
void sense();
void encode();
void odom();
void gotogoal(int x, int y);
void avoidObstacle();
void followWall();
void ensure_phi(int v, float w);



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

  ledcWrite(MotorA, 0);
  ledcWrite(MotorB, 0);
  delay(100);

  
}

void loop() {

  x_g = 5000;
  y_g = 5000;
  gotogoal(x_g, y_g);

 
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



void Move(float vel_r, float vel_l){

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

  float f_vel_r = vel_r;
  float f_vel_l = vel_l;
  unsigned long timer_pid_v = millis() - PID_millis_v;
  PID_millis_v = timer_pid_v;

  if (timer_pid_v >= 200){

    float vel_est_r = (counter_R - counter_r_pid)/(823.0*timer_pid_v/1000.0)*2*pi;
    float vel_est_l = (counter_L - counter_l_pid)/(823.0*timer_pid_v/1000.0)*2*pi;
    float e_p_v_r = vel_r - vel_est_r;
    float e_p_v_l = vel_l - vel_est_l;
    sum_i_v_r = sum_i_v_r + e_p_v_r * timer_pid_v;
    sum_i_v_l = sum_i_v_l + e_p_v_l * timer_pid_v;

      if (sum_i_v_r > limit_i_v){
        sum_i_v_r = limit_i_v;
      }

      if (sum_i_v_r > limit_i_v){
        sum_i_v_r = limit_i_v;
      }  

    corrected_v_r = Kp_v * e_p_v_r + Ki_v * sum_i_v_r;
    corrected_v_l = Kp_v * e_p_v_l + Ki_v * sum_i_v_l;

  }
  int u_v_r = f_vel_r + corrected_v_r;
  int u_v_l = f_vel_l + corrected_v_l;

  u_v_r = constrain(u_v_r, 0, 7500);
  u_v_l = constrain(u_v_l, 0, 7500);

  ledcWrite(MotorA, u_v_r);
  ledcWrite(MotorB, u_v_l);
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

void gotogoal(int x, int y){

// Update position estimate
 odom();

//Angle Errors
 int E_x = x - x_p;
 int E_y = y - y_p;

 float phi_g = atan2(E_y, E_x);

 float E_g = phi_g - phi_p;
 E_g = atan2(sin(E_g), cos(E_g));

 e_i_phi = e_i_phi + E_g*dt;
 e_d_phi = (E_g - prev_d_phi)/dt;
 prev_d_phi = E_g;

 if (e_i_phi > limit_i_phi){
   e_i_phi = limit_i_phi;
 }
 
 int w = E_g * Kp_phi + e_i_phi * Ki_phi + e_d_phi * Kd_phi;

//Distance Errors 
 
 
 int v;
 if (abs(E_g) > 1.22){
   v = 0;
 } else{
   int mag = pow(E_x, 2) + pow(E_y, 2);
   v = sqrt(mag);
 } 

//limiting V to ensure phi
 ensure_phi(v, w);

// Move

 Move(vel_r, vel_l);

 Serial.print(v);
 Serial.print(" ");
 Serial.print(w);
 Serial.print(" ");
 Serial.print(vel_r);
 Serial.print(" ");
 Serial.println(vel_l);


}

void ensure_phi(int v, float w){
  
  int vel_max = 21;
  int vel_min = -21;
  float w_int = w;

  float w_lim_comp = ((wheel_radius/wheel_base)*(vel_max-vel_min));
  float vel_lim = constrain(abs(v),(wheel_radius/2)*(2*vel_min), wheel_radius/2 * (2.0 * vel_max));
  float w_lim = constrain( w_int, 0, w_lim_comp);
  float vel_d_r = (2*vel_lim + w_lim*wheel_base)/(2*wheel_radius);
  float vel_d_l = (2*vel_lim - w_lim*wheel_base)/(2*wheel_radius);
  float vel_rl_max = max(vel_d_r, vel_d_l);
  float vel_rl_min = min(vel_d_r, vel_d_l);

  if (abs(v) > 0){
    if (vel_rl_max > vel_max) {
      vel_r  = vel_d_r - (vel_rl_max - vel_max);
      vel_l = vel_d_l - (vel_rl_max - vel_max);
    } else if(vel_rl_min <vel_min){
      vel_r  = vel_d_r + (vel_min - vel_rl_min);
      vel_l = vel_d_l + (vel_min - vel_rl_min);
    } else {
      vel_r = vel_d_r;
      vel_l = vel_d_l;
    }

  }

  else {
    float w_min = wheel_radius/wheel_base *  (2 * vel_min);
    float w_max = wheel_radius/wheel_base * (2* vel_max);

    if (abs(w) > w_min) {
      float abs_w = abs(w);
      w =  constrain (abs_w, w_min, w_max);
    } else {
      w = 0;
    }

    vel_r = (w*wheel_base)/(2*wheel_radius);
    vel_l = (-w*wheel_base)/(2*wheel_radius);

  }
  
}
