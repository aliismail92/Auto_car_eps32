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

uint32_t freq = 732;
uint32_t resolution = 12;
uint32_t MotorA = 0;
uint32_t MotorB = 1;


//Defining Morotr Encoders
const int encoderRight_A = 35;
const int encoderRight_B = 34;

const int encoderLeft_A = 23;
const int encoderLeft_B = 22;


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
float motion_r = 0;
float motion_l = 0;
float Dc;

// Command Velocity to each wheel

float vel_r = 0;
float vel_l = 0;

//robot geometry
const float wheel_base = 205.5;
const float wheel_radius = 33.5;

int phi_current;
int phi_des;

// State estimate from odometry
float x_p = 0;
float y_p = 0;
float phi_p = 0.0;

//Target Location

int x_g = 5000;
int y_g = 5000;


//PID terms for phi

const int Kp_phi = 10;
const int Kd_phi = 0.5;
const int Ki_phi = 0.2;

float prev_d_phi = 0;
float e_d_phi=0;
float e_i_phi = 0;
int sum_i_phi;
const int limit_i_phi = 1;

// PID terms for v
int Kp_v = 50;
int Kd_v = 10;
int Ki_v = 3;

int error_v_r;
int error_v_l;
float prev_d_v = 0;
float e_d_v=0;
float e_i_v = 0;
int sum_i_v_r = 0;
int sum_i_v_l = 0;
int limit_i_v = 500;
int move_est_r;
int move_est_l;
int f_vel_r;
int f_vel_l;

int counter_r_pid = 0;
int counter_l_pid = 0;
int corrected_v_r = 0;
int corrected_v_l = 0;

//Motor command:
int u_v_r; 
int u_v_l;

// Timers:
unsigned long global_prev_millis;
unsigned long PID_millis_v = 0;
unsigned long PID_prev_v = 0;
unsigned long PID_millis_gtg = 0;
unsigned long PID_prev_gtg = 0;
unsigned long loop_time = 0;
unsigned long PID_millis = 0;

// Steps:
int step = 1;

const float pi = 3.141592653989;
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
void ensure_v(float vel_r, float vel_l);
void set_pid();



void setup() {

  Serial.begin(115200);

  servo.attach(Servo_sig);

  ESP32Encoder::useInternalWeakPullResistors=UP;
	encoder_l.attachFullQuad(23, 22);
	encoder_r.attachFullQuad(34, 35);
  encoder_l.clearCount();
  encoder_r.clearCount();
  
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

  global_prev_millis = millis();


  x_g = 20000;
  y_g = 7000;
  gotogoal(x_g, y_g);

  if ((global_prev_millis - PID_millis_v) >=3){ // && (move_est_l >=1 || move_est_r >= 1)

    ensure_v(vel_r, vel_l);
    PID_millis_v = global_prev_millis;
    Serial.println(PID_millis_v);
  }

  if ((global_prev_millis - PID_millis) >=10){ // && (move_est_l >=1 || move_est_r >= 1)
    set_pid();
    PID_millis = global_prev_millis;
  }  
  

  
  //loop_time = micr() - prev_millis;
  //Serial.print(" time ");
  //Serial.println(loop_time);
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

  counter_R = -1 * encoder_r.getCount();
  counter_L = encoder_l.getCount();

}


void odom(){
  
  //Odometry calculations
  encode();
  motion_r = 2*pi*wheel_radius *(counter_R - prev_counter_r)/823.0;
  motion_l = 2*pi*wheel_radius*(counter_L - prev_counter_l)/823.0;
  Dc = (motion_r + motion_l)/2;

  x_p = x_p + Dc * cos(phi_p);
  y_p = y_p + Dc* sin(phi_p);
  phi_p = phi_p + (motion_l - motion_r)/wheel_base;

  move_est_r = (counter_R - prev_counter_r);
  move_est_l = (counter_L - prev_counter_l);
  

  prev_counter_r = counter_R;
  prev_counter_l = counter_L;

}

void Move(float v_r, float v_l){

  if (v_r >= 0 ){
    digitalWrite(DIR_A, HIGH);
  } else {
    digitalWrite(DIR_A, LOW);
  }
  
  if (v_l >= 0){
    digitalWrite(DIR_B, HIGH);
  } else {
    digitalWrite(DIR_B, LOW);
  }

  if (step == 1){
  f_vel_r = v_r * v_r* 12.0 - 523.0 * v_r + 7200;
  f_vel_l = v_l * v_l * 12.0 - 523.0 * v_l + 6200;
  u_v_r = constrain(f_vel_r, 1400, 4095);
  u_v_l = constrain(f_vel_l, 1400, 4095);

  }

  else{
    u_v_l = v_l;
    u_v_r = v_r;

  }
/*
  if (vel_r ==  20) {
    u_v_r = 0;
  }

  if (vel_l == 20){
    u_v_l = 0;
  }
  
  
  Serial.print(" ");
  Serial.print(u_v_r);
  Serial.print(" ");
  Serial.print(u_v_l);
  Serial.println(" ");*/

 // ledcWrite(MotorA, u_v_r);
 // ledcWrite(MotorB, u_v_l);
  
  /*
  Serial.println(timer_pid_v);
  Serial.print(" ");
  Serial.print(v_l);
  Serial.print(" ");
  Serial.print(v_r);
  Serial.print(" ");
  Serial.print(f_vel_l);
  Serial.print(" ");
  Serial.print(f_vel_r);
  Serial.print(" ");

  Serial.print(corrected_v_l);
  Serial.print(" ");
  Serial.print(corrected_v_r);
  Serial.print(" ");
  Serial.print(u_v_l);
  Serial.print(" ");
  Serial.print(u_v_r);
  Serial.print(" ");*/
}

void ensure_v(float v_r, float v_l){

  odom();


  error_v_r =  v_r * 3 / (2 * pi) * 823/1000 - (counter_R - counter_r_pid);
  error_v_l =  v_l * 3 / (2 * pi) * 823/1000 - (counter_L - counter_l_pid);

  counter_l_pid = counter_L;
  counter_r_pid = counter_R;
  
  sum_i_v_l += error_v_l;
  sum_i_v_r += error_v_r;

  sum_i_v_l = constrain(sum_i_v_l, -1 * limit_i_v, limit_i_v);
  sum_i_v_r = constrain(sum_i_v_r, -1 * limit_i_v, limit_i_v);

  int u_r =  Kp_v * error_v_r + Ki_v * sum_i_v_r;
  int u_l =  Kp_v * error_v_l + Ki_v * sum_i_v_l;

  if (vel_r ==  20) {
    u_r = 0;
  }

  if (vel_l == 20){
    u_l = 0;
  }


  u_r = constrain(u_r, 0, 4095);
  u_l = constrain(u_l, 0, 4095);

  //Move(u_r, u_l);
  
  ledcWrite(MotorA, u_l);
  ledcWrite(MotorB, u_r);
  

  PID_millis_v = millis();
  

}

void gotogoal(int x, int y){

if ((global_prev_millis - PID_prev_gtg) >=12 || step == 1){

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
  
  float w = E_g * Kp_phi + e_i_phi * Ki_phi + e_d_phi * Kd_phi;

  //Distance Errors 
  /*
  Serial.print(E_g);
  Serial.print( " ");
  Serial.print( w);
  Serial.print(" ");*/
  int v= 25*33.5;
  /*
  if (abs(E_g) > 1.22){
    v = 0;
  } else{
    int mag = pow(E_x, 2) + pow(E_y, 2);
    //v = sqrt(mag);
    v = 25;
  } 
  */
  //limiting V to ensure phi
  ensure_phi(v, w);

  // Move

  if (step == 1){
    Move(vel_r, vel_l);
  }


  step = 2;

  Serial.print(v);
  Serial.print(" ");
  Serial.print(w);
  Serial.print(" ");
  Serial.print(x_p);
  Serial.print(" ");
  Serial.print(y_p);
  Serial.print(" ");
  Serial.print(phi_p);
  Serial.print(" ");

}



}

void ensure_phi(int v, float w){
  
  int vel_max = 35;
  int vel_min = 20;
  float w_int = w;

  float w_lim_comp = ((wheel_radius/wheel_base)*(vel_max-vel_min));
  float vel_lim = constrain(abs(v),(wheel_radius/2)*(2*vel_min), wheel_radius/2 * (2.0 * vel_max));
  float w_lim = constrain( w_int, -1* w_lim_comp, w_lim_comp);
  float vel_d_r = (2*vel_lim - w_lim*wheel_base)/(2*wheel_radius);
  float vel_d_l = (2*vel_lim + w_lim*wheel_base)/(2*wheel_radius);
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

    vel_r = (-w*wheel_base)/(2*wheel_radius);
    vel_l = (w*wheel_base)/(2*wheel_radius);

  }
  
}

void set_pid(){

  Kp_v = analogRead(33);

  Kp_v = map(Kp_v, 0, 4095, 0, 300);

}
