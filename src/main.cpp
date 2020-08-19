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
int sum_m_r = 0;
int sum_m_l = 0;

// Command Velocity to each wheel

float vel_r = 0;
float vel_l = 0;

//robot geometry
const float wheel_base = 220;
const float wheel_radius = 32;

int phi_current;
int phi_des;

// State estimate from odometry
float x_p = 0;
float y_p = 0;
float phi_p = 0.0;

//Target Location

int x_g = 4000;
int y_g = 2000;

int E_x ;
int E_y ;


//PID terms for phi

const int Kp_phi = 30;
const int Kd_phi = 0.5;
const int Ki_phi = 0.4;

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
float degTorad = .017453;
float radTodeg = 57.2958;

// Vector and orientation calculations;
float crd[2];
float vec_x[] = {1, 0};
float vec_y[] = {0, 1};


//Defining Custom Functions:
void Move(float vel_r, float vel_l);
void sense(int angle);
void encode();
void odom();
void gotogoal(int x, int y);
void AvoidObstacle();
void followObstacle(int x, int y);
void ensure_phi(int v, float w);
void ensure_v(float vel_r, float vel_l);
void set_pid();
int goal_check(int x, int y);
void stop();
void rotate(float angle);
void get_crd(float m1, float m2, int x, int y, int angle);
float vect_angle(float v1[], float v2[]);
int gtg_check(int prev_dist, float obst_vect[]);


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

  int reached  = goal_check(x_g, y_g);

  if (reached  == 1){
    stop();
  }

  else {

    servo.write(80);
    sense(0);

    if (dist_f >=30 && dist_l >= 15 && dist_r >=15){
      gotogoal(x_g, y_g);

    }

    else if (dist_f >=15 && dist_l >= 5 && dist_r >=5){
      AvoidObstacle();
    }

    else{
      followObstacle(x_g, y_g);
    }

    if ((global_prev_millis - PID_millis) >=10){ 
      set_pid();
      PID_millis = global_prev_millis;
    }  
    


  }


  
  //loop_time = micr() - prev_millis;
  //Serial.print(" time ");
  //Serial.println(loop_time);
}




int goal_check(int x, int y){

  odom();
  E_x = x - x_p;
  E_y = y - y_p;

  int mag_dist = sqrt((E_x*E_x) + (E_y*E_y));


  if (mag_dist <= 100){
    return 1;
  }
  else {
    return 0;
  }
}

void gotogoal(int x, int y){

  if ((global_prev_millis - PID_prev_gtg) >=9 || step == 1){

    // Update position estimate
    odom();
    // update timer
    PID_prev_gtg = global_prev_millis;

    //Angle Errors
    E_x = x - x_p;
    E_y = y - y_p;

    float phi_g = atan2(E_y, E_x);

    float E_g = phi_g - phi_p;
    E_g = atan2(sin(E_g), cos(E_g));
    

    e_i_phi = e_i_phi + E_g*dt;
    e_d_phi = (E_g - prev_d_phi)/dt;
    prev_d_phi = E_g;

    if (e_i_phi > limit_i_phi){
      e_i_phi = limit_i_phi;
    }
    
    //PID for angle control
    float w = E_g * Kp_phi + e_i_phi * Ki_phi + e_d_phi * Kd_phi;

    int mag_dist  = sqrt((E_x*E_x)+ (E_y*E_y));
    int v ;
    if (mag_dist >= 1000){
      v= 25*33.5;
    }

    else {

      v = 25 - 3*(1-mag_dist/1000);
    }
    

    //limiting V to ensure phi
    ensure_phi(v, w);

  

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
    Serial.println(" ");
  }

  if ((global_prev_millis - PID_millis_v) >=3 ){ // && (move_est_l >=1 || move_est_r >= 1)

        ensure_v(vel_r, vel_l);
        PID_millis_v = global_prev_millis;
        //Serial.println(PID_millis_v);
      }

}

void stop(){

  ledcWrite(MotorA, 0);
  ledcWrite(MotorB, 0);
  
}

void AvoidObstacle(){

  
}

void followObstacle(int x, int y){

  // Get Odometry data
  stop();
  odom();
  sense(1);

  //Angle Errors
  E_x = x - x_p;
  E_y = y - y_p;
  float vect_gtg[] = {E_x, E_y};

  //local vector for front right and left obstacle points
  get_crd(dist_f_r, 0,112, 0, 50);
  float vec_l_fr[] = {crd[0], crd[1]};
  
  get_crd(dist_f_l, 0, 112, 0, -50);
  float vec_l_fl[] = {crd[0], crd[1]};

  //global coordinates for front right and left obstacle points
  get_crd(vec_l_fr[0],vec_l_fr[1], x_p, y_p, phi_p);
  float crd_g_fr[] = {crd[0], crd[1]};

  get_crd(vec_l_fl[0],vec_l_fl[1], x_p, y_p, phi_p);
  float crd_g_fl[] = {crd[0], crd[1]};
  

  //c and cc vectors
  float vec_obs_cc[] = {crd_g_fl[0] - crd_g_fr[0],crd_g_fl[1] - crd_g_fr[1]};
  float vec_obs_c[] = {crd_g_fr[0] - crd_g_fl[0],crd_g_fr[1] - crd_g_fl[1]};


  // angle between cc and gtg, c and gtg respectively 
  float ang_dir_cc = vect_angle(vec_obs_cc, vect_gtg);
  float ang_dir_c = vect_angle(vec_obs_c, vect_gtg);
  
  
  if (ang_dir_cc >=0){


    float ang_follow_obs = acos(vect_angle(vec_obs_cc, vec_x));

    // rotate robot to align with obstacle
    rotate(ang_follow_obs);
    
    //measure distance to front obstacle
    sense(0);
    int front_dist = dist_f;
    
    //rotate front sensor to the obstacle
    servo.write(130);
    
    int actual_dist = 0;
    int step = 1;
    float angle_dif = 0;
    int is_gtg = 0;
    int dist_tt = vec_normalize(vect_gtg);
    unsigned long PID_prev_fo = 0;

    while (((actual_dist <= 0.8*front_dist) && (angle_dif <= 0.52)) || is_gtg == 0){

      unsigned long follow_obs_millis = millis();

      if ((follow_obs_millis - PID_prev_fo) >=9){

        // Update position estimate
        odom();

        PID_prev_fo = follow_obs_millis;

        //Angle Errors

        float E_g = ang_follow_obs - phi_p;
        E_g = atan2(sin(E_g), cos(E_g));
      
        e_i_phi = e_i_phi + E_g*dt;
        e_d_phi = (E_g - prev_d_phi)/dt;
        prev_d_phi = E_g;

        if (e_i_phi > limit_i_phi){
          e_i_phi = limit_i_phi;
        }
        
        //PID for angle control
        float w = E_g * Kp_phi + e_i_phi * Ki_phi + e_d_phi * Kd_phi;

        actual_dist += Dc;

        int v;
        if (actual_dist >= 0.6 * front_dist){
          v= 25*33.5;
        }

        else {

          v = 25 - 3*(1-front_dist/1000);
        }

        //limiting V to ensure phi
        ensure_phi(v, w);

        if (step == 1){
          Move(vel_r, vel_l);
        }
        
        step = 2;

        // Re-calculate orientation vector
        sense(0);
        //local vector for front right and left obstacle points
        
        get_crd(dist_f, 0, 112, 0, 50);
        float vec_l_f[] = {crd[0], crd[1]};
        
        get_crd(dist_r, 0, 60, 40, -50);
        float vec_l_r[] = {crd[0], crd[1]};
        
        //global coordinates for front right and left obstacle points
        get_crd(vec_l_f[0],vec_l_f[1], x_p, y_p, phi_p);
        float crd_g_f[] = {crd[0], crd[1]};

        get_crd(vec_l_r[0],vec_l_r[1], x_p, y_p, phi_p);
        float crd_g_r[] = {crd[0], crd[1]};
        
        //c and cc vectors
        float vec_obs[] = {crd_g_f[0] - crd_g_r[0],crd_g_f[1] - crd_g_r[1]};

        // angle between cc and gtg, c and gtg respectively 
        ang_follow_obs = vect_angle(vec_obs, vec_x);

        angle_dif = ang_follow_obs - phi_p;

        // Check if we can gtg now
        is_gtg = gtg_check(dist_tt, vec_obs);

        Serial.print(v);
        Serial.print(" ");
        Serial.print(w);
        Serial.print(" ");
        Serial.print(x_p);
        Serial.print(" ");
        Serial.print(y_p);
        Serial.print(" ");
        Serial.print(phi_p);
        Serial.println(" ");
      }

      if ((follow_obs_millis - PID_millis_v) >=3 ){ // && (move_est_l >=1 || move_est_r >= 1)

            ensure_v(vel_r, vel_l);
            PID_millis_v = follow_obs_millis;
            
          }

        }
  }

}

void sense(int angle){
  
  if (angle == 0){

  dist_l = sonar_left.ping_cm();
  dist_f = sonar_front.ping_cm();
  dist_r = sonar_right.ping_cm();

  }

  else{

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
  
  sum_m_r +=  motion_r;
  sum_m_l +=  motion_l;
  
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

  ledcWrite(MotorA, u_v_l);
  ledcWrite(MotorB, u_v_r);

}

void rotate(float angle){
  
  float ang_err = angle - phi_p;
  ang_err = atan2(sin(ang_err), cos(ang_err));
  
  while (ang_err>= 0.087){
    
    unsigned long rotate_millis = millis();
    unsigned long prev_rotate_millis = 0;

    if (rotate_millis - prev_rotate_millis >= 3){
      e_i_phi = e_i_phi + ang_err*dt;
      e_d_phi = (ang_err - prev_d_phi)/dt;
      prev_d_phi = ang_err;

      if (e_i_phi > limit_i_phi){
        e_i_phi = limit_i_phi;
      }
      
      //PID for angle control
      float w = ang_err * Kp_phi + e_i_phi * Ki_phi + e_d_phi * Kd_phi;
      int v = 0;

      ensure_phi(v, w);
      ensure_v(vel_r, vel_l);

    }

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
    float w_min = wheel_radius/wheel_base * (2 * -0.7 * vel_max);
    float w_max = wheel_radius/wheel_base * (2 * 0.7 * vel_max);

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

void ensure_v(float v_r, float v_l){

  odom();

  float abs_v_r = abs(v_r);
  float abs_v_l = abs(v_l);

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


  Move(u_r * v_r/abs_v_r, u_l * v_l/abs_v_l);

  PID_millis_v = millis();

}


void set_pid(){

  Kp_v = analogRead(33);

  Kp_v = map(Kp_v, 0, 4095, 0, 300);

}

void get_crd(float m1, float m2, int x, int y, int angle){
   
   angle = angle * degTorad;
   crd[0] = m1 * cos(angle) - m2 * sin(angle) + x;
   crd[1] = m1 * sin(angle) + m2 * cos(angle) + y;

}

float vec_normalize(float vec[]){

  float normalized = sqrt( vec[0]*vec[0] + vec[1]*vec[1]);

  return normalized;

}

float vect_angle(float v1[], float v2[]){
  
  float norm1  = vec_normalize(v1);
  float norm2 = vec_normalize(v2);
  float cos_angle = (v1[0]*v2[0] + v1[1]*v2[1] )/(norm1* norm1);

  return cos_angle;
}

int gtg_check(int prev_dist, float obst_vect[]){

  
  // check angle 
  get_crd(obst_vect[0], obst_vect[1], 0, 0, 90);
 
  float avoid_obst_vect[] = {crd[0], crd[1]};

  float gtg_vect[] = {x_g - x_p, y_g - y_p};

  float vec_angle = vect_angle(avoid_obst_vect, gtg_vect);

  //check distance
  int actual_dist = vec_normalize(gtg_vect);

  if (vec_angle <=0 && actual_dist <= prev_dist){
    return 1;
  }

  else {
    return 0;
  }

}

