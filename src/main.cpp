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

const int freq = 30000;
const int resolution = 8;
const int MotorA = 0;
const int MotorB = 1;




//Defining Morotr Encoders
const int encoderRight_A = 34;
const int encoderRight_B = 35;

const int encoderLeft_A = 23;
const int encoderLeft_B = 22;


int angle = 90;
int counter_R = 0;
int counter_L = 0;

int dist_r;
int dist_l;
int dist_f;
int dist_f_r;
int dist_f_l;

const int wheel_base = 205.5;
const int wheel_radius = 67;


//Defining Custom Functions:
void Move(int vel, int ang);
void sense();
void encode();



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

  digitalWrite(PWM_A, 0);
  digitalWrite(PWM_B, 0);
  delay(100);

  
}

void loop() {

  
  Move(1000, 1);
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
}



void Move(int vel, int ang){

  digitalWrite(DIR_A, HIGH);
  digitalWrite(DIR_B, HIGH);

  digitalWrite(PWM_A, HIGH);

  //ledcWrite(PWM_A, vel);
  ledcWrite(PWM_B, 1000);
}