// Define Libraries
#define nullptr NULL
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>


// Define Variables
int opticalPin = 2;     // optical sensor connected to digital pin 2
int servoPin = 6;        // servo motor connected to digital pin 6
int ledPin = 13;        // The 13th pin is connected to a LED
int enable = 0;
int count = 0;
float diameter = 0.115;
float delta_L = 0.0;
float total_pulses = 1150.0;
float L_min = 1.0; // minimum length allowed 
float L_max = 10.0; // maximum length allowed 
float initial_L = 1.1; //initial length 
float current_L = 1.1; //initial length 
float ref_L = 0; // reference length to control  
float PWM = 0;
bool controlling = false;
bool reverse = false;
unsigned long current_time,  delta_time;
unsigned long previous_time = 0;

// ROS stuff
ros::NodeHandle  nh;

std_msgs::Float32 length_msg;
std_msgs::UInt16 count_steps_turn_motor_msg;

// Callbacks
void lengthSubCallback(const std_msgs::Float32& length_ref_msg) {
    if (fabs(ref_L - length_ref_msg.data) > 0.02) { 
      ref_L = length_ref_msg.data;
      count = 0;
      controlling  = true;
      nh.loginfo("Received new length command: ");
    }
    
}

void resetLengthCallback(const std_msgs::Float32& length_reset) {
     current_L = initial_L = length_reset.data;
     controlling = false;
     count = 0;
}

void enableCallback(const std_msgs::Bool& bool_msg) {
    enable = bool_msg.data;
    if (enable) {
        controlling = false;
        nh.loginfo("Control enabled");
    } else {
        nh.loginfo("Control disabled");
    }
    count = 0;
}

void reverseCallback(const std_msgs::Bool& bool_msg) {
    reverse = bool_msg.data;
    if (reverse) {
        nh.loginfo("Reverse enabled");
    } else {
        nh.loginfo("Reverse disabled");
    }
    controlling = false;
    count = 0;
}


// Subscriber for the length command
ros::Subscriber<std_msgs::Float32> sub("control_tie/set_length", &lengthSubCallback );

// Emit the current estimated longitude of the tie
ros::Publisher pub_length("control_tie/length_status", &length_msg);

// Wait for the message to start the system
ros::Subscriber<std_msgs::Bool> sub_enable("control_tie/enable", &enableCallback);

// Length reset topic just in case
ros::Subscriber<std_msgs::Float32> sub_reset("control_tie/reset_length_estimation", &resetLengthCallback);

// Reverse the control and estimation
ros::Subscriber<std_msgs::Bool> sub_reverse("control_tie/reverse", &reverseCallback);

// Para depurar el funcionamiento del cacharro
ros::Publisher pub_count_steps_turn_motor("control_tie/count_steps_turn_motor", &count_steps_turn_motor_msg); 

void controlReel(float ref_L_){
  // put your main code here, to run repeatedly:
  float error = ref_L_ - current_L;
  if (reverse)
    error *= -1.0;
  float tolerance_error = 0.02;
  int sign = 1; // If no control action an action would increase the longitude
   
  if (error < tolerance_error && error > -tolerance_error && controlling){
    PWM = 0;
    controlling == false;
    initial_L = current_L;
    sign = 0;
    count = 0;
  }
  else if (error > 0 && controlling){ // increase length: Max_Vel=98 pwm , Min_Vel=118 pwm ; slope = -20 
    if (error > 1.0)
      PWM = 98;
    else
      PWM = -20 * error + 118;
    sign = reverse?-1.0:1.0;
  }
  else if (error < 0 && controlling){ // reduce lentgh: Max_Vel=195 pwm , Min_Vel=175 pwm ; slope = -20
    if (error < -1.0)
      PWM = 195;
    else
      PWM = -20 * error + 175;
    sign = reverse?1.0:-1.0;
  }
  analogWrite(servoPin, PWM);
      
  delta_L = (count/total_pulses)* sign * PI * diameter;
  current_L = initial_L + delta_L;
}

void countPulse(){
  current_time = millis();
  delta_time = current_time - previous_time;
  previous_time = current_time;
  if (delta_time < 20)
    count++;
  count_steps_turn_motor_msg.data = count;
}


void setup() {
    // Put your setup code here, to run once:
    //Serial.begin(115200);
    pinMode(opticalPin, INPUT);   // sets the digital pin 2 as input
    pinMode(servoPin, OUTPUT);    // sets the digital pin 6 as output
    pinMode(ledPin,OUTPUT);      // to light and disable the LED of 13 pin

    attachInterrupt(digitalPinToInterrupt(opticalPin), countPulse, RISING);
    
    // ROS Config
    nh.initNode();
    nh.advertise(pub_length);
    nh.advertise(pub_count_steps_turn_motor);
    nh.subscribe(sub);
    nh.subscribe(sub_enable);
    nh.subscribe(sub_reset);
    nh.subscribe(sub_reverse);
}

void loop() { 
  if (enable){ // Reel working after security check         
      controlReel(ref_L);
      digitalWrite(ledPin, HIGH);

  }  else {
      // Waiting to receive the enable topic
      analogWrite(servoPin, 127);
      digitalWrite(ledPin, LOW);
      delay(10);       
  }
  length_msg.data = current_L;
  pub_length.publish(&length_msg);
  pub_count_steps_turn_motor.publish(&count_steps_turn_motor_msg);
  nh.spinOnce();
  delayMicroseconds(100); // when using delay alone --> milliseconds
}
