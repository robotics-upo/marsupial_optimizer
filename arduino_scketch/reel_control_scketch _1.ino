// Define Libraries
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

// Define Variables
int startPin = 8;     // start button (reel system) connected to digital pin 8
int opticalPin = 2;     // optical sensor connected to digital pin 2
int resetOpticalPin = 7;     // reset button (count turn) connected to digital pin 7
int servoPin = 6;        // servo motor connected to digital pin 6
int start = 0;
int count = 0;
bool detect_high = false;
bool detect_low = false;
float diameter = 0.115;
float delta_L = 0.0;
float total_pulses = 1150.0;
float L_min = 1.0; // minimum length allowed 
float L_max = 10.0; // maximum length allowed 
float initial_L = 1.1; //initial length 
float current_L = 1.1; //initial length 
float ref_L = 0; // reference length to control  
float PWM = 0;
bool controling = false;

std_msgs::Float32 length_msg;
std_msgs::Bool able_to_work_msg;
std_msgs::UInt16 count_steps_turn_motor_msg;

void lengthSubCallback(const std_msgs::Float32& length_ref_msg) {
    ref_L = length_ref_msg.data;
    count = 0;
    controling  = true;
    //initial_L = current_L;
}

int statusTether(){
  if (current_L >= L_min && current_L <= L_max)
    return 0;
  else if ( current_L < L_min)
    return 1;
  else if (current_L > L_max)
    return -1;  
}

void moveReel(float ref_L_){
  // put your main code here, to run repeatedly:
    float diff_L = ref_L_ - current_L;
    float tolerance_error = 0.05;
    int sign;
   
    if (diff_L < tolerance_error && diff_L > -tolerance_error && controling){
      PWM = 0;
      controling == false;
      initial_L = current_L;
      sign = 0;
    }
    else if (diff_L > 0 && controling){ // increase length
      PWM = 100;
      sign = 1.0;
    }
    else if (diff_L < 0 && controling){ // reduce lentgh
      PWM = 200;
      sign = -1.0;
    }
    
    analogWrite(servoPin, PWM);
    
    //int val = digitalRead(opticalPin);     // read the input pin
        
    //if (val == HIGH) 
    //  detect_high = true;

    //if (val == LOW) 
    //  detect_low = true;
    
    //if (detect_high && detect_low ){
    //  count++; 
    //  count_steps_turn_motor_msg.data = count;
    //  detect_high = false;
    //  detect_low = false;
    //} 
      
    delta_L = (count/total_pulses)* sign * PI * diameter;
    current_L = initial_L + delta_L;
}

void cleanCounter()
{
  int but = digitalRead(resetOpticalPin);     // read the input pin
      
  if (but == HIGH) {
    count = 0;
    count_steps_turn_motor_msg.data = 0;
    detect_high = false;
    detect_low = false;
  }
}

void countPulse(){
  count++;
  count_steps_turn_motor_msg.data = count;
}

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Float32> sub("set_length", &lengthSubCallback );

ros::Publisher pub_length("length_status", &length_msg);
ros::Publisher pub_able_to_work("able_to_work", &able_to_work_msg);
ros::Publisher pub_count_steps_turn_motor("count_steps_turn_motor", &count_steps_turn_motor_msg);

void setup() {
    // Put your setup code here, to run once:
    //Serial.begin(115200);
    pinMode(startPin, INPUT);        // sets the digital pin 8 as input
    pinMode(opticalPin, INPUT);        // sets the digital pin 2 as input
    pinMode(resetOpticalPin, INPUT);        // sets the digital pin 7 as input
    pinMode(servoPin, OUTPUT);    // sets the digital pin 6 as output

    attachInterrupt(digitalPinToInterrupt(opticalPin), countPulse, RISING);
    
    // ROS Config
    nh.initNode();
    nh.advertise(pub_length);
    nh.advertise(pub_able_to_work);
    nh.advertise(pub_count_steps_turn_motor);
    nh.subscribe(sub);
}

void loop() { 
  int start_ = digitalRead(startPin);     // read the input pin

  if (start_ == HIGH && start == 0){
    start = 1;
    delay(200);
  }
  else if (start_ == HIGH && start == 1){
    start = 0;
    delay(200);
  } 
  if (start == 1){ // Reel working after security check         
      moveReel(ref_L);
      able_to_work_msg.data = true;
  }
  else if (start == 0){ // Waiting to check the system and push START button to begin workin
      analogWrite(servoPin, 127);
      delay(100);       
      able_to_work_msg.data = false;
      start = 0;
  }

  cleanCounter();
  
  length_msg.data = current_L;
  pub_length.publish(&length_msg);
  pub_able_to_work.publish(&able_to_work_msg);
  pub_count_steps_turn_motor.publish(&count_steps_turn_motor_msg);
  nh.spinOnce();
}
