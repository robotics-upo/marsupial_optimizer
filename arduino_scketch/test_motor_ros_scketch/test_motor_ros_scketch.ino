// Define Libraries
#include <ros.h>
#include <std_msgs/UInt16.h>

// Define Variables
int servoPin = 6;        // servo motor connected to digital pin 6
int PWM;

void pwmSubCallback(const std_msgs::UInt16& pwm_msg) {
    PWM = pwm_msg.data;
}

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::UInt16> sub("set_pwm", &pwmSubCallback );

void setup() {
    // Put your setup code here, to run once:
    pinMode(servoPin, OUTPUT);    // sets the digital pin 6 as output
    
    // ROS Config
    nh.initNode();
    nh.subscribe(sub);
}

void loop() { 
  analogWrite(servoPin, PWM);
  delay(3000);
  PWM = 0;
  
  nh.spinOnce();
}
