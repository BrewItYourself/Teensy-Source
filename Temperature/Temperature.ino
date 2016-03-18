/*
 * rosserial Temperature Sensor Example
 * 
 * This tutorial demonstrates the usage of the
 * Sparkfun TMP102 Digital Temperature Breakout board
 * http://www.sparkfun.com/products/9418
 * 
 * Source Code Based off of:
 * http://wiring.org.co/learning/libraries/tmp102sparkfun.html
 */

#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

std_msgs::Float32 temp_msg;
ros::Publisher pub_temp("temperature", &temp_msg);

unsigned int analog_pin = 20;

void setup()
{  
  nh.initNode();
  nh.advertise(pub_temp); 
}

long publisher_timer;

void loop()
{
  if(millis() > publisher_timer) {
    int raw_voltage = analogRead(analog_pin);
    float volts = raw_voltage / 205.0;
    float celsius = 100.0 * volts - 50;
  
    temp_msg.data = celsius;
    pub_temp.publish(&temp_msg);
    
    publisher_timer = millis() + 1000;
  }
  
  nh.spinOnce();
}
