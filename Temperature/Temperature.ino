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

#define MAX_SAMPLE_RATE 10
#define MAX_SAMPLE_COUNT 10
#define MAX_DELTA 0.5

// ROS variables
static ros::NodeHandle  nh;
static std_msgs::Float32 temp_msg;
static ros::Publisher pub_temp("temperature", &temp_msg);
static unsigned long publisher_timer;

// Pinout variables
static unsigned int analog_pin = 0;

// Sample Rate variables
static unsigned int sample_rate = MAX_SAMPLE_RATE;
static unsigned int sample_count = 0;

// Temperature variables
static float prev_temp = 0.0f;

static void increment_sample_rate(void) {
  if(sample_rate == MAX_SAMPLE_RATE) {
    return;
  }
  sample_rate++;
}

static void decrement_sample_rate(void) {
  if(sample_rate == 0) {
    return;
  }
  sample_rate--;
}

static void scale_rate(float prev, float curr)
{
    if(prev == curr) {
       sample_count++;
    } else {
       sample_count--;
    }
    
    if(sample_count == MAX_SAMPLE_COUNT) {
      increment_sample_rate();
      sample_count = MAX_SAMPLE_COUNT >> 1;
    }
    else if(sample_count == 0) {
     decrement_sample_rate();
     sample_count = MAX_SAMPLE_COUNT >> 1;
   }
}

static void handle_temp(float temp) {
  if(abs(prev_temp - temp) > MAX_DELTA) {
    prev_temp = temp;
    temp_msg.data = temp;
    pub_temp.publish(&temp_msg);
  }
}

void setup()
{  
  nh.initNode();
  nh.advertise(pub_temp); 
}

void loop()
{
  if(millis() > publisher_timer) {
    int raw_voltage = analogRead(analog_pin);
    float volts = raw_voltage / 205.0f;
    float celsius = 100.0 * volts - 50;

    scale_rate(prev_temp, celsius);
    handle_temp(celsius);

    publisher_timer = millis() + (MAX_SAMPLE_RATE - sample_rate + 1) * 1000;
  }
  
  nh.spinOnce();
}
