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
#include <std_msgs/String.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define SENSOR_COUNT 1
#define TYPE_ANALOG 0
#define ANALOG_PIN 0

#define TYPE_ONE_WIRE 1
#define ONE_WIRE_PIN 1

#define MAX_SAMPLE_RATE 10
#define MAX_SAMPLE_COUNT 10
#define MAX_DELTA 0.5

typedef struct one_wire_data {
  OneWire one_wire;
  DallasTemperature dt_sensor;
} OneWireData;

typedef struct temperature_sensor {
  unsigned int id;
  float prev_temp;
  float curr_temp;
  unsigned int type;
  unsigned int pin;
  unsigned int sample_rate;
  unsigned int sample_count;
  unsigned long publisher_timer;
  OneWireData* one_wire_data;
} TemperatureSensor;

const TemperatureSensor default_sensor {
  0, 0.0f, 0.0f, 0, 0, 0, 0, 0
};

// Temperature Sensor variables
TemperatureSensor sensors[SENSOR_COUNT];
static const String json_prefix = "{\"id\":";
static const String json_middle = ",\"temperature\":";
static const String json_suffix = "}";

// ROS variables
static ros::NodeHandle  nh;
static std_msgs::String temp_msg;
static ros::Publisher pub_temp("temperature", &temp_msg);

static void increment_sample_rate(TemperatureSensor* sensor) {
  if(sensor->sample_rate == MAX_SAMPLE_RATE) {
    return;
  }
  sensor->sample_rate++;
}

static void decrement_sample_rate(TemperatureSensor* sensor) {
  if(sensor->sample_rate == 0) {
    return;
  }
  sensor->sample_rate--;
}

static void scale_rate(TemperatureSensor* sensor)
{
    if(sensor->prev_temp == sensor->curr_temp) {
       sensor->sample_count--;
    } else {
       sensor->sample_count++;
    }
    
    if(sensor->sample_count == MAX_SAMPLE_COUNT) {
      increment_sample_rate(sensor);
      sensor->sample_count = MAX_SAMPLE_COUNT >> 1;
    }
    else if(sensor->sample_count == 0) {
     decrement_sample_rate(sensor);
     sensor->sample_count = MAX_SAMPLE_COUNT >> 1;
   }
}

static void handle_temp(TemperatureSensor* sensor) {
  if(abs(sensor->prev_temp - sensor->curr_temp) > MAX_DELTA) {
    sensor->prev_temp = sensor->curr_temp;
    String message = json_prefix + sensor->id + json_middle + sensor->curr_temp + json_suffix;
    temp_msg.data = message.c_str();
    pub_temp.publish(&temp_msg);
  }
}

static void handle_analog(TemperatureSensor* sensor) {
  unsigned int raw_voltage = analogRead(sensor->pin);
  float volts = raw_voltage / 205.0f;
  float celsius = 100.0 * volts - 50;
  sensor->curr_temp = celsius;
}

static void handle_one_wire(TemperatureSensor* sensor) {
  sensor->one_wire_data->dt_sensor.requestTemperatures();
  sensor->curr_temp = sensor->one_wire_data->dt_sensor.getTempCByIndex(0);
}

static void create_sensor(unsigned int id, unsigned int pin, unsigned int type, TemperatureSensor* sensor) {
  *sensor = default_sensor;
  sensor->id = id;
  sensor->pin = pin;
  sensor->type = type;
  if(type == TYPE_ONE_WIRE) {
    sensor->one_wire_data = (OneWireData*)malloc(sizeof(OneWireData));
    sensor->one_wire_data->one_wire = OneWire(sensor->pin);
    sensor->one_wire_data->dt_sensor = DallasTemperature(&sensor->one_wire_data->one_wire);
    sensor->one_wire_data->dt_sensor.begin();
  }
}

void setup()
{  
  nh.initNode();
  nh.advertise(pub_temp);

  unsigned int sensor_count = 0;
  // Create analog temperature sensor
  TemperatureSensor analog_sensor;
  create_sensor(sensor_count, ANALOG_PIN, TYPE_ANALOG, &analog_sensor);
  sensors[sensor_count++] = analog_sensor;

  TemperatureSensor one_wire_sensor;
  create_sensor(sensor_count, ONE_WIRE_PIN, TYPE_ONE_WIRE, &one_wire_sensor);
  sensors[sensor_count++] = one_wire_sensor;
}

void loop()
{
  TemperatureSensor* sensor;
  for(unsigned int c = 0; c < SENSOR_COUNT; c++) {
    sensor = &sensors[c];
    if(millis() > sensor->publisher_timer) {
      switch(sensor->type) {
        case TYPE_ANALOG:
          handle_analog(sensor);
          break;
      }
      
      scale_rate(sensor);
      handle_temp(sensor);
  
      sensor->publisher_timer = millis() + (MAX_SAMPLE_RATE - sensor->sample_rate + 1) * 1000;
    }
  }
  
  nh.spinOnce();
}
