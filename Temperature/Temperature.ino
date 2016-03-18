//TODO: Create "library" for each component
#include <Wire.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define SENSOR_COUNT 6
#define TYPE_ANALOG 0
#define ANALOG_PIN 0

#define TYPE_ONE_WIRE 1
#define ONE_WIRE_PIN 1

#define TYPE_I2C 2
#define TMP102_ADDR_0 0xF4628
#define TMP102_ADDR_1 0xF4629
#define TMP102_ADDR_2 0xF4632
#define TMP102_ADDR_3 0xF4633

#define MAX_SAMPLE_RATE 10
#define MAX_SAMPLE_COUNT 4
#define MAX_DELTA 0.5

#define HEATER_PIN 22

typedef struct one_wire_data {
  OneWire one_wire;
  DallasTemperature dt_sensor;
} OneWireData;

typedef struct temperature_sensor {
  uint32_t id;
  float prev_temp;
  float curr_temp;
  uint32_t type;
  uint32_t pin_addr;
  uint32_t sample_rate;
  uint32_t sample_count;
  unsigned long publisher_timer;
  OneWireData* one_wire_data;
} TemperatureSensor;

const TemperatureSensor default_sensor {
  0, 0.0f, 0.0f, 0, 0, 0, 0, 0, NULL
};

// Temperature Sensor variables
TemperatureSensor sensors[SENSOR_COUNT];
static const String json_prefix = "{\"id\":";
static const String json_middle = ",\"temperature\":";
static const String json_suffix = "}";

// ROS variables
static ros::NodeHandle  nh;
static std_msgs::String temp_msg;
static std_msgs::UInt8 count_msg;
static ros::Publisher pub_temp("temperature", &temp_msg);
static ros::Publisher pub_count("temp_sensor_count", &count_msg);

static void temperature_callback(const std_msgs::UInt8& id) {
  if(id.data > SENSOR_COUNT) {
    return;
  }
  TemperatureSensor* sensor = &sensors[id.data];
  send_message(sensor);
}

static void count_callback(const std_msgs::Empty& empty) {
  count_msg.data = (uint8_t)sizeof(sensors) / sizeof(sensors[0]);
  pub_temp.publish(&temp_msg);
}

static void heater_callback(const std_msgs::Bool& state) {
  if(state.data) {
    digitalWrite(HEATER_PIN, HIGH);
  } else {
    digitalWrite(HEATER_PIN, LOW);
  }
}

static ros::Subscriber<std_msgs::UInt8> sub_temp("req_temperature", &temperature_callback);
static ros::Subscriber<std_msgs::Empty> sub_count("req_count", &count_callback);
static ros::Subscriber<std_msgs::Bool> sub_heater("heater", &heater_callback);

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
    if(abs(sensor->prev_temp - sensor->curr_temp) <= MAX_DELTA) {
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

static void send_message(TemperatureSensor* sensor) {
  String message = json_prefix + sensor->id + json_middle + sensor->curr_temp + json_suffix;
  temp_msg.data = message.c_str();
  pub_temp.publish(&temp_msg);
}

static void handle_temp(TemperatureSensor* sensor) {
  if(abs(sensor->prev_temp - sensor->curr_temp) > MAX_DELTA) {
    sensor->prev_temp = sensor->curr_temp;
    send_message(sensor);
  }
}

static void handle_analog(TemperatureSensor* sensor) {
  uint32_t raw_voltage = analogRead(sensor->pin_addr);
  float volts = raw_voltage / 205.0f;
  float celsius = 100.0 * volts - 50;
  sensor->curr_temp = celsius;
}

static void handle_one_wire(TemperatureSensor* sensor) {
  sensor->one_wire_data->dt_sensor.requestTemperatures();
  sensor->curr_temp = sensor->one_wire_data->dt_sensor.getTempCByIndex(0);
}

static void handle_i2c(TemperatureSensor* sensor) {
  int byte_req = 2;
  Wire.requestFrom(sensor->pin_addr, byte_req); 
  delay(10);
  if (Wire.available() >= byte_req) {
    byte msb;
    byte lsb;
    uint32_t temperature;
    
    msb = Wire.read(); // receive high byte (full degrees)
    lsb = Wire.read(); // receive low byte (fraction degrees) 
    temperature = ((msb) << 4); // MSB
    temperature |= (lsb >> 4); // LSB

    sensor->curr_temp = temperature * 0.0625f;
  }
}

static void create_sensor(uint32_t id, uint32_t pin_addr, uint32_t type, TemperatureSensor* sensor) {
  *sensor = default_sensor;
  sensor->id = id;
  sensor->pin_addr = pin_addr;
  sensor->type = type;
  if(type == TYPE_ONE_WIRE) {
    sensor->one_wire_data = (OneWireData*)malloc(sizeof(OneWireData));
    sensor->one_wire_data->one_wire = OneWire(sensor->pin_addr);
    sensor->one_wire_data->dt_sensor = DallasTemperature(&sensor->one_wire_data->one_wire);
    sensor->one_wire_data->dt_sensor.begin();
  }
}

void setup(void)
{  
  nh.initNode();
  nh.advertise(pub_temp);
  nh.advertise(pub_count);
  nh.subscribe(sub_temp);
  nh.subscribe(sub_count);
  nh.subscribe(sub_heater);
  
  uint32_t sensor_count = 0;
  // Create analog temperature sensor
  TemperatureSensor analog_sensor;
  create_sensor(sensor_count, ANALOG_PIN, TYPE_ANALOG, &analog_sensor);
  sensors[sensor_count++] = analog_sensor;

  // Create one wire temperature sensor
  TemperatureSensor one_wire_sensor;
  create_sensor(sensor_count, ONE_WIRE_PIN, TYPE_ONE_WIRE, &one_wire_sensor);
  sensors[sensor_count++] = one_wire_sensor;

  // Create I2C temperature sensors
  TemperatureSensor i2c_sensor0;
  create_sensor(sensor_count, TMP102_ADDR_0, TYPE_I2C, &i2c_sensor0);
  sensors[sensor_count++] = i2c_sensor0;

  // Set heater output pin and drive low
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
}

void loop(void)
{
  TemperatureSensor* sensor;
  for(uint32_t c = 0; c < SENSOR_COUNT; c++) {
    sensor = &sensors[c];
    if(millis() > sensor->publisher_timer) {
      switch(sensor->type) {
        case TYPE_ANALOG:
          handle_analog(sensor);
          break;
        case TYPE_ONE_WIRE:
          handle_one_wire(sensor);
          break;
        case TYPE_I2C:
          handle_i2c(sensor);
          break;
      }
      
      scale_rate(sensor);
      handle_temp(sensor);
  
      sensor->publisher_timer = millis() + (MAX_SAMPLE_RATE - sensor->sample_rate + 1) * 1000;
    }
  }
  
  nh.spinOnce();
}
