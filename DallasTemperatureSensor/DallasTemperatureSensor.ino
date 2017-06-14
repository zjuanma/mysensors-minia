
// Example sketch showing how to send in OneWire temperature readings
#include <MySensor.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Vcc.h>

#undef DEBUG
#include "debug.h"

#define NODE_ID 102

#define ONE_WIRE_BUS 3 // Pin where dallas sensor is connected 

const float TEMPERATURE_DELTA = 0.3; // Change of temperature to notify
const unsigned int READ_INTERVAL = 13; //READ Interval in seconds
const unsigned long SLEEP_TIME = READ_INTERVAL * 1000; // Sleep time between reads (in milliseconds)

const unsigned int MAX_ITERATION = (60 / READ_INTERVAL) * 60; // 60 minutes
unsigned int iteration = 0;

const float VccCorrection = 1.0 / 1.0; // Measured Vcc by multimeter divided by reported Vcc
const float MIN_V = 2.4; // 2.4 - 1.8
const float MAX_V = 3.2;// 3.4 - 3.2
Vcc vcc(VccCorrection);

const unsigned int MAX_BATT_ITERATION = (60 / READ_INTERVAL) * 60 * 12; //6 hours in minutes
unsigned int battery_iteration = 0;


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature temperatureSensor(&oneWire);
float lastTemperature = 0;
uint8_t numSensors = 0;

MySensor node;
#define RETRY 5

// Initialize temperature message
#define CHILD_ID_TEMP 0
#define CHILD_ID_BATT 1
MyMessage msgTemperature(CHILD_ID_TEMP, V_TEMP);
MyMessage msgBattery(CHILD_ID_BATT, V_VOLTAGE);

static char outstr[15];

void setup()
{
  // Startup and initialize MySensors library. Set callback for incoming messages.
  node.begin(NULL, NODE_ID);
  // Send the sketch version information to the gateway and Controller
  node.sendSketchInfo("Temperature Sensor ds18b20", "2.0");

  // Startup OneWire
  temperatureSensor.begin();
  // Fetch the number of attached temperature sensors
  numSensors = temperatureSensor.getDeviceCount();

  // Present all sensors to controller
  debug(PSTR("NS: %d\n"), numSensors);
  if (numSensors > 0) {
    node.present(CHILD_ID_TEMP, S_TEMP);
  }
  node.present(CHILD_ID_BATT, S_POWER);
}


void loop()
{
  debug(PSTR("IT: %u/%u BATT_IT: %u/%u\n"), iteration, MAX_ITERATION, battery_iteration, MAX_BATT_ITERATION);

  // Process incoming messages (like config from server)
  node.process();

  // Read temperatures and send them to controller
  if (numSensors > 0 ) {
    sendTemperature(readTemperature()); 
  }

  sendBattery();

  node.sleep(SLEEP_TIME);
}

boolean temperatureChanged(float last_temperature, float temperature) {
  return abs(last_temperature - temperature) >= TEMPERATURE_DELTA;
}

float readTemperature() {
  // Fetch temperatures from Dallas sensors
  temperatureSensor.requestTemperatures();
  // Fetch and round temperature to one decimal
  float temperature = static_cast<float>(static_cast<int>((node.getConfig().isMetric ? temperatureSensor.getTempCByIndex(CHILD_ID_TEMP) : temperatureSensor.getTempFByIndex(CHILD_ID_TEMP)) * 10.)) / 10.;
  debug(PSTR("T: %s\n"), dtostrf(temperature, 2, 1, outstr));
  
  return temperature;
}

void sendTemperature(float temperature) {
  if ((iteration == 0) || temperatureChanged(lastTemperature, temperature)) {
    debug(PSTR("Send temperature %s\n"), dtostrf(temperature, 2, 1, outstr));
    lastTemperature= temperature;
    iteration = sendValue(msgTemperature.set(temperature, 1), RETRY);
  }
  iteration = (iteration + 1) % MAX_ITERATION;
}

void sendBattery() {
  // Send battery level
  if ( battery_iteration == 0) {
    float batteryVolt = vcc.Read_Volts();
    float batteryPerc = vcc.Read_Perc(MIN_V, MAX_V);
    debug(PSTR("BL: %s V -> %d %%\n"), dtostrf(batteryVolt, 2, 3, outstr), static_cast<int>(batteryPerc));
    node.sendBatteryLevel(static_cast<int>(batteryPerc));
    sendValue(msgBattery.set(batteryVolt, 3), RETRY); 
  }
  battery_iteration = (battery_iteration + 1) % MAX_BATT_ITERATION;
}

uint8_t sendValue(MyMessage &message, int retry)
{
  bool result = false;
  uint8_t i = 0;
  for (i = 0; i < retry ; i++ ) {
    result = node.send(message); 
    if (result) break;
    node.sleep(SLEEP_TIME);
  }
  return i;
}
