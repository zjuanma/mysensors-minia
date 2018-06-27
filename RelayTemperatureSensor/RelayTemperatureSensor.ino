#include <MySensor.h>
#undef DEBUG
#include "debug.h"
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>

// ---- NODE
#define NODE_ID 110
#define CHILD_ID_LDR 3
#define CHILD_ID_MODE 2 // MODE true: AC, false: Heating
#define CHILD_ID_RELAY 1    // Relay for heating
#define CHILD_ID_TEMP 0

// ----- RELAY
#define RELAY_PIN  4  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay
#define PUSH_TIMEOUT 500 // Simulate push button on AC with a relay (milliseconds)

// LDR SENSOR
#define LDR_PIN A2

// ----- TEMPERATURE SENSOR
#define ONE_WIRE_BUS 3 // Pin where dallas sensor is connected 
const float TEMPERATURE_DELTA = 0.2; // Change of temperature to notify
const unsigned int READ_INTERVAL = 3; //READ Interval in seconds
const unsigned long SLEEP_TIME = READ_INTERVAL * 1000; // Sleep time between reads (in milliseconds)

const unsigned int MAX_ITERATION = (60 / READ_INTERVAL) * 3; // 3 minutes
unsigned int iteration = 0;

// Initialize message LDR
MyMessage msgLDR(CHILD_ID_LDR, V_LEVEL);

// Initialize temperature message
MyMessage msgTemperature(CHILD_ID_TEMP, V_TEMP);

// Initialize RELAY message
MyMessage msgRelay(CHILD_ID_RELAY, V_LIGHT);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature temperatureSensor(&oneWire);
float lastTemperature = 0.0;
uint8_t numSensors = 0;

// ----- GLOBAL
MySensor hw;
static char outstr[15];
static uint8_t MODE = 0;
uint8_t lastLDR = 0;

void setup()
{
  Serial.begin(115200); 
  // Initialize library and add callback for incoming messages
  hw.begin(incomingMessage, NODE_ID, true);
  // Send the sketch version information to the gateway and Controller
  hw.sendSketchInfo("Relay-Temperature Sensor", "2.1");

  // --- TEMPERATURE
  // Startup OneWire
  temperatureSensor.begin();
  // Fetch the number of attached temperature sensors
  numSensors = temperatureSensor.getDeviceCount();
  debug(PSTR("NS TEMP: %d\n"), numSensors);
  if (numSensors > 0) {
    hw.present(CHILD_ID_TEMP, S_TEMP);
  }

  // --- RELAY
  hw.present(CHILD_ID_RELAY, S_LIGHT);
  hw.present(CHILD_ID_MODE, S_BINARY);
  hw.present(CHILD_ID_LDR, S_LIGHT_LEVEL);
  
  // Set last mode 
  MODE = hw.loadState(CHILD_ID_MODE);

  // Then set relay pins in output mode
  pinMode(RELAY_PIN, OUTPUT);
  // Set relay to last known state (using eeprom storage)
  //digitalWrite(RELAY_PIN, hw.loadState(1) ? RELAY_ON : RELAY_OFF);
  digitalWrite(RELAY_PIN, RELAY_OFF);
}


void loop()
{
  debug(PSTR("IT: %u/%u\n"), iteration, MAX_ITERATION);

  // Process incoming messages (like config from server)
  hw.process();

  // Read temperatures and send them to controller
  if (numSensors > 0 ) {
    sendTemperature(readTemperature());
  }
  sendLDRStatus(analogRead(LDR_PIN));
  
  iteration = (iteration + 1) % MAX_ITERATION;
  
  hw.wait(SLEEP_TIME);
}

boolean temperatureChanged(float last_temperature, float temperature) {
  return abs(last_temperature - temperature) >= TEMPERATURE_DELTA;
}


float readTemperature() {
  // Fetch temperatures from Dallas sensors
  temperatureSensor.requestTemperatures();
  // Fetch and round temperature to one decimal
  float temperature = static_cast<float>(static_cast<int>((hw.getConfig().isMetric ? temperatureSensor.getTempCByIndex(CHILD_ID_TEMP) : temperatureSensor.getTempFByIndex(CHILD_ID_TEMP)) * 10.)) / 10.;
  debug(PSTR("T: %s\n"), dtostrf(temperature, 2, 1, outstr));
  
  return temperature;
}

void sendTemperature(float temperature) {
  if ((iteration == 0) || temperatureChanged(lastTemperature, temperature)) {
    debug(PSTR("Send temperature %s\n"), dtostrf(temperature, 2, 1, outstr));
    lastTemperature= temperature;
    sendValue(msgTemperature.set(temperature, 1));
  }
}


void sendLDRStatus(uint8_t value) {
   if ((iteration == 0) || (lastLDR - value) != 0) {
    debug(PSTR("Send LDR value %s\n"), dtostrf(value, 2, 1, outstr));
    lastLDR= value;
    sendValue(msgLDR.set(value, 1));
   }   
}

uint8_t sendValue(MyMessage &msg)
{
  hw.send(msg);
  return 0;
}

void incomingMessage(const MyMessage &message) {
  processModeMessage(message);
  processHeatingMessage(message);
  processACMessage(message);
}

void processHeatingMessage(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  if (!MODE && message.sensor == CHILD_ID_RELAY && message.type == V_LIGHT) {
    // Change relay state
    digitalWrite(RELAY_PIN, message.getBool() ? RELAY_ON : RELAY_OFF);
    // Store state in eeprom
    //hw.saveState(1, message.getBool());
    debug(PSTR("Incoming change for sensor: %d, old status: %d, new status: %d \n"), message.sensor, message.getBool() ? RELAY_ON : RELAY_OFF, message.getBool());
  }
}

void processACMessage(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  if (MODE && message.sensor == CHILD_ID_RELAY && message.type == V_LIGHT && message.getBool() == RELAY_ON) {
    digitalWrite(RELAY_PIN, RELAY_ON);
    hw.wait(PUSH_TIMEOUT);
    digitalWrite(RELAY_PIN, RELAY_OFF);
    debug(PSTR("Push button: %d \n"), message.sensor);
    uint8_t value = RELAY_OFF;
    sendValue(msgRelay.set(value,1));
  }
}

void processModeMessage(const MyMessage &message) {
  if (message.sensor == CHILD_ID_MODE && message.type == V_LIGHT) {
    MODE = message.getBool();
    // Store state in eeprom
    hw.saveState(CHILD_ID_MODE, MODE);
  }
}

