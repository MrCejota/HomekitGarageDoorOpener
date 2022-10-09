/*
 * simple_led.ino
 *
 * Setup code: 123-45-321
 * The Flash-Button(D3, GPIO0) on NodeMCU:
 *  Created on: 2020-02-08
 *      Author: Mixiaoxiao (Wang Bin)
 *  Edited on: 2020-03-01
 *      Edited by: euler271 (Jonas Linn)
 */

#include <Arduino.h>
#include <arduino_homekit_server.h>
#include "wifi_info.h"
#include "garage_door_opener_func.h"
#include "print_esp_info.h"

#define LOG_D(fmt, ...)   printf_P(PSTR(fmt "\n") , ##__VA_ARGS__);
#define PL(s) Serial.println(s)
#define P(s) Serial.print(s)

//D0 16 //led
//D3  0 //flash button
//D4  2 //led

#define RELAY_CONTROL_PIN 15 // Relay
//#define PIN_SENSOR_CLOSED     5 // Closed Sensor
//#define PIN_SENSOR_OPENED     4 // Open Sensor

//Temperature Sensor Pin
#define TEMPERATURE_PIN 0 //HUZZAH ESP8266 only has one Analog Input Pin: A0
float temperatureArray[10];
//Ultrasonic Sensor Pins
//***********************
 #define OPEN_TRIG_PIN 16
 #define OPEN_ECHO_PIN 14
//***********************
//Ultrasonic Sensor Distances in mm
//***********************
#define OPEN_DOOR_DISTANCE 200.0
#define MEASUREMENT_TOLERANCE 40.0
//***********************
#define DOOR_CLOSING_COUNTDOWN  500
// Door Status Values
float ambientTemp;
bool doorOpen;
bool doorOpenPrev;
bool doorClosed;
bool doorClosedPrev;
bool *doorOpenPointer;
bool *doorClosedPointer;
bool countdownActive;
//

// Position values
//***********************
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN 0
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED 1
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING 2
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING 3
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_STOPPED 4
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN 255

#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN 0
#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED 1
#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_UNKNOWN 255

#define HOMEKIT_CHARACTERISTIC_LOCK_CURRENT_STATE_UNSECURED 0
#define HOMEKIT_CHARACTERISTIC_LOCK_CURRENT_STATE_SECURED 1
#define HOMEKIT_CHARACTERISTIC_LOCK_CURRENT_STATE_JAMMED 2
#define HOMEKIT_CHARACTERISTIC_LOCK_CURRENT_STATE_UNKNOWN 3

#define HOMEKIT_CHARACTERISTIC_LOCK_TARGET_STATE_UNSECURED 0
#define HOMEKIT_CHARACTERISTIC_LOCK_TARGET_STATE_SECURED 1
#define HOMEKIT_CHARACTERISTIC_LOCK_TARGET_STATE_JAMMED 2
#define HOMEKIT_CHARACTERISTIC_LOCK_TARGET_STATE_UNKNOWN 3

//==============================
// Homekit setup and loop
//==============================

extern "C" homekit_server_config_t config;
extern "C" homekit_characteristic_t name;
extern "C" homekit_characteristic_t cha_current_door_state;
extern "C" homekit_characteristic_t cha_target_door_state;
extern "C" homekit_characteristic_t cha_obstruction_detected;
extern "C" homekit_characteristic_t cha_name;
extern "C" homekit_characteristic_t cha_lock_current_state;
extern "C" homekit_characteristic_t cha_lock_target_state;

// For reporting heap usage on the serial output every 5 seconds
uint32_t next_heap_millis = 0;
//
// Getters
//
// Called when getting current door state
homekit_value_t cha_current_door_state_getter() {

  // Stash the current state so we can detect a change
  homekit_characteristic_t current_state = cha_current_door_state;


  // Read the sensors and use some logic to determine state
  if (*doorOpenPointer) {
    // If PIN_SENSOR_OPENED is low, it's being pulled to ground, which means the switch at the top of the track is closed, which means the door is open
    cha_current_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN;
  } 
  else if (*doorClosedPointer) {
    // If PIN_SENSOR_CLOSED is low, it's being pulled to ground, which means the switch at the bottom of the track is closed, which means the door is closed
    cha_current_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED;
  } else {
    // If neither, then the door is in between switches, so we use the last known state to determine which way it's probably going
    if (current_state.value.uint8_value == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED) {
      // Current door state was "closed" so we are probably now "opening"
      cha_current_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING;
    } else if ( current_state.value.uint8_value == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN ) {
      // Current door state was "opened" so we are probably now "closing"
      cha_current_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING;
    }

    // If it is traveling, then it might have been started by the button in the garage. Set the new target state:
    if ( cha_current_door_state.value.uint8_value == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING ) {
      cha_target_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN;
    } else if ( cha_current_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING ) {
      cha_target_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED;
    }
    // ... and then notify HomeKit clients
    LOG_D("Target door state: %i", cha_target_door_state.value.uint8_value);
    homekit_characteristic_notify(&cha_target_door_state, cha_target_door_state.value);
  }

  LOG_D("Current door state: %i", cha_current_door_state.value.uint8_value);
  return cha_current_door_state.value;
}

// Called when getting current obstruction state
homekit_value_t cha_obstruction_detected_getter() {
  LOG_D("Obstruction: %s", cha_obstruction_detected.value.bool_value ? "Detected" : "Not detected");
  return cha_obstruction_detected.value;
}

// Called when getting current lock state
homekit_value_t cha_lock_current_state_getter() {
  LOG_D("Current lock state: %i", cha_lock_current_state.value.uint8_value);
  return cha_lock_current_state.value;
}

// 
// Setters
//

// Called when setting target door state
void cha_target_door_state_setter(const homekit_value_t value) {

  // State value requested by HomeKit
  cha_target_door_state.value = value;
  LOG_D("Target door state: %i", value.uint8_value);

  // If the current state is not equal to the target state, then we "push the button"; otherwise, we do nothing
  if (cha_current_door_state.value.uint8_value != cha_target_door_state.value.uint8_value) {
    digitalWrite(RELAY_CONTROL_PIN, HIGH);
    delay(500);
    digitalWrite(RELAY_CONTROL_PIN, LOW);
  }
  
}

void homekit_setup() {
   
	// Set the setters and getters
	cha_current_door_state.getter = cha_current_door_state_getter;
	cha_target_door_state.setter = cha_target_door_state_setter;
	cha_lock_current_state.getter = cha_lock_current_state_getter;
	cha_obstruction_detected.getter = cha_obstruction_detected_getter;

	arduino_homekit_setup(&config);
}

void homekit_loop() {

	arduino_homekit_loop();
  
	uint32_t time = millis();
	if (time > next_heap_millis) {
		INFO("heap: %d, sockets: %d", ESP.getFreeHeap(), arduino_homekit_connected_clients_count());
		next_heap_millis = time + 5000;
	}
}

void setup() {
  //set the pin modes
  pinMode(OPEN_TRIG_PIN, OUTPUT);
  pinMode(OPEN_ECHO_PIN, INPUT);
  pinMode(RELAY_CONTROL_PIN, OUTPUT);

  doorOpenPointer = &doorOpen;
  doorClosedPointer = &doorClosed;
  countdownActive = false;

  Serial.begin(115200);
  wifi_connect(); // in wifi_info.h
  Serial.setRxBufferSize(32);
  Serial.setDebugOutput(false);
  printESP_Info();

  ambientTemp = temperatureCalculation(TEMPERATURE_PIN);
  doorOpen = sensorTriggered(ultrasonicDistance(OPEN_TRIG_PIN, OPEN_ECHO_PIN, ambientTemp), OPEN_DOOR_DISTANCE, MEASUREMENT_TOLERANCE);

  if (doorOpen){
  Serial.println("Door Open");
  doorOpen = true;
  doorClosed = false;
  }  
  else {
  Serial.println("Door Closed");
  doorClosed = true;
  doorOpen = false;
  }
  doorOpenPrev = doorOpen;
  doorClosedPrev = doorClosed;

  //homekit_storage_reset(); // to remove the previous HomeKit pairing storage when you first run this new HomeKit example
  homekit_setup();

  // Initialize the current door state
  homekit_value_t current_state = cha_current_door_state_getter();
  homekit_characteristic_notify(&cha_current_door_state, cha_current_door_state.value);

   // Initialize target door state based on the current door state
  switch (cha_current_door_state.value.uint8_value) {
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED: 
      cha_target_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED; 
      break;
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING: 
      cha_target_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED; 
      break;
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN: 
      cha_target_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN; 
      break;
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING: 
      cha_target_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN; 
      break;
    default: 
      cha_target_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_UNKNOWN; 
      break;
  }  
  homekit_characteristic_notify(&cha_target_door_state, cha_target_door_state.value);
}

void loop() {

bool valueChange;
unsigned int countingDown;

  ambientTemp = temperatureCalculation(TEMPERATURE_PIN);
  doorOpen = sensorTriggered(ultrasonicDistance(OPEN_TRIG_PIN, OPEN_ECHO_PIN, ambientTemp), OPEN_DOOR_DISTANCE, MEASUREMENT_TOLERANCE);
if (doorOpen){
  Serial.println("Sensing Distance Met");
  doorOpen = true;
  doorClosedPrev = false;
}  
else {
  Serial.println("Sensing Distance Not Met");
  doorOpen = false;
}

if(doorOpenPrev && !doorOpen && !countdownActive){
  countdownActive = true;
  countingDown = 0;
}

if (!countdownActive && (countingDown < DOOR_CLOSING_COUNTDOWN)){
  countingDown++;
}
if ((countingDown == DOOR_CLOSING_COUNTDOWN) && !doorClosedPrev){
  doorClosed = true;
  countdownActive = false;
  Serial.println("Door Closed");
}

if (doorOpen != doorOpenPrev || doorClosed!=doorClosedPrev){
  homekit_value_t new_state = cha_current_door_state_getter();
  homekit_characteristic_notify(&cha_current_door_state, new_state);
  doorOpenPrev = doorOpen;
  doorClosedPrev = doorClosed;  
}

  homekit_loop();
  delay(10);
}
