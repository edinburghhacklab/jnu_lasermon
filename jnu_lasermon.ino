#include <stdint.h>         // for uint8_t
#include <avr/interrupt.h>  // for interrupts
#include <JeeLib.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define RF_FREQ RF12_868MHZ
const byte rf_group = 38;
const byte rf_node = 11;

const byte rf_enable_pin = 0;
const byte laser_enable_pin = 3; // see also references to PA7
const byte laser_analog_pin = A1;
const byte one_wire_pin = 10;
const byte red_led_pin = 8;
const byte green_led_pin = 7;

const int interval = 5000;
const int session_timeout = 10000;

#define TEMPERATURE_PRECISION 10
OneWire oneWire(one_wire_pin);
DallasTemperature sensors(&oneWire);
DeviceAddress sensor0;
DeviceAddress sensor1;
DeviceAddress sensor2;
float temp0;
float temp1;
float temp2;

volatile uint8_t portahistory;

//volatile unsigned long laser_power_time = 0;   // time spent powered up
volatile unsigned long laser_session_time = 0; // time spent active (including time between cuts)
volatile unsigned long laser_cutting_time = 0; // time spent cutting (excluding time between cuts) 
volatile unsigned long laser_analog_time = 0;  // time spent cutting, adjusted according to power level

volatile unsigned long last_tx = 0;
volatile unsigned long last_interrupt_time;
volatile boolean last_interrupt_state;
//volatile unsigned int laser_level;

volatile unsigned long last_temperature_tx_time = 0;
volatile unsigned long last_laser_tx_time = 0;

struct temperaturePacket {
  int temperature0; // temperature in degrees C * 100
  int temperature1; // temperature in degrees C * 100
  int temperature2; // temperature in degrees C * 100
} 
temperature_packet;

struct laserPacket {
  boolean active;
  unsigned int power_time; // seconds powered-up
  unsigned int session_time; // seconds of cutting session (including intra-cut time)
  unsigned int cutting_time; // seconds of cutting time (laser-on)
  unsigned int analog_time; // seconds of cutting time (adjusted for power level)
  unsigned int pulse_count; // number of laser-on events
  unsigned int error_count; // number of missed laser-on-off changes
  unsigned int laser_level; // current analog laser level 0-1023
} 
laser_packet;

void setup() {
  portahistory = PINA;

  pinMode(red_led_pin, OUTPUT);
  pinMode(green_led_pin, OUTPUT);
  //digitalWrite(red_led_pin, HIGH);

  // initialize the radio
  pinMode(rf_enable_pin, OUTPUT);
  digitalWrite(rf_enable_pin, LOW);
  rf12_initialize(rf_node, RF_FREQ, rf_group);

  // configure ADC to use internal reference
  //ADMUX |= (1 << REFS1);

  // configure interrupts
  GIMSK  |= (1 << PCIE0);     // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT7);   // set PCINT7 to trigger an interrupt on state change 
  sei();                    // turn on interrupts

  // initialize the temperature sensor
  //int sensorCount = 0;
  sensors.begin();
  sensors.getAddress(sensor0, 0);
  sensors.getAddress(sensor1, 1);
  sensors.getAddress(sensor2, 2);

  // set precision
  sensors.setResolution(sensor0, TEMPERATURE_PRECISION);
  sensors.setResolution(sensor1, TEMPERATURE_PRECISION);
  sensors.setResolution(sensor2, TEMPERATURE_PRECISION);

  // throw away the first reading and give the sensor time to start-up
  //sensors.requestTemperaturesByAddress(sensor0);
  //sensors.requestTemperaturesByAddress(sensor1);
  //sensors.requestTemperaturesByAddress(sensor2);
  //Sleepy::loseSomeTime(3000);
  
  // do the initial transmit so the receiver can see that we've started up
  transmit();
  
}

void readTemperatures() {
  sensors.requestTemperaturesByAddress(sensor0);
  temperature_packet.temperature0 = (int)(sensors.getTempC(sensor0)*100);
  sensors.requestTemperaturesByAddress(sensor1);
  temperature_packet.temperature1 = (int)(sensors.getTempC(sensor1)*100);
  sensors.requestTemperaturesByAddress(sensor2);
  temperature_packet.temperature2 = (int)(sensors.getTempC(sensor2)*100);
}

void transmit() {
  int i;

  last_tx = millis();

  // update the laser stats packet
  laser_packet.power_time = millis()/1000;
  laser_packet.session_time = laser_session_time/1000;
  laser_packet.cutting_time = laser_cutting_time/1000;
  laser_packet.analog_time = laser_analog_time/1000;

  // wait for the radio to become ready
  i = 0;
  while (!rf12_canSend() && i<10) {
    rf12_recvDone();
    i++;
  }

  // send the data
  rf12_sendStart(0, &laser_packet, sizeof laser_packet);
  rf12_sendWait(0);

  readTemperatures();

  // wait for the radio to become ready
  i = 0;
  while (!rf12_canSend() && i<10) {
    rf12_recvDone();
    i++;
  }

  // send the data
  rf12_sendStart(0, &temperature_packet, sizeof temperature_packet);
  rf12_sendWait(0);

}

void loop() {

  if (millis()-last_interrupt_time > session_timeout && last_interrupt_state == 0) {
    digitalWrite(green_led_pin, LOW);
    laser_packet.active = 0;
  } 
  else {
    laser_packet.active = 1;
  }
  
  laser_packet.laser_level = analogRead(laser_analog_pin);
  
  if (millis() > last_tx+interval) {
    transmit();
  }

}

ISR (PCINT0_vect)
{
  uint8_t changedbits;
  boolean new_state;

  changedbits = PINA ^ portahistory;
  portahistory = PINA;

  if(changedbits & (1 << PA7)) {
    new_state = !((PINA & (1 << PA7)) >> PA7);
  } 
  else {
    return;
  }
  
  if (new_state == 1) {
    laser_packet.pulse_count++;
  }

  if (new_state == last_interrupt_state) {
    // we must have missed a change
    laser_packet.error_count++;
    last_interrupt_time = millis();
    return;
  }

  if (new_state == 1) {
    // laser has switched on
    digitalWrite(red_led_pin, HIGH);
    digitalWrite(green_led_pin, HIGH);
    if (millis()-last_interrupt_time < session_timeout) {
      // laser has been switched-off for a short time only
      laser_session_time += (millis()-last_interrupt_time);
    }
    laser_packet.laser_level = analogRead(laser_analog_pin);
  } 
  else {
    // laser has switched off
    digitalWrite(red_led_pin, LOW);
    laser_session_time += (millis()-last_interrupt_time);
    laser_cutting_time += (millis()-last_interrupt_time);
    laser_analog_time += ((millis()-last_interrupt_time)*laser_packet.laser_level)/1024;
  }

  last_interrupt_state = new_state;
  last_interrupt_time = millis();
}



