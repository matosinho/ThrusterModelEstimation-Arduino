#include <config.h>
#include <Arduino.h>
#include <HX711_ADC.h>
#include <EEPROM.h>

void pulse_event();
void tachometer();

long loop_timer, esc_timer;
int esc_1 = 1000;

//HX711 constructor (dout pin, sck pin)
HX711_ADC torque_load_cell(TORQUE_LC_DT, TORQUE_LC_SCK); //HX711 1
HX711_ADC thrust_load_cell(THRUST_LC_DT, THRUST_LC_SCK); //HX711 2
long t;

float a, b;

unsigned long last_time_measurement, period_between_pulses;
double RPM;
bool first_measurement = true;

void setup() {
  Serial.begin(115200); delay(10);
  Serial.println();
  Serial.println("Starting...");

  DDRB |= (1 << DDB1);
  attachInterrupt(digitalPinToInterrupt(2), pulse_event, RISING);

  torque_load_cell.begin();
  thrust_load_cell.begin();
  long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  byte torque_load_cell_rdy = 0;
  byte thrust_load_cell_rdy = 0;
  while ((torque_load_cell_rdy + thrust_load_cell_rdy) < 2) { //run startup, stabilization and tare, both modules simultaniously
    if (!torque_load_cell_rdy) torque_load_cell_rdy = torque_load_cell.startMultiple(stabilizingtime, _tare);
    if (!thrust_load_cell_rdy) thrust_load_cell_rdy = thrust_load_cell.startMultiple(stabilizingtime, _tare);
  }
  if (torque_load_cell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }
  if (thrust_load_cell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }
  torque_load_cell.setCalFactor(TORQUE_LC_CALVAL); // user set calibration value (float)
  thrust_load_cell.setCalFactor(THRUST_LC_CALVAL); // user set calibration value (float)
  Serial.println("Startup is complete");
}

void loop() {

  if (millis() < 10000)
    analogWrite(9, 127);
  else if(millis() >= 10000 && millis() <= 20000)
    analogWrite(9, 190);
  else 
    analogWrite(9, 127);

  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (torque_load_cell.update()) newDataReady = true;
  thrust_load_cell.update();

  //get smoothed value from data set
  if ((newDataReady)) {
    if (millis() > t + serialPrintInterval) {
      a = torque_load_cell.getData();
      b = thrust_load_cell.getData();
      newDataReady = 0;
      t = millis();
    }
  }

  if (period_between_pulses > 6) {
    RPM = 0.7*RPM + ((1.0 /(double) (period_between_pulses * 0.001)) * 60.0) * 0.3;  
    
    Serial.print(a);
    Serial.print("\t");
    Serial.print(b);

    Serial.print("\t");
    Serial.println(RPM);
  }
  period_between_pulses = 0;
  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    float i;
    char inByte = Serial.read();
    if (inByte == 't') {
      torque_load_cell.tareNoDelay();
      thrust_load_cell.tareNoDelay();
    }
  }

  //check if last tare operation is complete
  if (torque_load_cell.getTareStatus() == true) {
    Serial.print("Tare load cell 1 complete");
  }
  if (thrust_load_cell.getTareStatus() == true) {
    Serial.print("Tare load cell 2 complete");
  }

  // if (millis() > 5000)
  // esc_1 = 1300;

  // if (millis() > 15000)
  //   esc_1 = 1000;  
  
  // if ((PORTB & (1 << PORTB0)) && (micros() >= esc_timer))
  //     PORTB &= ~(1 << PORTB0);
}

void pulse_event()  // The interrupt runs this to calculate the period between pulses:
{ 
  period_between_pulses = millis() - last_time_measurement;
  last_time_measurement = millis();
}  // End of pulse_event.