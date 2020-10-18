#include <config.h>
#include <Arduino.h>
#include <HX711_ADC.h>
#include <EEPROM.h>

void pulse_event();
void tachometer();

//HX711 constructor (dout pin, sck pin)
HX711_ADC torque_load_cell(TORQUE_LC_DT, TORQUE_LC_SCK); //HX711 1
HX711_ADC thrust_load_cell(THRUST_LC_DT, THRUST_LC_SCK); //HX711 2
long t;

const byte PulsesPerRevolution = 2;  
const unsigned long ZeroTimeout = 100000; 
const byte numReadings = 2;  

volatile unsigned long LastTimeWeMeasured;  
volatile unsigned long PeriodBetweenPulses = ZeroTimeout + 1000; 
volatile unsigned long PeriodAverage = ZeroTimeout + 1000;
unsigned long FrequencyRaw; 
unsigned long FrequencyReal;  
unsigned long RPM;
unsigned int PulseCounter = 1; 
unsigned long PeriodSum;
unsigned long LastTimeCycleMeasure = LastTimeWeMeasured;  
unsigned long CurrentMicros = micros();
unsigned int AmountOfReadings = 1;

unsigned int ZeroDebouncingExtra;  

unsigned long readings[numReadings];  // The input.
unsigned long readIndex;  // The index of the current reading.
unsigned long total;  // The running total.
unsigned long average;  // The RPM value after applying the smoothing.

void setup() {
  Serial.begin(115200); delay(10);
  Serial.println();
  Serial.println("Starting...");

  DDRD |= (1 << DDD7);
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
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  tachometer();

  // check for new data/start next conversion:
  if (torque_load_cell.update()) newDataReady = true;
  thrust_load_cell.update();

  //get smoothed value from data set
  if ((newDataReady)) {
    if (millis() > t + serialPrintInterval) {
      float a = torque_load_cell.getData();
      float b = thrust_load_cell.getData();
      Serial.print("\tTorque: ");
      Serial.print(a);
      Serial.print("\tThrust: ");
      Serial.print(b);

      Serial.print("\tTachometer: ");
      Serial.println(average);
      newDataReady = 0;
      t = millis();
    }
  }

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
}

void pulse_event()  // The interrupt runs this to calculate the period between pulses:
{

  PeriodBetweenPulses = micros() - LastTimeWeMeasured;  // Current "micros" minus the old "micros" when the last pulse happens.
  // This will result with the period (microseconds) between both pulses.
  // The way is made, the overflow of the "micros" is not going to cause any issue.

  LastTimeWeMeasured = micros();  // Stores the current micros so the next time we have a pulse we would have something to compare with.





  if (PulseCounter >= AmountOfReadings) // If counter for amount of readings reach the set limit:
  {
    PeriodAverage = PeriodSum / AmountOfReadings;  // Calculate the final period dividing the sum of all readings by the
    // amount of readings to get the average.
    PulseCounter = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
    PeriodSum = PeriodBetweenPulses;  // Reset PeriodSum to start a new averaging operation.


    // Change the amount of readings depending on the period between pulses.
    // To be very responsive, ideally we should read every pulse. The problem is that at higher speeds the period gets
    // too low decreasing the accuracy. To get more accurate readings at higher speeds we should get multiple pulses and
    // average the period, but if we do that at lower speeds then we would have readings too far apart (laggy or sluggish).
    // To have both advantages at different speeds, we will change the amount of readings depending on the period between pulses.
    // Remap period to the amount of readings:
    int RemapedAmountOfReadings = map(PeriodBetweenPulses, 40000, 5000, 1, 10);  // Remap the period range to the reading range.
    // 1st value is what are we going to remap. In this case is the PeriodBetweenPulses.
    // 2nd value is the period value when we are going to have only 1 reading. The higher it is, the lower RPM has to be to reach 1 reading.
    // 3rd value is the period value when we are going to have 10 readings. The higher it is, the lower RPM has to be to reach 10 readings.
    // 4th and 5th values are the amount of readings range.
    RemapedAmountOfReadings = constrain(RemapedAmountOfReadings, 1, 10);  // Constrain the value so it doesn't go below or above the limits.
    AmountOfReadings = RemapedAmountOfReadings;  // Set amount of readings as the remaped value.
  }
  else
  {
    PulseCounter++;  // Increase the counter for amount of readings by 1.
    PeriodSum = PeriodSum + PeriodBetweenPulses;  // Add the periods so later we can average.
  }

}  // End of pulse_event.

void tachometer() {
// The following is going to store the two values that might change in the middle of the cycle.
  // We are going to do math and functions with those values and they can create glitches if they change in the
  // middle of the cycle.
  LastTimeCycleMeasure = LastTimeWeMeasured;  // Store the LastTimeWeMeasured in a variable.
  CurrentMicros = micros();  // Store the micros() in a variable.
  if (CurrentMicros < LastTimeCycleMeasure)
  {
    LastTimeCycleMeasure = CurrentMicros;
  }
  FrequencyRaw = 10000000000 / PeriodAverage;  // Calculate the frequency using the period between pulses.

  if (PeriodBetweenPulses > ZeroTimeout - ZeroDebouncingExtra || CurrentMicros - LastTimeCycleMeasure > ZeroTimeout - ZeroDebouncingExtra)
  { // If the pulses are too far apart that we reached the timeout for zero:
    FrequencyRaw = 0;  // Set frequency as 0.
    ZeroDebouncingExtra = 2000;  // Change the threshold a little so it doesn't bounce.
  }
  else
  {
    ZeroDebouncingExtra = 0;  // Reset the threshold to the normal value so it doesn't bounce.
  }

  FrequencyReal = FrequencyRaw * 0.0001;  // Get frequency without decimals.
  // This is not used to calculate RPM but we remove the decimals just in case
  // you want to print it.

  // Calculate the RPM:
  RPM = FrequencyRaw / PulsesPerRevolution * 60;  // Frequency divided by amount of pulses per revolution multiply by
  // 60 seconds to get minutes.
  RPM = RPM * 0.0001;  // Remove the decimals.

  // Smoothing RPM:
  total = total - readings[readIndex];  // Advance to the next position in the array.
  readings[readIndex] = RPM;  // Takes the value that we are going to smooth.
  total = total + readings[readIndex];  // Add the reading to the total.
  readIndex = readIndex + 1;  // Advance to the next position in the array.

  if (readIndex >= numReadings)  // If we're at the end of the array:
  {
    readIndex = 0;  // Reset array index.
  }

  // Calculate the average:
  average = total / numReadings;  // The average value it's the smoothed result.
}