/*
  created 20 Mar 2020 by Casey Williams
  last modified 26 Apr 2020 by Casey Williams

  Program for the Ruggeduino in the system-monitoring design originally developed for the Cedar Grove Baptist Church clean water
  project in Sneedville. The code and other relevant files will be updated and maintained as part of a senior design project for
  the 2020 spring semester at the following GitHub repository:

  https://github.com/jjgiesey/Sneedville_Milli-microcontroller

  The generic version of the project that lacks the freeze protection features that are specialized to the Sneedville
  implementation is maintained in a different GitHub repository:

  https://github.com/Milligan-Engineering/Milli-microcontroller

  Should the project require maintenance/changes/improvements/additions in the future, this repository should serve as a
  reference for other contributors.

  A brief overview of the structure of this code and how it is used in the project:
  - the REOZONATE LED is triggered for all reset sources (power-on, external, watchdog system, and brown-out)
  - whenever reset from a safe (uncontaminated) system state, clear the LED/speaker alert with the RESET_REOZONATE button
  - calibration points and decision thresholds, the most likely values that need to be changed, are grouped as global constants
  - UV-C fluence rate [mW/cm²], pressure [psig], and differential pressure [psiΔ] readings are handled by timer-based interrupts
  - for these sensors/transmitters, interrupt frequencies were chosen to be > 2.205 / (response time from datasheet [s])
  - the interrupts also handle the LED outputs and button/switch inputs
  - the SOLENOID_POWER, HEATER_POWER, and UV_POWER switches correspond to being on when HIGH and off when LOW
  - system flow, UV-C fluence [mW-s/cm²], and temperatures are determined once per ~49 seconds by the main loop
  - general code order: globals -> setup/loop -> interrupts -> sensor/transmitter read functions -> output decision functions

  For more information on the project, see the 'README.md' file in the Github repository.
*/

#include <EEPROM.h> // non-volatile memory access to store states if power is reset

enum {
  // digital pin input names:
  RESET_REOZONATE = 1,
  RESET_FILTERS = 2,
  RESET_UV_BULB = 3,
  SOLENOID_POWER = 7,
  HEATER_POWER = 8,
  UV_POWER = 9,

  // digital pin output names:
  REOZONATE = 4,
  REPLACE_FILTERS = 5,
  REPLACE_UV_BULB = 6,
  UV = 10,
  SOLENOID = 12,
  HEATER = 13,

  // analog pin input names:
  UV_C_SENSOR = A0,
  DIFF_PRESSURE_TRANSMITTER = A1,
  PRESSURE_TRANSMITTER = A2,
  INSIDE_TEMPERATURE_SENSOR = A3,
  OUTSIDE_TEMPERATURE_SENSOR = A4,

  // pressure transmitter and differential pressure transmitter calibration array names:
  HI_PSI = 0, // high pressure [psig] or differential pressure [psiΔ]
  LO_PSI = 1, // low pressure [psig] or differential pressure [psiΔ]
  HI_VOLTAGE = 2, // voltage [V] at HI_PSI
  LO_VOLTAGE = 3 // voltage [V] at LO_PSI
};

const float cal_dpt[] = {5.0, 0.0, 2.381187, 1.01841}; // differential pressure transmitter calibration values
const float cal_pt[] = {65.0, 0.0, 4.61, 0.58}; // pressure transmitter calibration values
const float heater_temperature = 35.0; // internal temperature [°F] under which the heater is to be on
const float max_diff_pressure = 7.0; // differential pressure [psid] over which the REPLACE_FILTERS LED is triggered
const float max_pressure = 19.0; // pressure [psig] threshold for determining whether system is flowing
const float trickle_temperature = 20.0; // external temperature [°F] under which the trickle valve is to open
const float uv_bulb_unsafe = 30.0; // UV-C fluence [mW-s/cm²] under which the REOZONATE LED is triggered
const float uv_bulb_warning = 34.5; // UV-C fluence [mW-s/cm²] under which the REPLACE_UV_BULB LED is triggered

bool flow = false; // true when flow is detectedreozonate_state
bool heater_state = false;
bool reozonate_state = true; // REOZONATE LED turns on after any reset
bool replace_filters_state = false;
bool replace_uv_bulb_state = false;
bool solenoid_state = false;
bool uv_state = false;
float avg_fluence = 0.0; // average UV-C fluence [mW-s/cm²] for each 49-second interval
long fluence_it = -1; // incremental number of UV-C fluence iterations averaged for each 49-second interval

void setup() {
  noInterrupts(); // stop interrupts

  replace_filters_state = (bool)EEPROM.read(REPLACE_FILTERS); // read REPLACE_FILTERS state from non-volatile memory
  replace_uv_bulb_state = (bool)EEPROM.read(REPLACE_UV_BULB); // read REPLACE_UV_BULB state from non-volatile memory

  // initialize digital pin inputs with internal pull-up resistors enabled:
  pinMode(RESET_REOZONATE, INPUT_PULLUP);
  pinMode(RESET_FILTERS, INPUT_PULLUP);
  pinMode(RESET_UV_BULB, INPUT_PULLUP);
  pinMode(SOLENOID_POWER, INPUT_PULLUP);
  pinMode(HEATER_POWER, INPUT_PULLUP);
  pinMode(UV_POWER, INPUT_PULLUP);

  // initialize digital pin outputs:
  pinMode(REOZONATE, OUTPUT);
  pinMode(REPLACE_FILTERS, OUTPUT);
  pinMode(REPLACE_UV_BULB, OUTPUT);
  pinMode(UV, OUTPUT);
  pinMode(SOLENOID, OUTPUT);
  pinMode(HEATER, OUTPUT);

  // set digital pin outputs inital states:
  digitalWrite(REOZONATE, HIGH);
  digitalWrite(REPLACE_FILTERS, (uint8_t)replace_filters_state);
  digitalWrite(REPLACE_UV_BULB, (uint8_t)replace_uv_bulb_state);
  digitalWrite(UV, LOW);
  digitalWrite(SOLENOID, LOW);
  digitalWrite(HEATER, LOW);

  // set TIMER0 interrupt at 1.1 kHz for reading pressure transmitter:
  TCCR0A = 1 << WGM01; // set clear timer on compare mode
  TCCR0B = 1 << CS01 | 1 << CS00; // set prescaler to 64
  TCNT0 = 0; // reset counter to 0
  OCR0A = 225; // set output compare register A to trigger on count 225: 16 MHz / (64 * (225 + 1)) = 1.1 kHz
  TIMSK0 = 1 << OCIE0A; // enable interrupt on output compare A match

  // set TIMER1 interrupt at 44.1 Hz for reading differential pressure transmitter:
  TCCR1A = 0;
  TCCR1B = 1 << WGM12 | 1 << CS11; // set clear timer on compare mode and prescaler to 8
  TCNT1  = 0; // reset counter to 0
  OCR1A = 45350; // set output compare register A to trigger on count 45350: 16 MHz / (8 * (45350 + 1)) = 44.1 Hz
  TIMSK1 = 1 << OCIE1A; // enable interrupt on output compare A match

  //set TIMER2 interrupt at 2.4 kHz for reading UV-C sensor:
  TCCR2A = 1 << WGM21; // set clear timer on compare mode
  TCCR2B = 1 << CS21 | 1 << CS20; //set prescaler to 32
  TCNT2 = 0; // reset counter to 0
  OCR2A = 208; // set output compare register A to trigger on count 208: 16 MHz / (65 * (508 + 1)) = 2.4 kHz
  TIMSK2 = 1 << OCIE2A; // enable interrupt on output compare A match

  interrupts(); // start interrupts
}

void loop() {
  static bool skip_first_evaluation = true;

  delay(49000);

  if (!skip_first_evaluation)
    evaluate_avg_fluence();
  else
    skip_first_evaluation = false; // skip first evaluation of average UV-C fluence to allow UV bulb to warm up
  heater_control();
  trickle_control();
  flow = false; // reset flow detection for next 49-second interval
  fluence_it = 0; // reset UV-C fluence iteration count for next 49-second interval
}

ISR(TIMER0_COMPA_vect) {
  if ((!flow) && (pressure() < max_pressure))
    flow = true; // flow is reset to false every 49-second interval
}

ISR(TIMER1_COMPA_vect) {
  if (digitalRead(RESET_FILTERS) == LOW)
    replace_filters_state = update_output(REPLACE_FILTERS, false); // RESET_FILTERS button pressed
  else if ((!replace_filters_state) && (diff_pressure() > max_diff_pressure))
    replace_filters_state = update_output(REPLACE_FILTERS, true);
}

ISR(TIMER2_COMPA_vect) {
  static float prev_fluence_rate = fluence_rate();
  static unsigned long prev_t = micros();
  ++fluence_it;
  if (fluence_it > 0)
    avg_fluence = new_avg_fluence(fluence_rate(), prev_fluence_rate, micros(), prev_t);

  if (digitalRead(RESET_REOZONATE) == LOW)
    reozonate_state = update_output(REOZONATE, false); // RESET_REOZONATE button pressed

  if (digitalRead(RESET_UV_BULB) == LOW)
    replace_uv_bulb_state = update_output(REPLACE_UV_BULB, false); // RESET_UV_BULB button pressed

  uv_state = update_output(UV, (bool)digitalRead(UV_POWER)); // match UV power state to UV_POWER switch input state

  if (digitalRead(SOLENOID_POWER) == LOW && solenoid_state)
    solenoid_state = update_output(SOLENOID, false); // override solenoid power state to off

  if (digitalRead(HEATER_POWER) == LOW && heater_state)
    heater_state = update_output(HEATER, false); // override heater power state to off
}

float voltage(const int input_pin) {
  return analogRead(input_pin) / 1023.0 * 5.0; // convert 10-bit ADC value to voltage [V]
  // (voltage [V]) = (analog reading) / (max analog reading) * (analog reference voltage [V])
}

float diff_pressure() {
  static const float slope = (cal_dpt[HI_PSI] - cal_dpt[LO_PSI]) / (cal_dpt[HI_VOLTAGE] - cal_dpt[LO_VOLTAGE]);
  return (voltage(DIFF_PRESSURE_TRANSMITTER) - cal_dpt[LO_VOLTAGE]) * slope + cal_dpt[LO_PSI]; // differential pressure [psiΔ]
}

float pressure() {
  static const float slope = (cal_pt[HI_PSI] - cal_pt[LO_PSI]) / (cal_pt[HI_VOLTAGE] - cal_pt[LO_VOLTAGE]);
  return (voltage(PRESSURE_TRANSMITTER) - cal_pt[LO_VOLTAGE]) * slope + cal_pt[LO_PSI]; // pressure [psig]
}

float inside_temperature() {
  return ((voltage(INSIDE_TEMPERATURE_SENSOR) - 750.0e-3) / 10.0e-3 + 25.0) * 9.0 / 5.0 + 32.0; // temperature [°F]
  // (temperature [°F]) = (((voltage [V]) - (cal voltage [V])) / (slope [V/°C]) + (cal temp [°C])) * 9 [°F] / 5 [°C] + 32 [°F]
}

float outside_temperature() {
  return ((voltage(OUTSIDE_TEMPERATURE_SENSOR) - 750.0e-3) / 10.0e-3 + 25.0) * 9.0 / 5.0 + 32.0; // temperature [°F]
  // (temperature [°F]) = (((voltage [V]) - (cal voltage [V])) / (slope [V/°C]) + (cal temp [°C])) * 9 [°F] / 5 [°C] + 32 [°F]
}

float fluence_rate() {
  return voltage(UV_C_SENSOR) / (1.13e6 * 1.01 * 42.0e-9); // UV-C fluence rate [mW/cm²]
  // (fluence rate [mW/cm²]) = (voltage [V]) / ((resistance [Ω]) * (1 + tolerance) * (max photocurrent slope [A-cm²/mW]))
}

float new_avg_fluence(const float new_fluence_rate, float &prev_fluence_rate, const unsigned long new_t, unsigned long &prev_t) {
  // estimate area under fluence-rate curve for most recent time step:
  float new_fluence = (new_fluence_rate + prev_fluence_rate) / 2.0 * (new_t - prev_t);

  prev_fluence_rate = new_fluence_rate;
  prev_t = new_t;
  return (avg_fluence * (fluence_it - 1) + new_fluence) / fluence_it; // fluence [mW-s/cm²];
}

bool update_output(const uint8_t which_output, const bool on_off) {
  if (which_output == REPLACE_FILTERS || which_output == REPLACE_UV_BULB)
    EEPROM.write((int)which_output, (uint8_t)on_off); // store state in non-volatile memory
  digitalWrite(which_output, (uint8_t)on_off);
  return on_off;
}

void evaluate_avg_fluence() {
  if (!replace_uv_bulb_state && avg_fluence < uv_bulb_warning)
    replace_uv_bulb_state = update_output(REPLACE_UV_BULB, true);

  if (!reozonate_state && ((flow && !uv_state) || avg_fluence < uv_bulb_unsafe))
    reozonate_state = update_output(REOZONATE, true);
}

void heater_control() {
  if ((digitalRead(HEATER_POWER) == HIGH) && (inside_temperature() < heater_temperature))
    heater_state = update_output(HEATER, true); // turn heater on
  else
    heater_state = update_output(HEATER, false); // turn heater off
}

void trickle_control() {
  if ((digitalRead(SOLENOID_POWER) == HIGH) && (!flow) && (outside_temperature() < trickle_temperature))
    solenoid_state = update_output(SOLENOID, true); // open trickle valve
  else
    solenoid_state = update_output(SOLENOID, false); // close trickle valve
}
