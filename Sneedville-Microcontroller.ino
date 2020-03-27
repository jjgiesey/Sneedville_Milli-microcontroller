/*
  created 20 Mar 2020 by Casey Williams
  last modified 27 Mar 2020 by Casey Williams

  Program for the Ruggeduino in the system-monitoring design for the Cedar Grove Baptist Church clean water project in
  Sneedville. The code and other relevant files will be updated and maintained as part of a senior design project for the 2020
  spring semester at the following GitHub repository:

  https://github.com/Milligan-Engineering/Sneedville-Microcontroller

  Should the project require maintenance/changes/improvements/additions in the future, this repository should serve as a
  reference for other contributors.

  A brief overview of the structure of this code and how it is used in the project:
  - the REOZONATE LED and speaker alert are triggered for all reset sources (power-on, external, watchdog system, and brown-out)
  - whenever reset from a safe (uncontaminated) system state, clear the LED/speaker alert with the RESET_REOZONATE button
  - calibration points and decision thresholds are grouped together as global constants
  - UV-C fluence rate [mW/cm²], pressure [psig], and differential pressure [psiΔ] readings are handled by timer-based interrupts
  - for these sensors/transmitters, interrupt frequencies were chosen to be > 2.205 / (response time from datasheet [s])
  - the interrupts also handle the LED outputs and button/switch inputs
  - system flow, UV-C fluence [mW-s/cm²], temperature, and speaker output are determined once per ~49 seconds by the main loop

  For more information on the project, see the 'README.md' file in the Github repository.
*/

#include <EEPROM.h> // non-volatile memory access to store states if power is reset

enum {
  // digital pin input names:
  MUTE = 2,
  RESET_REOZONATE = 3,
  RESET_FILTERS = 6,
  RESET_UV_BULB = 9,

  // digital pin output names:
  SPEAKER = 1,
  REOZONATE = 4,
  REPLACE_FILTERS = 7,
  REPLACE_UV_BULB = 10,
  SPARE_RELAY_1 = 11, // not currently used
  SPARE_RELAY_2 = 12, // not currently used
  TRICKLE_VALVE = 13,

  // analog pin input names:
  UV_C_SENSOR = A0,
  PRESSURE_TRANSMITTER = A1,
  TEMPERATURE_SENSOR = A2,
  DIFF_PRESSURE_TRANSMITTER = A3,

  // pressure transmitter and differential pressure transmitter calibration array names:
  HI_PSI = 0, // high pressure [psig] or differential pressure [psiΔ]
  LO_PSI = 1, // low pressure [psig] or differential pressure [psiΔ]
  HI_VOLTAGE = 2, // voltage [V] at HI_PSI
  LO_VOLTAGE = 3 // voltage [V] at LO_PSI
};

const float cal_dpt[] = {5.0, 0.0, 2.381187, 1.01841}; // differential pressure transmitter calibration values
const float cal_pt[] = {65.0, 0.0, 4.61, 0.58}; // pressure transmitter calibration values
const float max_diff_pressure = 7.0; // differential pressure [psid] over which the REPLACE_FILTERS LED is triggered
const float max_pressure = 20.0; // pressure [psig] threshold for determining whether system is flowing
const float trickle_temperature = 20.0; // external temperature [°F] under which the trickle valve is to open
const float uv_bulb_unsafe = 30.0; // UV-C fluence [mW-s/cm²] under which the REOZONATE LED is triggered
const float uv_bulb_warning = 34.5; // UV-C fluence [mW-s/cm²] under which the REPLACE_UV_BULB LED is triggered

bool flow = false; // true when flow is detected
bool reozonate_state = true; // REOZONATE LED turns on after any reset
bool replace_filters_state = false;
bool replace_uv_bulb_state = false;
float avg_fluence = 0.0; // average UV-C fluence [mW-s/cm²] for each 49-second interval
long fluence_it = -1; // number of UV-C fluence iterations averaged for each 49-second interval

void setup() {
  noInterrupts(); // stop interrupts

  replace_filters_state = (bool)EEPROM.read(REPLACE_FILTERS); // read REPLACE_FILTERS state from non-volatile memory
  replace_uv_bulb_state = (bool)EEPROM.read(REPLACE_UV_BULB); // read REPLACE_UV_BULB state from non-volatile memory

  // initialize digital pin inputs with internal pull-up resistors enabled:
  pinMode(RESET_REOZONATE, INPUT_PULLUP);
  pinMode(RESET_FILTERS, INPUT_PULLUP);
  pinMode(RESET_UV_BULB, INPUT_PULLUP);

  // initialize digital pin outputs:
  pinMode(SPEAKER, OUTPUT);
  pinMode(REOZONATE, OUTPUT);
  pinMode(REPLACE_FILTERS, OUTPUT);
  pinMode(REPLACE_UV_BULB, OUTPUT);
  pinMode(TRICKLE_VALVE, OUTPUT);

  // set digital pin outputs inital states:
  digitalWrite(SPEAKER, LOW);
  digitalWrite(REOZONATE, HIGH);
  digitalWrite(REPLACE_FILTERS, (uint8_t)replace_filters_state);
  digitalWrite(REPLACE_UV_BULB, (uint8_t)replace_uv_bulb_state);
  digitalWrite(TRICKLE_VALVE, LOW);

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

float voltage(const int input_pin) {
  return analogRead(input_pin) / 1023.0 * 5.0; // convert 10-bit ADC value to voltage [V]
  // (voltage [V]) = (analog reading) / (max analog reading) * (analog reference voltage [V])
}

float pressure() {
  static const float pt_slope = (cal_pt[HI_PSI] - cal_pt[LO_PSI]) / (cal_pt[HI_VOLTAGE] - cal_pt[LO_VOLTAGE]);
  return (voltage(PRESSURE_TRANSMITTER) - cal_pt[LO_VOLTAGE]) * pt_slope; // pressure [psig]
}

ISR(TIMER0_COMPA_vect) {
  if ((!flow) && (pressure() < max_pressure))
    flow = true; // flow is reset to false every 49-second interval
}

bool update_LED(const uint8_t which_LED, const bool on_off) {
  if (which_LED == REPLACE_FILTERS || which_LED == REPLACE_UV_BULB)
    EEPROM.write((int)which_LED, (uint8_t)on_off); // store state in non-volatile memory
  digitalWrite(which_LED, (uint8_t)on_off);
  return on_off;
}

float diff_pressure() {
  static const float dpt_slope = (cal_dpt[HI_PSI] - cal_dpt[LO_PSI]) / (cal_dpt[HI_VOLTAGE] - cal_dpt[LO_VOLTAGE]);
  return (voltage(DIFF_PRESSURE_TRANSMITTER) - cal_dpt[LO_VOLTAGE]) * dpt_slope; // differential pressure [psiΔ]
}

ISR(TIMER1_COMPA_vect) {
  if (digitalRead(RESET_FILTERS) == LOW)
    replace_filters_state = update_LED(REPLACE_FILTERS, false); // RESET_FILTERS button pressed
  else if ((!replace_filters_state) && (diff_pressure() > max_diff_pressure))
    replace_filters_state = update_LED(REPLACE_FILTERS, true);
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

ISR(TIMER2_COMPA_vect) {
  static float prev_fluence_rate = fluence_rate();
  static unsigned long prev_t = micros();
  ++fluence_it;
  if (fluence_it > 0)
    avg_fluence = new_avg_fluence(fluence_rate(), prev_fluence_rate, micros(), prev_t);

  if (digitalRead(MUTE) == LOW)
    digitalWrite(SPEAKER, LOW); // speaker MUTE switch on

  if (digitalRead(RESET_UV_BULB) == LOW)
    replace_uv_bulb_state = update_LED(REPLACE_UV_BULB, false); // RESET_UV_BULB button pressed

  if (digitalRead(RESET_REOZONATE) == LOW)
    reozonate_state = update_LED(REOZONATE, false); // RESET_REOZONATE button pressed
}

void speaker_control() {
  if ((digitalRead(MUTE) == HIGH) && (reozonate_state || replace_filters_state || replace_uv_bulb_state))
    digitalWrite(SPEAKER, HIGH); // speaker MUTE switch is off and at least one LED is on
  else
    digitalWrite(SPEAKER, LOW);
}

float read_temperature() {
  return ((voltage(TEMPERATURE_SENSOR) - 750.0e-3) / 10.0e-3 + 25.0) * 9.0 / 5.0 + 32.0; // temperature [°F]
  // (temperature [°F]) = (((voltage [V]) - (cal voltage [V])) / (slope [V/°C]) + (cal temp [°C])) * 9 [°F] / 5 [°C] + 32 [°F]
}

void trickle_control() {
  if ((!flow) && (read_temperature() < trickle_temperature))
    digitalWrite(TRICKLE_VALVE, HIGH); // open trickle valve
  else
    digitalWrite(TRICKLE_VALVE, LOW); // close trickle valve
  flow = false; // reset flow detection for next 49-second interval
}

void evaluate_avg_fluence() {
  if (!replace_uv_bulb_state && avg_fluence < uv_bulb_warning)
    replace_uv_bulb_state = update_LED(REPLACE_UV_BULB, true);

  if (!reozonate_state && avg_fluence < uv_bulb_unsafe)
    reozonate_state = update_LED(REOZONATE, true);
}

void loop() {
  static bool skip_first_evaluation = true;

  speaker_control();
  delay(49000);
  trickle_control();

  if (!skip_first_evaluation)
    evaluate_avg_fluence();
  else
    skip_first_evaluation = false; // skip first evaluation of average UV-C fluence to allow UV bulb to warm up
  fluence_it = 0; // reset UV-C fluence iteration count for next 49-second interval
}
