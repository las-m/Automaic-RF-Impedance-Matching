// settings for 3.8 cm diameter single coil, tuned at 10 dBm, range: 15 MHz 

#include <EEPROM.h> //EEPROM is used to save the stepper locations during powerdown
#include <AccelStepper.h>
#define PARALLEL_DIR_PIN 2
#define PARALLEL_STEP_PIN 3
#define SERIES_DIR_PIN 4
#define SERIES_STEP_PIN 5
#define motorInterfaceType 1
#define motorReset 6
#define motorSleep 7
#define M0 8
#define M1 9
#define M2 10
#define resetPositionIn 16
#define tuningActivationIn 17
#define analogPin A3
#define LEDPin 13
AccelStepper parallel = AccelStepper(motorInterfaceType, PARALLEL_STEP_PIN, PARALLEL_DIR_PIN);
AccelStepper series = AccelStepper(motorInterfaceType, SERIES_STEP_PIN, SERIES_DIR_PIN);
int parallel_previous_position = 0;
bool supposed_to_tune = true;

// Settings:
float MAX_RPS = 1.2; // Maximum stepper Speed
int STEPS_PER_ROTATION = 800; // Please enter how many microsteps correspond to one rotation, considereing the stepper and driver
int baseline = 245; // ~ For tunign to < -30 dB reflection: 300 When tuning with + 10 dBm, 350 When tuning with 0 dBm (50 channels per 10 dB);
int noticable_channel_delta = 16;
int succesfull_value = 280;


void setup() {
   Serial.begin(9600);
   pinMode(LEDPin, OUTPUT);
   pinMode(motorReset, OUTPUT);
   pinMode(motorSleep, OUTPUT);
   pinMode(M1, OUTPUT);
   pinMode(tuningActivationIn, INPUT);
   digitalWrite(motorReset, HIGH);
   digitalWrite(M1, HIGH); // Set DRV8825 to four microsteps per step
   analogReference(INTERNAL); // Sets Reference Voltage to 2.56 V (use on Arduino micro only!!
   // ADC compares to analogReference voltage: ZX47-60-S+ Power Detector slope -25 mV/dB, Arduino Resolution: 2.5 mV ADC value, 5mV per 2xADC value
   // -> If analogRead(analogPin)/2 changes by 5, the detected power changes by 1dB 
   // -60dB = 2.1V=840ADC = 420 value
   // -30dB = 1.4V=560ADC = 280 value
   // 0dB = 0.65V = 132ADC = 64 value

   // Manual position reset
   if(digitalRead(resetPositionIn ) == HIGH) {
      int parallel_start_position = parallel.currentPosition();
      updateEEPROM(0, parallel_start_position);
      updateEEPROM(4, parallel_start_position);
   }
}

void loop() {
  if (digitalRead(tuningActivationIn) == HIGH) {supposed_to_tune = true;}
  else { supposed_to_tune = false; }

  int val = analogRead(analogPin) / 2;  // read the input pin
  printReading(val);
  delay(100);

  // Tune, if reflected power exceeds threshhold for more than 1 second
  if (supposed_to_tune == true && val < baseline) {
    val = analogRead(analogPin) / 2;
    if(val < baseline) {
      delay(900);
      val = analogRead(analogPin) / 2; 
      if(val < baseline) {}
        val = analogRead(analogPin) / 2;
        if(val < baseline) {tune();};
    }
  }
}

void tune() {
  analogWrite(LEDPin, 180);
  initializeMotors();
  if (find_minimum() == false) {
    inputOutOfRange();
    return;
  }
  match_real_impedance();
  if (analogRead(analogPin) < succesfull_value) {retune;}
  finishTuning();
  analogWrite(LEDPin, 00);
}

void retune() {
  if(parallel.currentPosition() > STEPS_PER_ROTATION) {
    series.move(2*STEPS_PER_ROTATION);
  } else if (parallel.currentPosition() < -25 * STEPS_PER_ROTATION) {
    series.move(- 2*STEPS_PER_ROTATION);
  }
  series.runToPosition();
  find_minimum();
  match_real_impedance();
}


/*
 * Main tuning functions:
 */

// Scan the range of the parallel capacitor to find the value with Im(Z) = 0 for the circuit
bool find_minimum() {
  int val = 0;
  int opt_val = analogRead(analogPin) / 2;
  int optimum = parallel.currentPosition();
  bool keep_going = true;
  
  // Home the parallel capacitor
  parallel.move( - parallel_previous_position);
  parallel.runToPosition();
  parallel.setCurrentPosition(0);

  //scan
  parallel.move(- 26 * STEPS_PER_ROTATION);
  do {
    parallel.run();
    if(parallel.distanceToGo() % (STEPS_PER_ROTATION/8) == 0) {
      val = analogRead(analogPin) / 2;
      printReading(val);
      if(val >  opt_val) {
        optimum = parallel.currentPosition();
        opt_val = val;
        }
      if(val < int(opt_val - 12)) {keep_going = false;} // stop if reflected power larger than "12"~5 dB more than optimal reflection
      }
    } while (parallel.distanceToGo() != 0 && keep_going == true);

  // Check if reflection minimum was found  
  if (opt_val - (analogRead(analogPin) / 2) < (3 * noticable_channel_delta / 2) && keep_going == true) {
    opt_val = hard_to_find_minimum();
    if (opt_val - val < (3 * noticable_channel_delta / 2)) {return false;}
  } else {
    parallel.moveTo(optimum - STEPS_PER_ROTATION/8);
    parallel.runToPosition();
  }

  // precision tune
  precision_tune_parallel(STEPS_PER_ROTATION/32);
  return true;
}

void match_real_impedance() {
  float correction_factor = 0.87; // Measured relative slope of linear trimmer capacitors
  int opt_val = 0;
  int val;
  int new_val;
  int scan_start_position;
  int optimum_parallel;
  int optimum_series;
  int range = 5;
  bool keep_going = true;
   if ((analogRead(analogPin) / 2) > 240) {range = 3;}
   else if ((analogRead(analogPin) / 2) > 220 ) {range = 4;}

  // scan for optimal position of series capacitor
  // 1. safeguard if parallel stepper is close to the capacitors turn limit
  if (parallel.currentPosition() + (range * STEPS_PER_ROTATION * correction_factor / 2) > 0) {
    series.move((range * STEPS_PER_ROTATION) + (parallel.currentPosition()  /  correction_factor));
    parallel.moveTo(- range * STEPS_PER_ROTATION * correction_factor);
  } else if (parallel.currentPosition() - range * STEPS_PER_ROTATION * correction_factor / 2 < - 26 * STEPS_PER_ROTATION) {
    series.move((26 * STEPS_PER_ROTATION + parallel.currentPosition()) / correction_factor);
    parallel.moveTo(- 26 * STEPS_PER_ROTATION);
  } else if (parallel.currentPosition() < parallel_previous_position -  8 * STEPS_PER_ROTATION) {
  // begin scan right here
  } else if (parallel.currentPosition() > parallel_previous_position +  8 * STEPS_PER_ROTATION) {
    series.move((range) * STEPS_PER_ROTATION);
    parallel.move(- (range) * STEPS_PER_ROTATION * correction_factor);    
  } else {
    series.move(range * STEPS_PER_ROTATION / 2);
    parallel.move(- range * STEPS_PER_ROTATION * correction_factor / 2);
  }
  series.runToPosition();
  parallel.runToPosition();
  scan_start_position = series.currentPosition();

  // 2. scan
  do {
    series.move(- STEPS_PER_ROTATION / 8);
    series.runToPosition();
    precision_tune_parallel(STEPS_PER_ROTATION/32);
    val = analogRead(analogPin) / 2;
    printReading(val);

    if (val > opt_val) {
      optimum_parallel = parallel.currentPosition();
      optimum_series = series.currentPosition();
      opt_val = val;
    }

    if (val < opt_val - 12) {keep_going = false;} 
  } while (series.currentPosition() > scan_start_position - (range * STEPS_PER_ROTATION) && keep_going == true);

  series.moveTo(optimum_series - STEPS_PER_ROTATION/32);
  series.runToPosition();
  parallel.moveTo(optimum_parallel + STEPS_PER_ROTATION/16);
  parallel.runToPosition();
  
  // precision tune
  precision_tune_parallel(STEPS_PER_ROTATION/32);
  
  new_val = analogRead(analogPin) / 2;
  printReading(new_val);
  do {
  val = new_val;
  series.move(10);
  series.runToPosition();
  catchup(-4);
  new_val = analogRead(analogPin) / 2;
  printReading(new_val);
  } while (val <= new_val);

  if(new_val < succesfull_value) {
    // if optimum was found at edge of range, run impedance match optimazation in that region
    if (scan_start_position - series.currentPosition() <  STEPS_PER_ROTATION / 3 ||
        scan_start_position - series.currentPosition() >  (range - 1/3)* STEPS_PER_ROTATION) {
      if (parallel.currentPosition() < - STEPS_PER_ROTATION * 3/2 && parallel.currentPosition() > - 26 * STEPS_PER_ROTATION) {
        hard_to_match_real_impedance();
      }
    }
  }
}


/*
 * Tuner Helper Functions
 */

void precision_tune_parallel(float step_size) {
  int val = analogRead(analogPin) / 2;
  int new_val = val;
  do{
    val = new_val;
    parallel.move(step_size);
    parallel.runToPosition();
    new_val = analogRead(analogPin) / 2;
    printReading(new_val);  
  } while (val <= new_val);
  parallel.move(- (step_size  + (float_sgn(step_size) * 4)));
  parallel.runToPosition();
}

void catchup(int stepwidth) {
  int val;
  int new_val = analogRead(analogPin) / 2;
  printReading(new_val);
  do {
    val = new_val;
    parallel.move(stepwidth);
    parallel.runToPosition();
    new_val = analogRead(analogPin) / 2;
    printReading(new_val);
  } while (val <= new_val);
  parallel.move(- (stepwidth + (int_sgn(stepwidth) * 4)));
  parallel.runToPosition();
}

// If no minimum could be found, tries again with different series capacitance
int hard_to_find_minimum() {
  // Try different series capacitance
  if (parallel_previous_position < - 12 * STEPS_PER_ROTATION) {
    series.move( 2.5 * STEPS_PER_ROTATION);
  } else {
    series.move(- 2.5 * STEPS_PER_ROTATION);
  }
  series.runToPosition();

  int val = 0;
  int opt_val = analogRead(analogPin) / 2;
  int optimum = parallel.currentPosition();
  bool keep_going = true;

  //scan
  parallel.moveTo(0);
  do {
    parallel.run();
    if(parallel.distanceToGo() % (STEPS_PER_ROTATION/8) == 0) {
      val = analogRead(analogPin) / 2;
      printReading(val);
   
      if(val >  opt_val) {
        optimum = parallel.currentPosition();
        opt_val = val;
      }
      if(val < int(opt_val - 12)) {keep_going = false;}
    }
  } while (parallel.distanceToGo() != 0 && keep_going == true);

  parallel.moveTo(optimum - STEPS_PER_ROTATION/6);
  parallel.runToPosition();
  return opt_val;
}

// If optimal impedance match was at the edge of the scanned range, scan around that point again
void hard_to_match_real_impedance() {
  float correction_factor = 0.87; // Measured relative capacitance slopes of linear trimmer capacitors
  int range = 3;
  int val;
  int new_val;
  int opt_val = 0;
  int scan_start_position;
  int optimum_parallel;
  int optimum_series;
  bool keep_going = true;

  // 1. safeguard if parallel stepper is close to the capacitors turn limit
  if (parallel.currentPosition() + (range * STEPS_PER_ROTATION * correction_factor / 2) > 0) {
    series.move((range * STEPS_PER_ROTATION) + (parallel.currentPosition()  /  correction_factor));
    parallel.moveTo(- range * STEPS_PER_ROTATION * correction_factor);
  } else if (parallel.currentPosition() - range * STEPS_PER_ROTATION * correction_factor / 2 < - 26 * STEPS_PER_ROTATION) {
    series.move((26 * STEPS_PER_ROTATION + parallel.currentPosition()) / correction_factor);
    parallel.moveTo(- 26 * STEPS_PER_ROTATION);
  } else if (parallel.currentPosition() < parallel_previous_position -  8 * STEPS_PER_ROTATION) {
  // begin scan right here
  } else if (parallel.currentPosition() > parallel_previous_position +  8 * STEPS_PER_ROTATION) {
    series.move((range) * STEPS_PER_ROTATION);
    parallel.move(- (range) * STEPS_PER_ROTATION * correction_factor);    
  } else {
    series.move(range * STEPS_PER_ROTATION / 2);
    parallel.move(- range * STEPS_PER_ROTATION * correction_factor / 2);
  }

  series.move(range * STEPS_PER_ROTATION / 2);
  parallel.move(- range * STEPS_PER_ROTATION * correction_factor / 2);
  series.runToPosition();
  parallel.runToPosition();
  scan_start_position = series.currentPosition();

  // 2. scan
  do {
    series.move(- STEPS_PER_ROTATION / 8);
    series.runToPosition();
    precision_tune_parallel(STEPS_PER_ROTATION/32);
    val = analogRead(analogPin) / 2;
    printReading(val);

    if (val > opt_val) {
      optimum_parallel = parallel.currentPosition();
      optimum_series = series.currentPosition();
      opt_val = val;
    }

    if (val < opt_val - 12) {keep_going = false;} 
  } while (series.currentPosition() > scan_start_position - (range * STEPS_PER_ROTATION) && keep_going == true);

  series.moveTo(optimum_series - STEPS_PER_ROTATION/32);
  series.runToPosition();
  parallel.moveTo(optimum_parallel + STEPS_PER_ROTATION/16);
  parallel.runToPosition();
  
  precision_tune_parallel(STEPS_PER_ROTATION/32);
  
  new_val = analogRead(analogPin) / 2;
  printReading(new_val);
  do {
  val = new_val;
  series.move(10);
  series.runToPosition();
  catchup(-4);
  new_val = analogRead(analogPin) / 2;
  printReading(new_val);
  } while (val <= new_val);
}


/*
 * miscellaneous
 */
// powering on motor drivers, setting motor speeds, resetting motor position
void initializeMotors() {
   digitalWrite(motorSleep, HIGH);
   parallel.setMaxSpeed(STEPS_PER_ROTATION * MAX_RPS);
   parallel.setAcceleration(3 * STEPS_PER_ROTATION * MAX_RPS);
   series.setMaxSpeed(STEPS_PER_ROTATION * MAX_RPS);
   series.setAcceleration(3 * STEPS_PER_ROTATION * MAX_RPS);
   parallel_previous_position = readFromEEPROM(4);
}

// saving current position, powering down motors
void finishTuning() {
  digitalWrite(motorSleep, LOW);
  updateEEPROM(4, parallel.currentPosition());
  updateEEPROM(8, series.currentPosition());
}

void inputOutOfRange() {
    series.moveTo(0);
    series.runToPosition();
    parallel.moveTo(0);
    parallel.runToPosition();
    finishTuning();
    for(int i = 0; i < 50; i++) {
      delay(250);
      analogWrite(LEDPin, 180);
      delay(250);
      analogWrite(LEDPin, 0);
    }
}

void printReading(int val) {
    Serial.println(val);
}

// EEPROM stuff
// Basic EEPROM commands can only work with bytes, but this program uses 2 byte integers
void writeToEEPROM(int address, int number) {
  byte byte0 = number >> 8;
  byte byte1 = number & 0xFF;
      
  EEPROM.write(address, byte0);
  EEPROM.write(address + 1, byte1);
}

void updateEEPROM(int address, int number) {
  byte byte0 = number >> 8;
  byte byte1 = number & 0xFF;
      
  EEPROM.update(address, byte0);
  EEPROM.update(address + 1, byte1);
}

int readFromEEPROM(int address) {
  byte byte0 = EEPROM.read(address);
  byte byte1 = EEPROM.read(address + 1);
  return(byte0 << 8) + byte1;
}

// Determine signs
int int_sgn(int x) {
  if (x > 0) {return 1;}
  if (x < 0) {return -1;}
  return 0; 
}

int float_sgn(float x) {
  if (x > 0) {return 1;}
  if (x < 0) {return -1;}
  return 0; 
}
