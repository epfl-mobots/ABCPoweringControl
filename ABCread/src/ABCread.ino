// Version Aug 27, working on all 3 boards, created by Raphael Kohler

#include "M5StickCPlus.h"
#include "M5_ADS1115.h"
#include <Arduino.h>

#define BAUD_RATE 115200

#define M5_UNIT_AMETER_1_I2C_ADDR 0x48
#define M5_UNIT_AMETER_2_I2C_ADDR 0x49

#define M5_UNIT_AMETER_EEPROM_I2C_ADDR 0x51
#define M5_UNIT_AMETER_PRESSURE_COEFFICIENT 0.05

#define SWITCH_1_PIN 25
#define SWITCH_2_PIN 26

#define SWITCH_CLOSED HIGH
#define SWITCH_OPEN LOW

const static bool verbose = false;
const static bool display = true;
static float maximal_current = 2000; // maximal current in mA

static ADS1115 Ameter_1;
static float resolution_1 = 0.0;
static float calibration_factor_1 = 0.0;
static float current_1 = 0.0;

static ADS1115 Ameter_2;
static float resolution_2 = 0.0;
static float calibration_factor_2 = 0.0;
static float current_2 = 0.0;

const long interval = 500;  // Interval to compare current (in milliseconds)


// Getter functions
static float get_current1() {
    return current_1;
}

static float get_current2() {
    return current_2;
}

void clearSerialBuffer() {
    while (Serial.available() > 0) {
        Serial.read();
    }
    Serial.flush();
}

void setup_switches(){
    pinMode(SWITCH_1_PIN, OUTPUT);
    pinMode(SWITCH_2_PIN, OUTPUT);

    // Start with the pins ON (CLOSED)
    digitalWrite(SWITCH_1_PIN, SWITCH_CLOSED);
    digitalWrite(SWITCH_2_PIN, SWITCH_CLOSED);
}

void setup_ammeters() {
    while (!Ameter_1.begin(&Wire, M5_UNIT_AMETER_1_I2C_ADDR, 32, 33, 400000UL)) {
        Serial.println("Unit Ameter 1 Init Fail");
        delay(1000);
    }
    Ameter_1.setEEPROMAddr(M5_UNIT_AMETER_EEPROM_I2C_ADDR);
    Ameter_1.setMode(ADS1115_MODE_SINGLESHOT);
    Ameter_1.setRate(ADS1115_RATE_8);
    Ameter_1.setGain(ADS1115_PGA_256);

    resolution_1 = Ameter_1.getCoefficient() / M5_UNIT_AMETER_PRESSURE_COEFFICIENT;
    calibration_factor_1 = Ameter_1.getFactoryCalibration();
    if (verbose) {
        Serial.print("Ammeter 1 successfully initialized with resolution: ");
        Serial.print(resolution_1);
        Serial.print(" and calibration factor: ");
        Serial.println(calibration_factor_1);
    }

        while (!Ameter_2.begin(&Wire, M5_UNIT_AMETER_2_I2C_ADDR, 32, 33, 400000UL)) {
        Serial.println("Unit Ameter 2 Init Fail");
        delay(1000);
    }
    Ameter_2.setEEPROMAddr(M5_UNIT_AMETER_EEPROM_I2C_ADDR);
    Ameter_2.setMode(ADS1115_MODE_SINGLESHOT);
    Ameter_2.setRate(ADS1115_RATE_8);
    Ameter_2.setGain(ADS1115_PGA_256);

    resolution_2 = Ameter_2.getCoefficient() / M5_UNIT_AMETER_PRESSURE_COEFFICIENT;
    calibration_factor_2 = Ameter_2.getFactoryCalibration();
    if (verbose) {
        Serial.print("Ammeter 2 successfully initialized with resolution: ");
        Serial.print(resolution_2);
        Serial.print(" and calibration factor: ");
        Serial.println(calibration_factor_2);
    }
}


float measure_current(ADS1115* Ameter, float resolution, float calibration_factor) {
    int16_t adc_raw = Ameter->getSingleConversion();
    float current = adc_raw * resolution * calibration_factor;

    if (verbose) {
        Serial.printf("Cal ADC:%.0f\n", adc_raw * calibration_factor);
        Serial.printf("Cal Current:%.2f mA\n", current);
        Serial.printf("Raw ADC:%d\n\n", adc_raw);
    }
    return current;
}

void displayLatestCurrents(){
    static float prev_c1,prev_c2 = -1.0;
    if(prev_c1!=current_1 || prev_c2!=current_2){
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(10, 30);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(YELLOW);
        M5.Lcd.printf("Current 1: %.2f mA\n", current_1);
        M5.Lcd.setCursor(10, 100);
        M5.Lcd.printf("Current 2: %.2f mA\n", current_2);
        prev_c1=current_1;
        prev_c2=current_2;
    }
}

void ensure_safe_currents(){
  if (abs(current_1) > maximal_current){
    digitalWrite(SWITCH_1_PIN, SWITCH_OPEN);
  }
  else if(abs(current_2) > maximal_current){
    digitalWrite(SWITCH_2_PIN, SWITCH_OPEN);
  } 
}

void command_handler(String command){
      command.trim();
      // Serial.print("Command was: ");
      // Serial.println(command);
      if (command == "Init"){ // clear the serial buffer  
        clearSerialBuffer();
        digitalWrite(SWITCH_1_PIN, SWITCH_CLOSED);  // Turn on SWITCH 1
        digitalWrite(SWITCH_2_PIN, SWITCH_CLOSED);  // Turn on SWITCH 2
        Serial.println("Buffer successfully reset, switches turned on");
      }
      else if (command == "Measure current 0")
      {
        Serial.print("Measuring current -> ");
        current_1 = measure_current(&Ameter_1, resolution_1, calibration_factor_1);
        current_2 = measure_current(&Ameter_2, resolution_2, calibration_factor_2);
        Serial.print("I1 = ");
        Serial.print(get_current1());
        Serial.print("mA, ");
        Serial.print("I2 = ");
        Serial.print(get_current2());
        Serial.println("mA");
        }
      else if (command == "Measure current 1")
      {
        Serial.print("Measuring current 1 -> ");
        current_1 = measure_current(&Ameter_1, resolution_1, calibration_factor_1);
        Serial.print("I1 = ");
        Serial.print(get_current1());
        Serial.println("mA");        }
      else if (command == "Measure current 2")
      {
        Serial.print("Measuring current 2 -> ");
        current_2 = measure_current(&Ameter_2, resolution_2, calibration_factor_2);
        Serial.print("I2 = ");
        Serial.print(get_current2());
        Serial.println("mA");        
        }
      else if (command == "Switch 1 on"){
      
              digitalWrite(SWITCH_1_PIN, SWITCH_CLOSED);  // Turn on SWITCH 1
              Serial.println("SWITCH 1 closed (on)");

      }
      else if (command == "Switch 1 off"){
      
              digitalWrite(SWITCH_1_PIN, SWITCH_OPEN);  // Turn off SWITCH 1
              Serial.println("SWITCH 1 opened (off)");

      }
      else if (command == "Switch 2 on"){
      
              digitalWrite(SWITCH_2_PIN, SWITCH_CLOSED);  // Turn on SWITCH 1
              Serial.println("SWITCH 2 closed (on)");

      }
      else if (command == "Switch 2 off"){
      
              digitalWrite(SWITCH_2_PIN, SWITCH_OPEN);  // Turn of SWITCH 1
              Serial.println("SWITCH 2 opened (off)");

      }
      else if (command == "Get switch 1 state"){
        if (digitalRead(SWITCH_1_PIN) == SWITCH_OPEN)
            Serial.println("SWITCH 1 open (off)");
        else if (digitalRead(SWITCH_1_PIN) == SWITCH_CLOSED)
            Serial.println("SWITCH 1 closed (on)");
      }
      else if (command == "Get switch 2 state"){
        if (digitalRead(SWITCH_2_PIN) == SWITCH_OPEN)
            Serial.println("SWITCH 2 open (off)");
        else if (digitalRead(SWITCH_2_PIN) == SWITCH_CLOSED)
            Serial.println("SWITCH 2 closed (on)");
      }
      else if (command.startsWith("Max current ")){
        String maxCurrentStr = command.substring(12); // 12 is length of "Max current "
        
        float maxCurrentValue = maxCurrentStr.toFloat(); // Convert to float        
        maximal_current = maxCurrentValue;
        Serial.print("Maximal current set to: ");
        Serial.println(maximal_current);
      }
      else {
        Serial.println("Command not recognized");
      }
}

void setup() {
    M5.begin();
    delay(2000);
    Serial.println("M5StickC started");
    setup_switches();
    setup_ammeters();

    clearSerialBuffer();
}


void loop() {
    static unsigned long previousMillis = 0;
    static String inputString = "";  
    static bool stringComplete = false;
    // Handle serial input
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        inputString += inChar;
        
        if (inChar == '\n') {
            stringComplete = true;
        }
    }
    
    // Process the command if complete
    if (stringComplete) {
        command_handler(inputString);
        inputString = "";         
        stringComplete = false;  
    }


    unsigned long currentMillis = millis();    

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;  // Update the previous time

        current_1 = measure_current(&Ameter_1, resolution_1, calibration_factor_1);
        current_2 = measure_current(&Ameter_2, resolution_2, calibration_factor_2);
        if (verbose) Serial.printf("Current 1: %.2f mA, Current 2: %.2f mA\n", current_1, current_2);
        if (display) displayLatestCurrents();
        ensure_safe_currents();  // Open switches if either current is too high
        delay(10);
    }
    delay(10);
}