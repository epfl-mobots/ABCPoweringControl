//#include "M5Unified.h" Cyril 15.08.2024: Replace with M5StickCPlus.h
#include "M5StickCPlus.h"
//#include "M5GFX.h" Cyril 15.08.2024: Not sure this is needed
#include "M5_ADS1115.h"
#include <Arduino.h>

#define BAUD_RATE 115200

#define M5_UNIT_AMETER_1_I2C_ADDR 0x48
#define M5_UNIT_AMETER_2_I2C_ADDR 0x49 // Changed to 0x49 as per your comment

#define M5_UNIT_AMETER_EEPROM_I2C_ADDR 0x51
#define M5_UNIT_AMETER_PRESSURE_COEFFICIENT 0.05

#define SWITCH_1_PIN 25
#define SWITCH_2_PIN 26

#define SWITCH_ON HIGH
#define SWITCH_OFF LOW

const static bool verbose = true;
const static bool display = true;
static float maximal_current = 800; // maximal current in mA

static ADS1115 Ameter_1;
static float resolution_1 = 0.0;
static float calibration_factor_1 = 0.0;
static float current_1 = 0.0;
static float current_1_prev = current_1;

static ADS1115 Ameter_2;
static float resolution_2 = 0.0;
static float calibration_factor_2 = 0.0;
static float current_2 = 0.0;
static float current_2_prev = current_2;



void clearSerialBuffer() {
    while (Serial.available() > 0) {
        Serial.read();
    }
    Serial.flush();
}

void setup_switches(){
    pinMode(SWITCH_1_PIN, OUTPUT);
    pinMode(SWITCH_2_PIN, OUTPUT);

    // Start with the pins HIGH or ON
    digitalWrite(SWITCH_1_PIN, SWITCH_ON);
    digitalWrite(SWITCH_2_PIN, SWITCH_ON);
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
    if (current_1 == current_1_prev and current_1 == current_2_prev) return;
    current_1_prev = current_1;
    current_2_prev = current_2;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(YELLOW);
    M5.Lcd.printf("Current 1: %.2f mA\n", current_1);
    M5.Lcd.printf("Current 2: %.2f mA\n", current_2);
}

void limit_current(float current_1, float current_2){
  if (current_1 > maximal_current){
    digitalWrite(SWITCH_1_PIN, SWITCH_OFF);
  }
  else if(current_2 > maximal_current){
    digitalWrite(SWITCH_2_PIN, SWITCH_OFF);
  } 
}

void command_handler(String command){
      if (command == "Init"){ // clear the serial buffer  
        clearSerialBuffer();
        Serial.println("Buffer successfully reset");
      }
      else if (command == "Measure current 0")
      {
        Serial.print("Measuring current -> ");
        current_1 = measure_current(&Ameter_1, resolution_1, calibration_factor_1);
        current_2 = measure_current(&Ameter_2, resolution_2, calibration_factor_2);
        Serial.print("I1 = ");
        Serial.print(current_1);
        Serial.print("mA, ");
        Serial.print("I2 = ");
        Serial.print(current_2);
        Serial.println("mA");
        }
      else if (command == "Measure current 1")
      {
        Serial.print("Measuring current 1 -> ");
        current_1 = measure_current(&Ameter_1, resolution_1, calibration_factor_1);
        Serial.print("I1 = ");
        Serial.print(current_1);
        Serial.println("mA");        }
      else if (command == "Measure current 2")
      {
        Serial.print("Measuring current 2 -> ");
        current_2 = measure_current(&Ameter_2, resolution_2, calibration_factor_2);
        Serial.print("I2 = ");
        Serial.print(current_2);
        Serial.println("mA");        
        }
      else if (command == "Switch 1 on"){
      
              digitalWrite(SWITCH_1_PIN, SWITCH_OFF);  // Turn off SWITCH 1
              Serial.println("SWITCH 1 turned on");

      }
      else if (command == "Switch 1 off"){
      
              digitalWrite(SWITCH_1_PIN, SWITCH_OFF);  // Turn off SWITCH 1
              Serial.println("SWITCH 1 turned off");

      }
      else if (command == "Switch 2 on"){
      
              digitalWrite(SWITCH_2_PIN, SWITCH_OFF);  // Turn off SWITCH 1
              Serial.println("SWITCH 2 turned on");

      }
      else if (command == "Switch 2 off"){
      
              digitalWrite(SWITCH_2_PIN, SWITCH_OFF);  // Turn off SWITCH 1
              Serial.println("SWITCH 2 turned off");

      }
      else if (command == "Get switch 1 state"){
        if (digitalRead(SWITCH_1_PIN) == SWITCH_OFF)
             Serial.println("SWITCH 1 turned off");
        else if (digitalRead(SWITCH_1_PIN) == SWITCH_ON)
             Serial.println("SWITCH 1 turned on");
      }
      else if (command == "Get switch 2 state"){
        if (digitalRead(SWITCH_2_PIN) == SWITCH_OFF)
             Serial.println("SWITCH 2 turned off");
        else if (digitalRead(SWITCH_2_PIN) == SWITCH_ON)
             Serial.println("SWITCH 2 turned on");
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
  //Serial.begin(BAUD_RATE); // Initialised in M5.begin()
    M5.begin();
    delay(2000);
    Serial.println("M5StickC started");
    setup_switches();
    setup_ammeters();

   clearSerialBuffer();
}

void loop() {

    for (int i = 0; i < 20; i++){ // wait for 20 * 50 ms hence update value every second not requested
        if (Serial.available() > 0) {
            // Read the incoming command
            String command = Serial.readStringUntil('\n');
            // delay(20);
            command_handler(command);
        }
        delay(49);
    }

    current_1 = measure_current(&Ameter_1, resolution_1, calibration_factor_1);
    current_2 = measure_current(&Ameter_2, resolution_2, calibration_factor_2);
    if (verbose) Serial.printf("Current 1: %.2f mA, Current 2: %.2f mA\n", current_1, current_2);
    if (display) displayLatestCurrents();
    limit_current(current_1, current_2);  
    delay(100);
}