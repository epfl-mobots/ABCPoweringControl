/*Version Aug 27, working on all 3 boards, created by Raphael Kohler
This code reads the current from two ammeters and switches them off if the current is too high.
Can receive commands from a host to send the lastest current values, switch the switches on and off, and set the maximal current.
*/

#include <Arduino.h>
#include "M5Unified.h"
#include "M5_ADS1115.h"
#include "M5_4Relay.h"

#define BAUD_RATE 115200 // To communicate with the RPi (host)

#define M5_UNIT_MULTIPLEXER_I2C_ADDR 0x70
#define M5_UNIT_AMETER_I2C_ADDR 0x48

//I2C Mux channels
#define AMPMETER_1_CH   3
#define AMPMETER_2_CH   5
#define RELAY_CH        4

//Relay channels. The -1 is to convert the channel number to the index in the relay array
#define ROBOT1_RELAY_CH 4 -1
#define ROBOT2_RELAY_CH 3 -1
#define LEDS_RELAY_CH   1 -1 

#define M5_UNIT_AMETER_EEPROM_I2C_ADDR 0x51
#define M5_UNIT_AMETER_PRESSURE_COEFFICIENT 0.05

#define SWITCH_CLOSED   HIGH
#define SWITCH_OPEN     LOW

const static bool verbose = true;
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

static M5_4Relay relay; // The 4-Relay unit

const long interval = 750;  // Interval to check new current value (in milliseconds)

// Function to select the I2C Multiplexer channel
// input parameter: i: the channel number to select, between 0 and 5. The channels are as defined by the #define statements above
void tcaselect(uint8_t i) {
    if (i != AMPMETER_1_CH && i != AMPMETER_2_CH && i != RELAY_CH) {
        if (verbose) Serial.println("Invalid channel number");
        return;
    }
    
    Wire.beginTransmission(M5_UNIT_MULTIPLEXER_I2C_ADDR); 
    Wire.write(1 << i); // select channel i 
    Wire.endTransmission();  
}

// Function to scan all I2C addresses and print the addresses of the devices found or an error message
// helps for debug
void scanI2CAddresses() {
    byte error, address;
    int nDevices;
    
    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println(" !");
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }

    if (nDevices == 0) {
        Serial.println("No I2C devices found\n");
    } else {
        Serial.println("Done\n");
    }
}

void clearSerialBuffer() {
    while (Serial.available() > 0) {
        Serial.read();
    }
    Serial.flush();
}

/*
This function initialises the switch by ensuring that both switched of the robot are closed (ON).
*/
void init_switches(){
    tcaselect(RELAY_CH); 
    relay.begin(Wire); // Initialize the relay
    relay.SyncMode(true); // Sync the LEDs with the relays
    relay.AllOff(); // Turn off all relays
    // Close switches ROBOT1_RELAY_CH and ROBOT2_RELAY_CH only
    relay.Write4Relay(ROBOT1_RELAY_CH, SWITCH_CLOSED);
    relay.Write4Relay(ROBOT2_RELAY_CH, SWITCH_CLOSED);
}

/*
This function initialises the ammeters by setting the resolution and calibration factors.
*/
void init_ammeters() {
    tcaselect(AMPMETER_1_CH); // Select the Ammeter 1 I2C channel
    delay(1000);
    while (!Ameter_1.begin(&Wire, M5_UNIT_AMETER_I2C_ADDR, 32, 33, 400000UL)) {
        if (verbose) Serial.println("Unit Ameter 1 Init Fail");
        delay(2000);
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

    tcaselect(AMPMETER_2_CH); // Select the Ammeter 2 I2C channel
    while (!Ameter_2.begin(&Wire, M5_UNIT_AMETER_I2C_ADDR, 32, 33, 400000UL)) {
        if (verbose) Serial.println("Unit Ameter 2 Init Fail");
        delay(2000);
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


float measure_current(uint8_t I2C_ch, ADS1115* Ameter, float resolution, float calibration_factor) {
    tcaselect(I2C_ch);
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
    tcaselect(RELAY_CH);
    // Tell the relay to open channel ROBOT1_RELAY_CH
    relay.Write4Relay(ROBOT1_RELAY_CH, SWITCH_OPEN);
  }
  else if(abs(current_2) > maximal_current){
    tcaselect(RELAY_CH);
    // Tell the relay to open channel ROBOT2_RELAY_CH
    relay.Write4Relay(ROBOT2_RELAY_CH, SWITCH_OPEN);
  } 
}

void command_handler(String command){ //TODO: change from 2-Relay to 4-Relay
    command.trim();
    if (verbose){
        Serial.print("Received command: ");
        Serial.println(command);
    }

    if (command == "Init"){ // clear the serial buffer  
        clearSerialBuffer();
        init_switches();
        Serial.println("Buffer successfully reset, switches turned on");
    }
    else if (command == "Measure current 0"){
        Serial.print("Measuring current -> ");
        current_1 = measure_current(AMPMETER_1_CH, &Ameter_1, resolution_1, calibration_factor_1);
        current_2 = measure_current(AMPMETER_2_CH, &Ameter_2, resolution_2, calibration_factor_2);
        Serial.print("I1 = ");
        Serial.print(current_1);
        Serial.print("mA, ");
        Serial.print("I2 = ");
        Serial.print(current_2);
        Serial.println("mA");
    }
    else if (command == "Measure current 1"){
        Serial.print("Measuring current 1 -> ");
        current_1 = measure_current(AMPMETER_1_CH, &Ameter_1, resolution_1, calibration_factor_1);
        Serial.print("I1 = ");
        Serial.print(current_1);
        Serial.println("mA");
    }
    else if (command == "Measure current 2"){
        Serial.print("Measuring current 2 -> ");
        current_2 = measure_current(AMPMETER_2_CH, &Ameter_2, resolution_2, calibration_factor_2);
        Serial.print("I2 = ");
        Serial.print(current_2);
        Serial.println("mA");        
    }
    else if (command.startsWith("Switch ")){
        String switchNumberStr = command.substring(7); // 7 is length of "Switch "
        int switchNumber = switchNumberStr.toInt(); // Convert to int
        if (switchNumber < 1 || switchNumber > 4){
            Serial.println("Invalid switch number");
            return;
        }
        if (command.endsWith("on")){
            // Turn on the switch
            tcaselect(RELAY_CH);
            relay.Write4Relay(switchNumber - 1, SWITCH_CLOSED);
            Serial.print("SWITCH ");
            Serial.print(switchNumber);
            Serial.println(" closed (on)");
        }
        else if (command.endsWith("off")){
            // Turn off the switch
            tcaselect(RELAY_CH);
            relay.Write4Relay(switchNumber - 1, SWITCH_OPEN);
            Serial.print("SWITCH ");
            Serial.print(switchNumber);
            Serial.println(" opened (off)");
        }
        else Serial.println("Command not recognized");
    }
    else if (command.startsWith("Get switch ") && command.endsWith(" state")){
        String switchNumberStr = command.substring(11); // 11 is the start index of the switch number
        int switchNumber = switchNumberStr.toInt(); // Convert to int
        if (switchNumber < 1 || switchNumber > 4){
            Serial.println("Invalid switch number");
            return;
        }
        tcaselect(RELAY_CH);
        int switchState = relay.ReadState() & (1 << (switchNumber - 1));
        if (switchState == SWITCH_OPEN)
            Serial.printf("SWITCH %d open (off)\n", switchNumber);
        else if (switchState == SWITCH_CLOSED)
            Serial.printf("SWITCH %d closed (on)\n", switchNumber);
    }
    else if (command.startsWith("Max current ")){
        String maxCurrentStr = command.substring(12, command.length()); // 12 is length of "Max current "
        float maxCurrentValue = maxCurrentStr.toFloat(); // Convert to float        
        maximal_current = maxCurrentValue;
        Serial.print("Maximal current set to: ");
        Serial.println(maximal_current);
    }
    else Serial.println("Command not recognized");
}

void setup() {
    Serial.begin(BAUD_RATE);
    delay(500);
    Wire.begin(32,33,400000UL); // SDA, SCL, frequency
    delay(50);

    M5.begin();
    delay(2000);
    Serial.println("M5StickC started");

    init_ammeters();
    init_switches();

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

        current_1 = measure_current(AMPMETER_1_CH, &Ameter_1, resolution_1, calibration_factor_1);
        current_2 = measure_current(AMPMETER_2_CH, &Ameter_2, resolution_2, calibration_factor_2);
        if (verbose) Serial.printf("Current 1: %.2f mA, Current 2: %.2f mA\n", current_1, current_2);
        if (display) displayLatestCurrents();
        ensure_safe_currents();  // Open switches if either current is too high
        delay(10);
    }
    delay(20);
}