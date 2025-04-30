#include "M5StickCPlus.h" // Includes Wire and Arduino libraries
#include "M5_4Relay.h"

#define BAUD_RATE 115200 // For communication with Raspberry Pi
#define PAHUB_ADDR 0x70  // Unit PaHub v2.1 (PCA9548AP) I2C address

const static bool verbose = false;
const static bool display = true;

// Relay mapping: unit 1 (relays 1-4), unit 2 (relays 1-4), unit 3 (relays 1-2)
struct RelayMap {
    uint8_t unit;    // User-facing unit number (1-3)
    uint8_t relay;   // User-facing relay number (1-4)
    uint8_t channel; // PaHub channel (0-2)
};
static const RelayMap relayMapping[10] = {
    {1, 1, 0}, {1, 2, 0}, {1, 3, 0}, {1, 4, 0}, // Unit 1, relays 1-4, channel 0
    {2, 1, 1}, {2, 2, 1}, {2, 3, 1}, {2, 4, 1}, // Unit 2, relays 1-4, channel 1
    {3, 1, 2}, {3, 2, 2}                       // Unit 3, relays 1-2, channel 2
};

// Track states of all relays [unit][relay]
static bool relayStates[3][4] = {
    {false, false, false, false}, // Unit 1: relays 1-4
    {false, false, false, false}, // Unit 2: relays 1-4
    {false, false, false, false}  // Unit 3: relays 1-2 (3-4 unused)
};

#define SWITCH_CLOSED HIGH
#define SWITCH_OPEN  LOW

static M5_4Relay relay; // Single instance reused across channels

void clearSerialBuffer() {
    while (Serial.available() > 0) Serial.read();
    Serial.flush();
}

// Select PaHub I2C channel
void paHubSelect(uint8_t channel) {
    if (channel > 7) {
        if (verbose) Serial.println("Invalid PaHub channel");
        return;
    }
    Wire.beginTransmission(PAHUB_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

// Initialize relays to open state
void init_relays() {
    for (uint8_t i = 0; i < 3; i++) {
        paHubSelect(i); // Select unit's channel
        relay.begin(Wire); // Initialize relay unit
        relay.SyncMode(true); // Sync LEDs with relays
        for (uint8_t j = 0; j < 4; j++) {
            if (i == 2 && j >= 2) continue; // Skip relays 3-4 on unit 3
            relay.Write4Relay(j, SWITCH_OPEN);
            relayStates[i][j] = false;
        }
    }
    if (verbose) Serial.println("Relays initialized to open");
    if (display) displayAllRelayStates();
}

// Display all relay states on LCD
void displayAllRelayStates() {
    if (!display) return;

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println("\n___________\n\n   Relays\n   States\n___________\n");

    for (int i = 0; i < 10; i++) {
        uint8_t unit = relayMapping[i].unit - 1;
        uint8_t relay = relayMapping[i].relay - 1;
        M5.Lcd.setTextSize(1.7);
        M5.Lcd.print("Relay ");
        M5.Lcd.print(i + 1);
        M5.Lcd.print(": ");
        M5.Lcd.println(relayStates[unit][relay] ? "Closed" : "Open");
    }
}

// Set relay state (true for closed, false for open)
void setRelay(uint8_t relayNum, bool state) {
    if (relayNum < 1 || relayNum > 10) {
        if (verbose) Serial.printf("Invalid relay number: %d\n", relayNum);
        return;
    }

    uint8_t index = relayNum - 1;
    uint8_t unit = relayMapping[index].unit - 1;
    uint8_t relayIndex = relayMapping[index].relay - 1;
    uint8_t channel = relayMapping[index].channel;

    relayStates[unit][relayIndex] = state; // Update state tracker

    paHubSelect(channel); // Select PaHub channel
    relay.begin(Wire); // Ensure relay is initialized for this channel
    relay.SyncMode(true); // Ensure sync mode
    relay.Write4Relay(relayIndex, state ? SWITCH_CLOSED : SWITCH_OPEN);

    if (verbose) Serial.printf("Relay %d set to %s\n", relayNum, state ? "Closed" : "Open");
}

// Process single relay command (e.g., "Relay X close")
void processSingleRelayCommand(String command) {
    command.trim();
    int space1 = command.indexOf(' ', 6);
    if (space1 == -1) {
        Serial.println("Invalid relay command");
        return;
    }
    String relayNumStr = command.substring(6, space1);
    String action = command.substring(space1 + 1);
    int relayNum = relayNumStr.toInt();
    if (relayNum < 1 || relayNum > 10) {
        Serial.println("Invalid relay number (use 1-10)");
        return;
    }
    if (action == "close") {
        setRelay(relayNum, true);
        Serial.printf("Relay %d closed\n", relayNum);
        if (display) displayAllRelayStates();
    } else if (action == "open") {
        setRelay(relayNum, false);
        Serial.printf("Relay %d opened\n", relayNum);
        if (display) displayAllRelayStates();

    } else {
        Serial.println("Invalid action (use close or open)");
    }
}

// Get status of all relays
void getRelayStatus() {
    // Optionally sync relayStates with hardware
    for (uint8_t i = 0; i < 3; i++) {
        paHubSelect(i);
        relay.begin(Wire);
        uint16_t state = relay.ReadState();
        for (uint8_t j = 0; j < 4; j++) {
            if (i == 2 && j >= 2) continue; // Skip relays 3-4 on unit 3
            relayStates[i][j] = (state & (1 << j)) ? true : false;
        }
    }

    for (int i = 0; i < 10; i++) {
        uint8_t unit = relayMapping[i].unit - 1;
        uint8_t relay = relayMapping[i].relay - 1;
        Serial.printf("Relay %d: %s\n", i + 1, relayStates[unit][relay] ? "Closed" : "Open");
    }
    if (display) displayAllRelayStates();
}

// Handle serial commands
void command_handler(String command) {
    command.trim();
    if (verbose) {
        Serial.print("Received command: ");
        Serial.println(command);
    }

    if (command == "Init") {
        clearSerialBuffer();
        init_relays();
        Serial.println("Buffer reset, relays initialized to open");
    } else if (command == "CloseAll") {
        for (uint8_t i = 1; i <= 10; i++) {
            setRelay(i, true);
        }
        Serial.println("All relays closed");
        if (display) displayAllRelayStates();
    } else if (command == "OpenAll") {
        for (uint8_t i = 1; i <= 10; i++) {
            setRelay(i, false);
        }
        Serial.println("All relays opened");
        if (display) displayAllRelayStates();
    } else if (command == "Status") {
        getRelayStatus();
    } else if (command.startsWith("Relay ")) {
        int commaPos = command.indexOf(',');
        if (commaPos == -1) {
            processSingleRelayCommand(command);
        } else {
            int startPos = 0;
            while (commaPos != -1) {
                String singleCommand = command.substring(startPos, commaPos);
                singleCommand.trim();
                if (singleCommand.startsWith("Relay ")) {
                    processSingleRelayCommand(singleCommand);
                } else {
                    Serial.printf("Invalid sub-command: %s\n", singleCommand.c_str());
                }
                startPos = commaPos + 1;
                commaPos = command.indexOf(',', startPos);
            }
            String lastCommand = command.substring(startPos);
            lastCommand.trim();
            if (lastCommand.startsWith("Relay ")) {
                processSingleRelayCommand(lastCommand);
            } else {
                Serial.printf("Invalid sub-command: %s\n", lastCommand.c_str());
            }
            if (display) displayAllRelayStates();
        }
    } else {
        Serial.println("Command not recognized");
        Serial.println("Available commands:");
        Serial.println("Init - Reset buffer and open all relays");
        Serial.println("CloseAll - Close all relays");
        Serial.println("OpenAll - Open all relays");
        Serial.println("Relay X open/close - Control single relay (X=1-10)");
        Serial.println("Relay X open, Relay Y close,... - Multiple relay commands");
        Serial.println("Status - Show current relay states");
    }
}

void setup() {
    M5.begin(); // Initializes Serial and internal I2C (Wire1)
    delay(1000);
    Serial.begin(BAUD_RATE);
    Serial.println("M5StickC Relay Controller Started");

    Wire.begin(32, 33, 400000UL); // External I2C: SDA, SCL, frequency
    delay(50);
    Serial.println("External I2C started");

    // Initialize display
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.print("STARTING");
    delay(2000);
    // Initialize relays
    init_relays();
    clearSerialBuffer();
}

void loop() {
    static String inputString = "";
    static bool stringComplete = false;

    // Handle serial input
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        inputString += inChar;
        if (inChar == '\n') stringComplete = true;
    }

    // Process complete command
    if (stringComplete) {
        command_handler(inputString);
        inputString = "";
        stringComplete = false;
    }

    delay(10); // Small delay to prevent tight looping
}