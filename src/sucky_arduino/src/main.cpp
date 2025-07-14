
#include <Arduino.h>
#include <Servo.h>

//function prototypes
void processCommand();
void setCycloneState(bool state);
void setDoorState(bool open);
void printStatus();
void printHelp();
void serialEvent();

constexpr uint8_t PWM_PIN = 10;        // ESC PWM pin (OC1B on UNO)
constexpr uint8_t DOOR_LEFT_PIN = 5;   // Left door servo pin
constexpr uint8_t DOOR_RIGHT_PIN = 3;  // Right door servo pin

// ESC pulse configuration
constexpr uint16_t PULSE_MIN = 1500;   // µs (idle)
constexpr uint16_t PULSE_MAX = 2000;   // µs (full power)

// Door servo positions
constexpr uint8_t DOOR_LEFT_CLOSED = 180;    // Left door closed position
constexpr uint8_t DOOR_LEFT_OPEN = 0;    // Left door open position
constexpr uint8_t DOOR_RIGHT_CLOSED = 0; // Right door closed position  
constexpr uint8_t DOOR_RIGHT_OPEN = 180;     // Right door open position

// Global objects and state
Servo doorLeft;
Servo doorRight;
Servo esc;

bool cycloneOn = false;
bool doorsOpen = false;
String inputString = "";
bool stringComplete = false;

// Safety timeout (10 minutes max cyclone runtime)
constexpr unsigned long CYCLONE_TIMEOUT_MS = 600000;
unsigned long cycloneStartTime = 0;

void setup() {
  Serial.begin(9600);

  doorLeft.attach(5);  // Attach left door servo to pin 5
  doorRight.attach(3); // Attach right door servo to pin 3
  
  doorLeft.write(180);  // Set left door to neutral position 180-closed
  doorRight.write(0); // Set right door to neutral position 0-closed
  doorsOpen = false;  // Initialize doors as closed

  esc.attach(PWM_PIN);            // 50 Hz by default (Timer-1)

  /* --------- NORMAL ARM --------- */
  esc.writeMicroseconds(PULSE_MIN);   // idle pulse
  delay(3000);                        // wait for ESC “ready” beep
  /* --------------------------------*/

}

void loop() {
  // Safety check: Auto-shutdown cyclone after timeout
  if (cycloneOn && (millis() - cycloneStartTime > CYCLONE_TIMEOUT_MS)) {
    cycloneOn = false;
    esc.writeMicroseconds(PULSE_MIN);
    Serial.println("SAFETY: Cyclone auto-shutdown after timeout");
  }
  
  // Handle serial input
  if (stringComplete) {
    processCommand();
  }
  
  // Small delay to prevent overwhelming the system
  delay(10);
}

void processCommand() {
  inputString.trim();
  inputString.toUpperCase();
  
  if (inputString == "ON") {
    setCycloneState(true);
  }
  else if (inputString == "OFF") {
    setCycloneState(false);
  }
  else if (inputString == "DOOR_OPEN") {
    setDoorState(true);
  }
  else if (inputString == "DOOR_CLOSE") {
    setDoorState(false);
  }
  else if (inputString == "STATUS") {
    printStatus();
  }
  else if (inputString == "HELP") {
    printHelp();
  }
  else {
    Serial.print("ERROR: Unknown command '");
    Serial.print(inputString);
    Serial.println("'. Send HELP for available commands.");
  }
  
  // Clear the string for next command
  inputString = "";
  stringComplete = false;
}

void setCycloneState(bool state) {
  if (state && !cycloneOn) {
    cycloneOn = true;
    cycloneStartTime = millis();
    esc.writeMicroseconds(PULSE_MAX);
    Serial.println("Cyclone ON");
  }
  else if (!state && cycloneOn) {
    cycloneOn = false;
    esc.writeMicroseconds(PULSE_MIN);
    Serial.println("Cyclone OFF");
  }
  else {
    Serial.print("Cyclone already ");
    Serial.println(cycloneOn ? "ON" : "OFF");
  }
}

void setDoorState(bool open) {
  if (open && !doorsOpen) {
    doorsOpen = true;
    doorLeft.write(DOOR_LEFT_OPEN);
    doorRight.write(DOOR_RIGHT_OPEN);
    Serial.println("Doors OPEN");
  }
  else if (!open && doorsOpen) {
    doorsOpen = false;
    doorLeft.write(DOOR_LEFT_CLOSED);
    doorRight.write(DOOR_RIGHT_CLOSED);
    Serial.println("Doors CLOSED");
  }
  else {
    Serial.print("Doors already ");
    Serial.println(doorsOpen ? "OPEN" : "CLOSED");
  }
}

void printStatus() {
  Serial.println("=== SYSTEM STATUS ===");
  Serial.print("Cyclone: ");
  Serial.println(cycloneOn ? "ON" : "OFF");
  Serial.print("Doors: ");
  Serial.println(doorsOpen ? "OPEN" : "CLOSED");
  Serial.print("ESC Pulse: ");
  Serial.print(cycloneOn ? PULSE_MAX : PULSE_MIN);
  Serial.println(" µs");
  if (cycloneOn) {
    unsigned long runtime = (millis() - cycloneStartTime) / 1000;
    Serial.print("Cyclone Runtime: ");
    Serial.print(runtime);
    Serial.println(" seconds");
  }
  Serial.println("====================");
}

void printHelp() {
  Serial.println("=== AVAILABLE COMMANDS ===");
  Serial.println("ON         - Turn cyclone ON");
  Serial.println("OFF        - Turn cyclone OFF");
  Serial.println("DOOR_OPEN  - Open doors");
  Serial.println("DOOR_CLOSE - Close doors");
  Serial.println("STATUS     - Show system status");
  Serial.println("HELP       - Show this help");
  Serial.println("===========================");
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX.
  This routine is run between each time loop() runs, so using delay inside
  loop can delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // Get the new byte
    char inChar = (char)Serial.read();
    
    // If the incoming character is a newline, set a flag so the main loop can
    // do something about it
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      // Add it to the inputString
      inputString += inChar;
    }
  }
}