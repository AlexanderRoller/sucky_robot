/*
 * AFI-ESC610 test – Arduino UNO version
 * Pin 10 → ESC signal (5 V logic)
 * 50 Hz frame, 1 ms = idle, 2 ms = full
 */

#include <Arduino.h>
#include <Servo.h>

constexpr uint8_t PWM_PIN = 10;   // OC1B on the UNO
constexpr uint16_t PULSE_MIN = 1500;   // µs
constexpr uint16_t PULSE_MAX = 2000;   // µs

Servo esc;

void setup() {
  esc.attach(PWM_PIN);            // 50 Hz by default (Timer-1)

  /* --------- NORMAL ARM --------- */
  esc.writeMicroseconds(PULSE_MIN);   // idle pulse
  delay(3000);                        // wait for ESC “ready” beep
  /* --------------------------------*/

  Serial.begin(9600);
}

void loop() {
  for (uint16_t pulse = PULSE_MIN; pulse <= PULSE_MAX; pulse += 5) {
    esc.writeMicroseconds(pulse);
    Serial.print("Throttle: ");
    Serial.println(pulse);
    delay(75);
  }
  for (uint16_t pulse = PULSE_MAX; pulse >= PULSE_MIN; pulse -= 5) {
    esc.writeMicroseconds(pulse);
    Serial.print("Throttle: ");
    Serial.println(pulse);
    delay(75);
  }
}