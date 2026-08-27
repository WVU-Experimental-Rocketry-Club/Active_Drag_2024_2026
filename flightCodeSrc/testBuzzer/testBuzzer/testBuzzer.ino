#include "EasyBuzzer.h"

// Define the pin your buzzer's positive (longer) leg or signal wire is connected to
const int BUZZER_PIN = 9; 

void setup() {
  Serial.begin(115200);
  delay(2000); // Give the serial monitor time to open
  Serial.println("Starting Buzzer Test...");

  // 1. Tell the library which pin to control
  EasyBuzzer.setPin(BUZZER_PIN);

  // 2. Configure the double-beep pattern
  // Arguments: 
  // Frequency (Hz), On-Time (ms), Off-Time (ms), Beeps per cycle, Pause between cycles (ms), Number of cycles (0 = forever)
  EasyBuzzer.beep(2000, 100, 100, 2, 2000, 0); 
}

void loop() {
  // 3. This single command handles all the timing without freezing your board
  EasyBuzzer.update(); 
}