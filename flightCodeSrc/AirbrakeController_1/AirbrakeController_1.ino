#include <ODriveUART.h>

HardwareSerial& odrive_serial = Serial1;
unsigned long baudrate = 115200;

ODriveUART odrive(odrive_serial);

// ================================
// Homing Configuration
// ================================
const float HOMING_TORQUE      = 0.125f;   // Nm - gentle torque for homing
const float HOMING_VEL_LIMIT   = 2.0f;    // turns/sec - slow homing speed
const float HOMING_STALL_VEL   = 0.02f;   // turns/sec - velocity threshold for stall detection
const uint32_t HOMING_STALL_MS = 200;     // ms - time at low velocity to confirm stall
const float HOMING_BACKOFF     = 0.1f;    // turns - back off from hard stop

// ================================
// Trap Trajectory Configuration
// ================================
const float VEL_LIMIT   = 35.0f;   // turns/sec   - gentle speed
const float ACCEL_LIMIT = 250.0f;  // turns/sec²  - gentle acceleration
const float DECEL_LIMIT = 100.0f;  // turns/sec²  - gentle deceleration

const float POS_A       = 0.0f;    // home position (turns)
const float POS_B       = -15.0f;   // extended position (turns)
const float DWELL_MS    = 5000;    // ms to pause at each end

// ================================
// State
// ================================
bool     at_B            = false;
bool     moving          = false;
uint32_t dwell_start     = 0;
bool     dwelling        = false;
uint32_t lastWatchdog    = 0;
uint32_t lastTelemetry   = 0;
float    homing_offset   = 0.0f;  // Store the homing offset

// ================================
// Torque-Based Homing Function
// ================================
bool performHoming() {
  Serial.println("=== Starting Torque-Based Homing ===");
  
  // ---- Set velocity limit for homing ----
  odrive.setParameter("axis0.controller.config.vel_limit", HOMING_VEL_LIMIT);
  
  // ---- Switch to torque control mode ----
  odrive.setParameter("axis0.controller.config.control_mode", 1); // CONTROL_MODE_TORQUE
  odrive.setParameter("axis0.controller.config.input_mode", 1);   // INPUT_MODE_PASSTHROUGH
  
  // ---- Enter closed loop ----
  Serial.println("Entering closed loop for homing...");
  while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }
  
  // ---- Apply gentle torque to find hard stop ----
  Serial.println("Applying homing torque...");
  odrive.setTorque(HOMING_TORQUE);
  
  uint32_t stall_start = 0;
  bool stalled = false;
  
  // ---- Monitor velocity to detect stall ----
  while (!stalled) {
    ODriveFeedback fb = odrive.getFeedback();
    
    // Feed watchdog
    odrive_serial.println("u 0 1");
    
    Serial.print("Homing - vel: ");
    Serial.print(fb.vel, 4);
    Serial.print(" pos: ");
    Serial.println(fb.pos, 3);
    
    // Check if velocity is below stall threshold
    if (fabsf(fb.vel) < HOMING_STALL_VEL) {
      if (stall_start == 0) {
        stall_start = millis();
      } else if (millis() - stall_start >= HOMING_STALL_MS) {
        stalled = true;
        Serial.println("Hard stop detected!");
      }
    } else {
      stall_start = 0; // Reset stall timer if velocity increases
    }
    
    delay(50);
  }
  
  // ---- Stop applying torque ----
  odrive.setTorque(0.0f);
  delay(100);
  
  // ---- Switch to position control for backoff ----
  odrive.setParameter("axis0.controller.config.control_mode", 3); // CONTROL_MODE_POSITION
  odrive.setParameter("axis0.controller.config.input_mode", 1);   // INPUT_MODE_PASSTHROUGH
  
  // ---- Back off from hard stop ----
  Serial.println("Backing off from hard stop...");
  ODriveFeedback fb = odrive.getFeedback();
  float backoff_target = fb.pos - HOMING_BACKOFF;
  odrive.setPosition(backoff_target);
  
  delay(500); // Wait for backoff to complete
  
  // ---- Store this position as the homing offset ----
  Serial.println("Establishing zero reference...");
  ODriveFeedback fb_home = odrive.getFeedback();
  homing_offset = fb_home.pos;  // Store the offset to subtract from all future positions
  
  Serial.print("Homing offset set to: ");
  Serial.println(homing_offset, 3);
  
  // Set controller to hold at what we'll call "zero"
  odrive.setPosition(fb_home.pos);
  delay(100);
  
  Serial.println("=== Homing Complete ===");
  return true;
}

void setup() {
  Serial.begin(115200);
  odrive_serial.begin(baudrate);
  delay(10);

  // ---- Wait for ODrive ----
  Serial.println("Waiting for ODrive...");
  while (odrive.getState() == AXIS_STATE_UNDEFINED) {
    delay(100);
  }
  Serial.println("ODrive found");

  // ---- Perform homing sequence ----
  if (!performHoming()) {
    Serial.println("HOMING FAILED!");
    while(1) { delay(1000); } // Halt on homing failure
  }

  // Feed watchdog after homing
  odrive_serial.println("u 0 1");
  delay(50);

  // ---- Configure trap trajectory limits ----
  odrive.setParameter("axis0.controller.config.vel_limit", VEL_LIMIT);
  odrive_serial.println("u 0 1");
  delay(50);
  
  odrive.setParameter("axis0.trap_traj.config.vel_limit", VEL_LIMIT);
  odrive_serial.println("u 0 1");
  delay(50);
  
  odrive.setParameter("axis0.trap_traj.config.accel_limit", ACCEL_LIMIT);
  odrive_serial.println("u 0 1");
  delay(50);
  
  odrive.setParameter("axis0.trap_traj.config.decel_limit", DECEL_LIMIT);
  odrive_serial.println("u 0 1");
  delay(50);

  // ---- Use position control with trap trajectory input mode ----
  odrive.setParameter("axis0.controller.config.control_mode", 3); // CONTROL_MODE_POSITION
  odrive_serial.println("u 0 1");
  delay(50);
  
  odrive.setParameter("axis0.controller.config.input_mode", 5);   // INPUT_MODE_TRAP_TRAJ
  odrive_serial.println("u 0 1");
  delay(50);

  // ---- Re-enter closed loop after mode change ----
  Serial.println("Re-entering closed loop control...");
  while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrive_serial.println("u 0 1");
    delay(10);
  }
  Serial.println("Closed loop control restored");

  Serial.println("Running — trapezoid motion 0 ↔ 15 turns");

  // ---- Initialize timers ----
  lastWatchdog = millis();
  lastTelemetry = millis();

  // ---- Send first move to POS_A (home position with offset applied) ----
  odrive.setPosition(POS_A + homing_offset);
  odrive_serial.println("u 0 1");
  moving = true;
}

void loop() {
  uint32_t now = millis();

  // ---- Feed watchdog via feedback poll ----
  if (now - lastWatchdog >= 50) {
    ODriveFeedback fb = odrive.getFeedback();
    odrive_serial.println("u 0 1");
    lastWatchdog = now;

    // Apply homing offset to display position relative to home
    float display_pos = fb.pos - homing_offset;

    // ---- Check if motion has settled ----
    if (moving && fabsf(fb.vel) < 0.05f) {
      moving       = false;
      dwelling     = true;
      dwell_start  = now;
    }

    // ---- Telemetry ----
    Serial.print("pos:");    Serial.print(display_pos, 3);
    Serial.print("  vel:");  Serial.print(fb.vel, 3);
    Serial.print("  target:"); Serial.print(at_B ? POS_B : POS_A);
    Serial.println();
  }

  // ---- Dwell at endpoint then trigger next move ----
  if (dwelling && (now - dwell_start >= DWELL_MS)) {
    dwelling = false;
    at_B     = !at_B;
    float target = (at_B ? POS_B : POS_A) + homing_offset;  // Apply offset to commanded position
    Serial.print("Moving to: ");
    Serial.println(at_B ? POS_B : POS_A);  // Display without offset
    odrive.setPosition(target);
    moving = true;
  }
}