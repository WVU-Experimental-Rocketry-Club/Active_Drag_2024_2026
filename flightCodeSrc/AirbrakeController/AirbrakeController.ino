// Airbrake Controller
// SparkFun RP2350 Pro Micro  +  ODrive S1  (D6374, sensorless)
//
// Required library (Arduino Library Manager):
//   "ODriveArduino" by ODrive Robotics
//
// Wiring:
//   ODrive S1 GPIO1 (TX) → RP2350 GPIO1 (Serial1 RX)
//   ODrive S1 GPIO2 (RX) → RP2350 GPIO0 (Serial1 TX)
//   ODrive GND            → RP2350 GND
//
// ODrive one-time setup (odrivetool → odrv0.save_configuration()):
//   odrv0.axis0.config.motor.motor_type                  = MotorType.HIGH_CURRENT
//   odrv0.axis0.config.motor.pole_pairs                  = 7      # D6374 = 14 poles
//   odrv0.axis0.config.motor.resistance_calib_max_voltage = 4
//   odrv0.axis0.config.motor.requested_current_range     = 25
//   odrv0.axis0.config.motor.current_control_bandwidth   = 100
//   odrv0.axis0.config.sensorless_estimator.pm_flux_linkage = 5.51328895e-3
//   odrv0.axis0.config.controller.control_mode           = ControlMode.POSITION_CONTROL
//   odrv0.axis0.config.controller.input_mode             = InputMode.TRAP_TRAJ
//   odrv0.axis0.config.trap_traj.vel_limit               = 10     # turns/s
//   odrv0.axis0.config.trap_traj.accel_limit             = 5      # turns/s²
//   odrv0.axis0.config.trap_traj.decel_limit             = 5
//   odrv0.axis0.config.controller.vel_limit              = 12
//   odrv0.config.uart0_baudrate                          = 115200
//
// Homing:
//   Drives the lead screw in the retract direction at a low current limit.
//   When |Iq_measured| exceeds HOMING_STALL_THRESHOLD for
//   HOMING_STALL_CONFIRM_MS continuously, a stall is confirmed at the end-stop.
//   The ODrive position at that instant is saved as home_offset; all subsequent
//   setpoints are sent as (home_offset + desired_turns).

#include <ODriveArduino.h>

/* ── Configuration ─────────────────────────────────────────────────────────── */

// Serial1 on the RP2350 Pro Micro uses GPIO0 (TX) and GPIO1 (RX) by default.
// Baud must match odrv0.config.uart0_baudrate set in odrivetool.
#define ODRIVE_SERIAL   Serial1
#define ODRIVE_BAUD     115200

// Motor turns from fully retracted (home) to fully deployed.
// Measure this on your physical mechanism.
#define DEPLOY_TURNS    5.0f

// Homing
#define HOMING_VELOCITY         -2.0f   // turns/s — negative = retract direction
#define HOMING_CURRENT_LIMIT     3.0f   // A — current cap while seeking end-stop
#define HOMING_STALL_THRESHOLD   2.5f   // A — Iq level that indicates a stall
#define HOMING_STALL_CONFIRM_MS  150    // ms Iq must stay high to confirm the stall
#define HOMING_TIMEOUT_MS       10000   // ms before homing is declared failed

// Normal operation
#define RUN_CURRENT_LIMIT   15.0f   // A
#define RUN_VELOCITY_LIMIT  12.0f   // turns/s

// Move-complete detection
#define POSITION_TOLERANCE  0.05f   // turns
#define VELOCITY_AT_REST    0.10f   // turns/s

/* ── ODrive instance ────────────────────────────────────────────────────────── */

ODriveUART odrive(ODRIVE_SERIAL);

/* ── Airbrake state ─────────────────────────────────────────────────────────── */

enum class State {
    Homing,         // driving toward the retracted end-stop
    HomingConfirm,  // stall candidate — waiting to confirm
    Ready,          // homed and stationary
    Moving,         // travelling to a commanded position
    Error           // fault — send "home" over serial to retry
};

State state       = State::Homing;  // initialised properly in setup()
bool  homed       = false;
float home_offset = 0.0f;   // raw ODrive turns recorded at the end-stop
float target_pos  = 0.0f;   // commanded position, home-relative [turns]

uint32_t homing_start_ms = 0;
uint32_t stall_start_ms  = 0;
uint32_t last_poll_ms    = 0;

/* ── Homing ─────────────────────────────────────────────────────────────────── */

void startHoming() {
    Serial.println("[AIRBRAKE] Starting sensorless homing...");
    homed = false;

    odrive.clearErrors();

    // Low current cap so stalling at the end-stop is safe
    odrive.setParameter("axis0.motor.config.current_lim", String(HOMING_CURRENT_LIMIT));
    odrive.setParameter("axis0.controller.config.control_mode", "2");  // VELOCITY_CONTROL
    odrive.setParameter("axis0.controller.config.input_mode",   "1");  // PASSTHROUGH

    while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrive.clearErrors();
        odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        delay(10);
    }

    odrive.setVelocity(HOMING_VELOCITY);

    state           = State::Homing;
    homing_start_ms = millis();
    stall_start_ms  = 0;
}

// Called once a genuine stall has been confirmed at the end-stop
void acceptHomePosition() {
    Serial.println("[AIRBRAKE] End-stop confirmed. Zeroing position.");

    odrive.setVelocity(0.0f);
    delay(300);

    // Snapshot the raw ODrive position — this becomes our zero reference.
    // All future setpoints are (home_offset + desired_turns).
    home_offset = odrive.getParameterAsFloat("axis0.pos_estimate");

    // Restore trapezoidal position control for normal operation
    odrive.setParameter("axis0.controller.config.control_mode", "3");  // POSITION_CONTROL
    odrive.setParameter("axis0.controller.config.input_mode",   "5");  // TRAP_TRAJ
    odrive.setParameter("axis0.motor.config.current_lim",       String(RUN_CURRENT_LIMIT));

    odrive.setPosition(home_offset);  // hold at home

    homed      = true;
    target_pos = 0.0f;
    state      = State::Ready;
    Serial.println("[AIRBRAKE] Homing complete. System ready.");
}

void tickHoming() {
    uint32_t now = millis();

    if (now - homing_start_ms > HOMING_TIMEOUT_MS) {
        Serial.println("[AIRBRAKE] Homing timed out — check mechanism.");
        odrive.setState(AXIS_STATE_IDLE);
        state = State::Error;
        return;
    }

    float iq = fabsf(odrive.getParameterAsFloat("axis0.motor.current_control.Iq_measured"));

    if (state == State::Homing) {
        if (iq >= HOMING_STALL_THRESHOLD) {
            stall_start_ms = now;
            state = State::HomingConfirm;
            Serial.println("[AIRBRAKE] Stall candidate — confirming...");
        }

    } else if (state == State::HomingConfirm) {
        if (iq < HOMING_STALL_THRESHOLD) {
            // Current dropped — was a transient spike, not a real stall
            state = State::Homing;
        } else if (now - stall_start_ms >= HOMING_STALL_CONFIRM_MS) {
            acceptHomePosition();
        }
    }
}

/* ── Position control ───────────────────────────────────────────────────────── */

// fraction: 0.0 = fully retracted, 1.0 = fully deployed
void setPosition(float fraction) {
    if (!homed) {
        Serial.println("[AIRBRAKE] Cannot move — not homed yet.");
        return;
    }

    fraction   = constrain(fraction, 0.0f, 1.0f);
    target_pos = fraction * DEPLOY_TURNS;
    odrive.setPosition(home_offset + target_pos);
    state = State::Moving;

    Serial.print("[AIRBRAKE] Moving to ");
    Serial.print(fraction * 100.0f, 1);
    Serial.print("%  (");
    Serial.print(target_pos, 3);
    Serial.println(" turns from home)");
}

void deploy()  { setPosition(1.0f); }
void retract() { setPosition(0.0f); }

/* ── Serial debug interface ─────────────────────────────────────────────────── */
// Replace this with your flight computer's command source when ready.

void handleSerialCommands() {
    if (!Serial.available()) return;

    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    if      (input == "deploy")        { deploy(); }
    else if (input == "retract")       { retract(); }
    else if (input == "home")          { startHoming(); }
    else if (input.startsWith("pos ")) { setPosition(input.substring(4).toFloat()); }
    else if (input == "status") {
        ODriveFeedback feedback = odrive.getFeedback();
        Serial.print("State=");      Serial.print((int)state);
        Serial.print("  Homed=");    Serial.print(homed);
        Serial.print("  Pos=");      Serial.print(feedback.pos - home_offset, 4);
        Serial.print(" turns  Vel="); Serial.print(feedback.vel, 3);
        Serial.print(" t/s  Target="); Serial.print(target_pos, 4);
        Serial.println(" turns");
    }
    else if (input == "errors") {
        Serial.print("AxisState=");  Serial.println(odrive.getState());
    }
}

/* ── Arduino entry points ───────────────────────────────────────────────────── */

void setup() {
    Serial.begin(115200);
    // Wait up to 3 s for USB-CDC to enumerate (remove for headless flight)
    for (uint32_t t = millis(); !Serial && millis() - t < 3000; ) {}

    ODRIVE_SERIAL.begin(ODRIVE_BAUD);
    Serial.println("[AIRBRAKE] Waiting for ODrive...");

    // Block until the ODrive responds — getState() returns AXIS_STATE_UNDEFINED
    // if the ODrive is not yet online or UART is misconfigured.
    while (odrive.getState() == AXIS_STATE_UNDEFINED) {
        delay(100);
    }

    Serial.print("[AIRBRAKE] ODrive found. Vbus=");
    Serial.print(odrive.getParameterAsFloat("vbus_voltage"));
    Serial.println(" V");

    odrive.setState(AXIS_STATE_IDLE);
    delay(200);
    odrive.clearErrors();

    startHoming();
}

void loop() {
    // Run homing until it resolves (or times out)
    if (state == State::Homing || state == State::HomingConfirm) {
        tickHoming();
        return;  // don't check move-complete or faults while homing
    }

    // Poll position and velocity to detect move completion
    if (state == State::Moving) {
        ODriveFeedback feedback = odrive.getFeedback();
        float current_pos = feedback.pos - home_offset;
        bool position_reached = fabsf(current_pos - target_pos) < POSITION_TOLERANCE;
        bool motor_settled    = fabsf(feedback.vel) < VELOCITY_AT_REST;
        if (position_reached && motor_settled) {
            state = State::Ready;
            Serial.println("[AIRBRAKE] Move complete.");
        }
    }

    // Periodic reminder while in error state
    if (state == State::Error) {
        static uint32_t last_print_ms = 0;
        if (millis() - last_print_ms > 2000) {
            last_print_ms = millis();
            Serial.println("[AIRBRAKE] Error state — send 'home' to retry.");
        }
    }

    handleSerialCommands();
}
