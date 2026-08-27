/*
 * m10q_qwiic_check.ino
 *
 * Verifies that the u-blox M10Q is reachable over I2C (Qwiic) and
 * accepts basic configuration commands.
 *
 * Wiring (Qwiic cable, RP2350):
 *   GP4  →  SDA  (yellow)
 *   GP5  →  SCL  (blue)
 *   3.3V →  3.3V (red)
 *   GND  →  GND  (black)
 *
 * Board: Raspberry Pi Pico 2 (RP2350)
 * Library: SparkFun u-blox GNSS v3  (install via Library Manager)
 */

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>

SFE_UBLOX_GNSS gps;

// ── Config to verify ─────────────────────────────────────────────────────
static const uint8_t  TARGET_HZ   = 1;
static const uint32_t I2C_FREQ    = 400000;
static const uint8_t  GNSS_ADDR   = 0x42;

// ─────────────────────────────────────────────────────────────────────────

void pass(const char* msg) { Serial.printf("  [PASS] %s\n", msg); }
void fail(const char* msg) { Serial.printf("  [FAIL] %s\n", msg); }

// ─────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("========================================");
    Serial.println(" u-blox M10Q  —  Qwiic connection check");
    Serial.println("========================================\n");

    // ── 1. I2C bus ────────────────────────────────────────────────────────
    Serial.println("[ 1 ] I2C bus");
    Wire.setSDA(17);
    Wire.setSCL(16);
    Wire.begin();
    Wire.setClock(I2C_FREQ);

    // Raw ping before involving the library
    Wire.beginTransmission(GNSS_ADDR);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
        pass("Device found at 0x42");
    } else {
        fail("No response at 0x42  (check wiring / power)");
        Serial.println("\nHalting — fix wiring and reset.");
        while (true) delay(1000);
    }

    // ── 2. Library initialisation ─────────────────────────────────────────
    Serial.println("\n[ 2 ] Library init (SFE_UBLOX_GNSS)");
    if (gps.begin(Wire, GNSS_ADDR)) {
        pass("begin() succeeded — module acknowledged");
    } else {
        fail("begin() failed — module did not respond to UBX handshake");
        while (true) delay(1000);
    }

    // ── 3. Protocol version ───────────────────────────────────────────────
    Serial.println("\n[ 3 ] Firmware / protocol version");
    if (gps.getModuleInfo()) {
        Serial.printf("  Module:   %s\n",  gps.getModuleName());
        Serial.printf("  Firmware: %s\n",  gps.getFirmwareVersionLow());
        Serial.printf("  Protocol: %d.%02d\n",
                      gps.getProtocolVersionHigh(),
                      gps.getProtocolVersionLow());
        pass("Version info retrieved");
    } else {
        fail("Could not retrieve version info (non-fatal)");
    }

    // ── 4. Output mode ────────────────────────────────────────────────────
    Serial.println("\n[ 4 ] Set I2C output to UBX-only");
    if (gps.setI2COutput(COM_TYPE_UBX)) {
        pass("I2C output = UBX (NMEA suppressed)");
    } else {
        fail("setI2COutput() returned false");
    }

    // ── 5. Navigation rate ────────────────────────────────────────────────
    Serial.printf("\n[ 5 ] Set navigation rate to %u Hz\n", TARGET_HZ);
    if (gps.setNavigationFrequency(TARGET_HZ)) {
        uint8_t actual = gps.getNavigationFrequency();
        if (actual == TARGET_HZ) {
            Serial.printf("  Confirmed rate: %u Hz\n", actual);
            pass("Navigation rate set and verified");
        } else {
            Serial.printf("  Reported rate: %u Hz (expected %u)\n", actual, TARGET_HZ);
            fail("Rate mismatch (non-fatal)");
        }
    } else {
        fail("setNavigationFrequency() returned false");
    }

    // ── 6. Enable NAV-PVT ─────────────────────────────────────────────────
    Serial.println("\n[ 6 ] Enable NAV-PVT auto-messages");
    if (gps.setAutoPVT(true)) {
        pass("NAV-PVT auto-send enabled");
    } else {
        fail("setAutoPVT() returned false");
    }

    // ── 7. Live NAV-PVT round-trip ────────────────────────────────────────
    Serial.println("\n[ 7 ] NAV-PVT round-trip (waiting up to 3 s)");
    uint32_t deadline = millis() + 3000;
    bool gotPvt = false;
    while (millis() < deadline) {
        if (gps.getPVT()) { gotPvt = true; break; }
        delay(50);
    }
    if (gotPvt) {
        pass("NAV-PVT packet received");
        Serial.printf("  Fix type : %u  (%s)\n",
                      gps.getFixType(), fixLabel(gps.getFixType()));
        Serial.printf("  Satellites: %u\n", gps.getSIV());
        Serial.printf("  Lat/Lon  : %.7f, %.7f\n",
                      gps.getLatitude()  / 1e7,
                      gps.getLongitude() / 1e7);
    } else {
        fail("No NAV-PVT packet within 3 s — module may still be starting up");
    }

    // ── Summary ───────────────────────────────────────────────────────────
    Serial.println("\n========================================");
    Serial.println(" Check complete. Streaming fix status...");
    Serial.println("========================================\n");
}

// ─────────────────────────────────────────────────────────────────────────

void loop() {
    if (gps.getPVT()) {
        uint8_t fix = gps.getFixType();
        if (fix >= 2) {
            Serial.printf("FIX  type=%u  sats=%u  lat=%.7f  lon=%.7f  alt=%.1fm\n",
                          fix,
                          gps.getSIV(),
                          gps.getLatitude()    / 1e7,
                          gps.getLongitude()   / 1e7,
                          gps.getAltitudeMSL() / 1000.0);
        } else {
            Serial.printf("Acquiring...  type=%u  sats=%u\n",
                          fix, gps.getSIV());
        }
    }
    delay(1000);
}

// ─────────────────────────────────────────────────────────────────────────

const char* fixLabel(uint8_t fixType) {
    switch (fixType) {
        case 0: return "no fix";
        case 1: return "dead reckoning";
        case 2: return "2D";
        case 3: return "3D";
        case 4: return "GNSS + dead reckoning";
        case 5: return "time only";
        default: return "unknown";
    }
}
