#include <array>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <vector>
#include "config_data.h"
#include "computations.h"

std::array<float, 2> get_acceleration_2d(float altitude, float velocity_x, float velocity_y, float mass, float deployPct) {
    // Constants
    const float g = 9.80665f; // gravitational acceleration in m/s^2

    // Calculate speed and drag force
    float velocity = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
    float dragForce = get_total_drag(velocity, altitude, deployPct);

    // Calculate acceleration components
    float accel_x = - (dragForce / mass) * (velocity_x / velocity); // Drag deceleration in x
    float accel_y = -g - (dragForce / mass) * (velocity_y / velocity); // Gravity and drag deceleration in y

    return {accel_x, accel_y};
}

float get_isa_pressure(float altitude) {
    float P0 = 101325.0f; // Pa at sea level
    float Tb = 288.15f;   // K at sea level
    float Lb = 0.0065f;   // K/m temperature lapse rate
    float R = 287.05f;    // J/(kg·K) specific gas constant for dry air
    float g0 = 9.80665f;  // m/s^2 standard gravity

    float h = altitude;
    float T = Tb - Lb * h;
    float P = P0 * std::pow(T / Tb, g0 / (R * Lb));
    return P;
}
float get_isa_temperature(float altitude) {
    float Tb = 288.15;     // K at sea level
    float Lb = 0.0065;     // K/m temperature lapse rate
    float hb = 0;          // m base altitude (sea level)

    float h = altitude;
    float T = Tb - (h - hb) * Lb;
    return T;
}
float get_isa_density(float altitude) {
    float P = get_isa_pressure(altitude);
    float T = get_isa_temperature(altitude);
    float R = 287.058f; // J/(kg·K) specific gas constant for dry air

    float rho = P / (R * T);
    return rho;
}
float get_isa_speed_of_sound(float altitude) {
    float T = get_isa_temperature(altitude);
    float gamma = 1.4f; // Ratio of specific heats for air
    float R = 287.058f;  // J/(kg·K) specific gas constant for dry air

    float a = std::sqrt(gamma * R * T);
    return a;
}

float get_pressure(float altitude) {
    const int index = std::lower_bound(BALLOON_ALT_LUT, BALLOON_ALT_LUT + BALLOON_TABLE_SIZE, altitude) - BALLOON_ALT_LUT;
    const float pressure = BALLOON_PRESSURE_LUT[std::clamp(index, 0, static_cast<int>(BALLOON_TABLE_SIZE - 1))];
    return pressure * 100.0f; // Convert hPa to Pa
}
float get_temperature(float altitude) {
    const int index = std::lower_bound(BALLOON_ALT_LUT, BALLOON_ALT_LUT + BALLOON_TABLE_SIZE, altitude) - BALLOON_ALT_LUT;
    const float temperature = BALLOON_TEMP_LUT[std::clamp(index, 0, static_cast<int>(BALLOON_TABLE_SIZE - 1))];
    return temperature + 273.15f;
}
float get_density(float altitude) {
    float pressure = get_pressure(altitude);
    float temperature = get_temperature(altitude);
    const float R = 287.058f; // J/(kg·K) specific gas constant for dry air

    float density = pressure / (R * temperature); //rho = P / (R * T)
    return density;
}
float get_speed_of_sound(float altitude) {
    float temperature = get_temperature(altitude);
    const float gamma = 1.4f; // Ratio of specific heats for air
    const float R = 287.058f;  // J/(kg·K) specific gas constant for dry air

    float speed_of_sound = std::sqrt(gamma * R * temperature);
    return speed_of_sound;
}

float get_total_drag(float velocity, float altitude, float deployPct) {
    float pressure = get_pressure(altitude);
    float rho = get_density(altitude);
    float gamma = 1.4f;
    float C = get_speed_of_sound(altitude);
    float mach = velocity / C;

    float q = pressure * std::pow(1.0f + (((gamma - 1.0f)/2.0f) * mach * mach), (gamma/(gamma - 1.0f))) - pressure;
    // percent deploy is linear in projected area, so the flap angle is the arcsin.
    // Brake Cd fit is a function of flap angle (0.35 stowed to 1.15 at 90 deg).
    // Matches getTotalDrag in the python sim
    if (deployPct < 0.0f) deployPct = 0.0f;
    if (deployPct > 100.0f) deployPct = 100.0f;
    float deploy_angle = std::asin(deployPct / 100.0f) * (180.0f / 3.14159265f);
    float area_deployed = (deployPct / 100.0f) * BRAKE_FACE_AREA;
    float brake_drag = q * area_deployed * (0.00889f * deploy_angle + 0.35f);

    const int index = std::lower_bound(AERO_MACH_LUT, AERO_MACH_LUT + AERO_TABLE_SIZE, mach) - AERO_MACH_LUT;
    const float cd = AERO_CD_LUT[std::clamp(index, 0, static_cast<int>(AERO_TABLE_SIZE - 1))];

    float body_drag = 0.5f * rho * velocity * velocity * ROCKET_REFERENCE_AREA * cd;

    return brake_drag + body_drag;
}
float get_brake_drag(float velocity, float altitude, float deployPct) {
    float pressure = get_pressure(altitude);
    float gamma = 1.4f;
    float C = get_speed_of_sound(altitude);
    float mach = velocity / C;

    float q = pressure * std::pow(1 + (((gamma - 1)/2) * mach * mach), (gamma/(gamma - 1))) - pressure;
    // same brake model as get_total_drag: area linear in percent, Cd fit vs flap angle
    if (deployPct < 0.0f) deployPct = 0.0f;
    if (deployPct > 100.0f) deployPct = 100.0f;
    float deploy_angle = std::asin(deployPct / 100.0f) * (180.0f / 3.14159265f);
    float area_deployed = (deployPct / 100.0f) * BRAKE_FACE_AREA;
    float brake_drag = q * area_deployed * (0.00889f * deploy_angle + 0.35f);

    return brake_drag;
}
float get_body_drag(float velocity, float altitude) {
    float rho = get_density(altitude);
    float C = get_speed_of_sound(altitude);
    float mach = velocity / C;

    const int index = std::lower_bound(AERO_MACH_LUT, AERO_MACH_LUT + AERO_TABLE_SIZE, mach) - AERO_MACH_LUT;
    const float cd = AERO_CD_LUT[std::clamp(index, 0, static_cast<int>(AERO_TABLE_SIZE - 1))];

    float body_drag = 0.5f * rho * velocity * velocity * ROCKET_REFERENCE_AREA * cd;

    return body_drag;
}

rk4State rk4_step(rk4State state, float dt, float deployPct) {
    auto acceleration = get_acceleration_2d(state.y, state.vx, state.vy, ROCKET_MASS, deployPct);

    rk4State k1;
    k1.x = state.vx;
    k1.y = state.vy;
    k1.vx = acceleration[0];
    k1.vy = acceleration[1];

    rk4State tempState;
    tempState.x = state.x + 0.5f * dt * k1.x;
    tempState.y = state.y + 0.5f * dt * k1.y;
    tempState.vx = state.vx + 0.5f * dt * k1.vx;
    tempState.vy = state.vy + 0.5f * dt * k1.vy;
    acceleration = get_acceleration_2d(tempState.y, tempState.vx, tempState.vy, ROCKET_MASS, deployPct);

    rk4State k2;
    k2.x = tempState.vx;
    k2.y = tempState.vy;
    k2.vx = acceleration[0];
    k2.vy = acceleration[1];

    tempState.x = state.x + 0.5f * dt * k2.x;
    tempState.y = state.y + 0.5f * dt * k2.y;
    tempState.vx = state.vx + 0.5f * dt * k2.vx;
    tempState.vy = state.vy + 0.5f * dt * k2.vy;
    acceleration = get_acceleration_2d(tempState.y, tempState.vx, tempState.vy, ROCKET_MASS, deployPct);

    rk4State k3;
    k3.x = tempState.vx;
    k3.y = tempState.vy;
    k3.vx = acceleration[0];
    k3.vy = acceleration[1];

    tempState.x = state.x + dt * k3.x;
    tempState.y = state.y + dt * k3.y;
    tempState.vx = state.vx + dt * k3.vx;
    tempState.vy = state.vy + dt * k3.vy;
    acceleration = get_acceleration_2d(tempState.y, tempState.vx, tempState.vy, ROCKET_MASS, deployPct);

    rk4State k4;
    k4.x = tempState.vx;
    k4.y = tempState.vy;
    k4.vx = acceleration[0];
    k4.vy = acceleration[1];

    rk4State newState;
    newState.x = state.x + (dt / 6.0f) * (k1.x + 2.0f * k2.x + 2.0f * k3.x + k4.x);
    newState.y = state.y + (dt / 6.0f) * (k1.y + 2.0f * k2.y + 2.0f * k3.y + k4.y);
    newState.vx = state.vx + (dt / 6.0f) * (k1.vx + 2.0f * k2.vx + 2.0f * k3.vx + k4.vx);
    newState.vy = state.vy + (dt / 6.0f) * (k1.vy + 2.0f * k2.vy + 2.0f * k3.vy + k4.vy);

    return newState;
}