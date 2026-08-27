#pragma once

#include <array>
#include <cmath>
#include <algorithm>
#include "config_data.h"

struct rk4State {
    float x;
    float y;
    float vx;
    float vy;
};

std::array<float, 2> get_acceleration_2d(float altitude, float velocity_x, float velocity_y, float mass, float deployAngle);

float get_isa_pressure(float altitude);
float get_isa_temperature(float altitude);
float get_isa_density(float altitude);  
float get_isa_speed_of_sound(float altitude);

float get_pressure(float altitude);
float get_temperature(float altitude);  
float get_density(float altitude);  
float get_speed_of_sound(float altitude);

float get_total_drag(float velocity, float altitude, float deployAngle);
float get_brake_drag(float velocity, float altitude, float deployAngle);
float get_body_drag(float velocity, float altitude, float deployAngle);

rk4State rk4_step(rk4State state, float dt, float deployAngle);

