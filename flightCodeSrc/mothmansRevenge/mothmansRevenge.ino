#include "computations.h"
#include "config_data.h"

float predictApgee();
float dt = 0.1f; // time step for RK4 integration

rk4State startState = {1.0f, 1110.0f, 53.3f, 308.85f}; // initial conditions: x=0, y=0, vx=0, vy=0

void setup() {
    // Initialization code here
    Serial.begin(115200);

}

void loop() {
    // Main code here
    Serial.println("\n\nStarting Computation");
    Serial.printf("Rk4 test: %f\n", rk4_step(startState, dt, 0.0f).y);
    Serial.printf("get drag: %f\n", get_total_drag(startState.vy, startState.y, 0.0));
    int startTime = millis();
    float predictedApogee = predictApgee(startState, 0.0f, dt);
    int elapsedTime = millis() - startTime;
    Serial.println("Computation Complete");
    Serial.printf("Predicted Apogee: %f\n", predictedApogee);
    Serial.printf("Compute Time: %d ms", elapsedTime);

    delay(2000);
}

float predictApgee(rk4State initialState, float deployAngle, float dt) {
    // Predict apogee logic here
    rk4State state = initialState;
    Serial.printf("Alt: %f\nVy: %f\n", state.y, state.vy);
    int iterations = 0;
    while (state.vy > 0.0f) {
        state = rk4_step(state, dt, deployAngle);
        iterations++;
    }
    Serial.printf("Iterations: %d\n", iterations);
    return state.y;
}

