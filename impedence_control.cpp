#include <iostream>
#include <cstdio>
#include <cmath>

using namespace std;

int main() {

float kp = 1.0f, ki = 0.0f, kd = 0.0f;
float goal_point = 0.0; 
float error = 0.0; 
float prev_error = 0.0; 
float integral = 0.0; 
float derivative = 0.0; 
float output = 0.0; 
float sample_time = 0.1; 
float acceleration, velocity, position;


    float mass = 1.0f, damping = 1.0f, stiffness = 1.0f, damping_coeff = 1.0f;


    while (true) {

        float current_point = get_current_value(); 
        error = goal_point - current_point;

        acceleration = get_status()(0);
        velocity = get_status()(1);
        position = get_status()(2);

        integral = integral + error * sample_time;
        derivative = (error - prev_error) / sample_time;

        damping = damping_value(velocity, damping_coeff);
        output = kp * error + ki * integral + kd * derivative - mass * acceleration - damping * velocity + stiffness * position;
        prev_error = error;

        motor_control(output);  //함수 구현 필요
        sleep(sample_time);
    }

    return 0;
}


float damping_value(float velocity, float damping_coeff){
    float damping = damping_coeff * velocity * velocity;
    return damping;
}