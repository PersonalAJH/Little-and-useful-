#include <iostream>
#include <cstdio>
#include <cmath>

using namespace std;

int main() {
// pid 제어 기본 설정값
float kp = 1.0, ki = 0.0, kd = 0.0;
float goal_point = 0.0; 
float error = 0.0; 
float prev_error = 0.0; 
float integral = 0.0; 
float derivative = 0.0; 
float output = 0.0; 
float sample_time = 0.1; // 샘플링 시간 (초)


    while (true) {

        float current_point = get_current_value();  //함수 구현 필요
        error = goal_point - current_point;

        //I,D gain
        integral = integral + error * sample_time;
        derivative = (error - prev_error) / sample_time;

        //모터 출력 output
        output = kp * error + ki * integral + kd * derivative;

        //D gain을 위한 이전 에러 저장
        prev_error = error;

        motor_control(output);  //함수 구현 필요

        sleep(sample_time);    // 사실 이거보다는 내가 사용하는 timer eclipse가 더 좋음
    }
    return 0;
}