#include "driveMotor.h"
#include "Arduino.h"
// chứa nội dung function
void DriveMotor :: run_forward(int pwm_speed){
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
    analogWrite(en_pin, pwm_speed);

}

void DriveMotor :: run_backward(int pwm_speed){
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
    analogWrite(en_pin, pwm_speed);

}

void DriveMotor :: stop_motor(){
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
    analogWrite(en_pin, 0);
}

