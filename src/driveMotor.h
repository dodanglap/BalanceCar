#ifndef DRIVEMOTOR_H
#define DRIVEMOTOR_H

class DriveMotor
{

public:
    int in1_pin, in2_pin, en_pin;
    void run_forward(int pwm_speed);
    void run_backward(int pwm_speed);
    void stop_motor();
    
};

#endif


