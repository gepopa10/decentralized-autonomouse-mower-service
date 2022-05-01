#include<iostream>
#include<rover_hardware_interface/motor.h>

namespace rovercpp
{
  Motor::Motor() {
  }

  Motor::Motor(double kp, double kd, double ki) :
  kp_(kp), kd_(kd), ki_(ki) {
    _sum_error = 0;
    _last_error = 0;
	}

	Motor::~Motor() {
	}

  double Motor::controlPID(const double& desired_speed) {

    double error = desired_speed - _last_speed;
    _sum_error += error;
    double control_output = kp_* error + kd_* (error - _last_error) + _sum_error;
    _last_error = error;

    return control_output;
  }

  void Motor::setSpeed(const double& actual_speed){
    _last_speed = actual_speed;
  }

  double Motor::getSpeed(void){
    double encoder_speed = 0.0;

    //TO DO using GPIO
    //Currently we use a python node to publish the speed

    return encoder_speed;
  }
}

int main(int argc, char** argv)
{
	rovercpp::Motor motor1(0.1,0.01,0.01);
  std::cout << "motor1 speed: " << motor1.getSpeed() << std::endl;
	return 0;
}
