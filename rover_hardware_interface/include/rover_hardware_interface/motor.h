#include<iostream>

namespace rovercpp
{
	class Motor {
		public:
      Motor(); //default constructor
			Motor(double kp, double kd, double ki);
			virtual ~Motor();

      // returns speed to command using desired velocity and PID control
      double controlPID(const double& desired_speed);
      // set the speed of the motor, useful in simulation
      void setSpeed(const double& actual_speed);
      // read speed of motor
      double getSpeed(void);

		private:

      //gains to control motor speed
      double kp_;
      double kd_;
      double ki_;

      double _last_speed; // rad/s
      double _sum_error; // rad/s
      double _last_error; // rad/s

	};
}
