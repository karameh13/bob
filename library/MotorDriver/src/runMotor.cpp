#include "MotorDriver.h"
//#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>
//#include <ros/console.h>

//#include <JetsonGPIO.h>

//ros::NodeHandle nh;
//geometry_msgs::Twist msg;


//MD::Motor rightMotor(35, 33, -255, 255);


	
	//double linearVelocityX = cmd_vel.linear.x;
	//double angularVelocityZ = cmd_vel.angular.z;
	
	
	//leftWheel = (linearVelocityX - angularVelocityZ) * 100.0;
	//int rightWheel = (linearVelocityX + angularVelocityZ) * 100.0;
	
	
	//leftMotor.setSpeed(leftWheel);
	//rightMotor.setSpeed(rightWheel);
	
	//double newLeftWheel = leftMotor.getSpeed();
	//int newRightWheel = rightMotor.getSpeed();
	
	
	
	//rightMotorPWMPin.ChangeDutyCycle(newRightWheel);
	/*
	ROS_INFO_STREAM("------------------------------------");
	ROS_INFO_STREAM("linVel: " << linearVelocityX);
	ROS_INFO_STREAM("angVel: " << angularVelocityZ);
	ROS_INFO_STREAM("wheelSpeed: " << leftWheel);
	ROS_INFO_STREAM("newWheelSpeed: " << newLeftWheel);
	     if(leftWheel > 0) {
		ROS_INFO_STREAM("Going forward");
	}
	else if(leftWheel < 0) {
		ROS_INFO_STREAM("Going backwards");
	}
	else {
		ROS_INFO_STREAM("Not moving");
	}
	
	     if(linearVelocityX > 0 && angularVelocityZ < 0) {
		ROS_INFO_STREAM("up right");
	}
	else if(linearVelocityX > 0 && angularVelocityZ > 0) {
		ROS_INFO_STREAM("up left");
	}
	else if(linearVelocityX < 0 && angularVelocityZ < 0) {
		ROS_INFO_STREAM("down right");
	}
	else if(linearVelocityX < 0 && angularVelocityZ > 0) {
		ROS_INFO_STREAM("down left");
	}
	
	ROS_INFO_STREAM("------------------------------------");
	
	
	if (newLeftWheel == 0.0) {
		leftMotor.stop();
		//rightMotor.stop();
		
		leftMotorPWMPin.stop();
		//rightMotorPWMPin.stop();
		
		//GPIO::cleanup();
	}*/
}

//double linx, angZ;

int main(int argc, char** argv) {

	// motor(directionPin, PwmPin, minSpeed, maxSpeed)
	MD::Motor leftMotor(31, 32, 0.0, 100.0);
	
	int leftMotorPwmPin = 32; //leftMotor.getPwmPin();
//int rightMotorPwmPin = rightMotor.getPwmPin();

	
//int leftMotorDirection = leftMotor.getDirection();


	GPIO::PWM leftMotorPWMPin(32, 1000); // for GPIO::BOARD
//GPIO::PWM rightMotorPWMPin(rightMotorPwmPin, 100);

//double leftWheel = leftMotor.getSpeed();

//void messageCallback(const geometry_msgs::Twist& cmd_vel) {
	
	leftMotorPWMPin.start(0.0);
//rightMotorPWMPin.start(10.0);

	leftMotorPWMPin.ChangeDutyCycle(100.0);
	
	/*
		ros::init(argc, argv, "run_motor");
		ros::NodeHandle nh;
		
		//ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", &messageCallback);
		ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, messageCallback);	
		
		ROS_INFO_STREAM("Pwm pin: " << leftMotorPwmPin);
		ROS_INFO_STREAM("Direction: " << leftMotorDirection);
			
		
		ros::spin();
	*/
}
