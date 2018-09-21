#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include "wiringPi.h"
#include "softPwm.h"

/** Servo motor Pin Setting & Macro */
#define SERVO_PIN 0

#define LEFT_MAX_WIDTH 10
#define MID_WIDTH 15
#define RIGHT_MAX_WIDTH 20

/** DC motor Pin Setting & Macro */
#define M0_EN_PIN 1
#define M0_PWM_PIN 2

/** Parameter Variable */
int dutyCycle = 0; // DC motor PWM pin's dutyCycle
int preDutyCycle = 0;

float pGain = 0.01; // Controller P Gain
float prePGain = 0.01;

/** etc Variable */
int positionError = 0; // if error > 0 : Car's position is left side
int prePositionError = 0;

int iConstrain(int value, int min, int max)
{
	if (value < min)
	{
		value = min;
	}
	else if (value > max)
	{
		value = max;
	}
	return value;
}

void msgCallback(const std_msgs::Int32::ConstPtr &msg)
{
	positionError = msg->data;
}

int main(int argc, char **argv)
{
	// GPIO Setting
	wiringPiSetup();

	pinMode(M0_EN_PIN, OUTPUT);
	digitalWrite(M0_EN_PIN, 1); // Motor driver M0 enable

	softPwmCreate(SERVO_PIN, 0, 200);
	softPwmCreate(M0_PWM_PIN, 0, 200);

	softPwmWrite(SERVO_PIN, MID_WIDTH); // Heading : MID
	softPwmWrite(M0_PWM_PIN, dutyCycle);
	// ROS Setting
	ros::init(argc, argv, "topic_subscriber");

	ros::NodeHandle nh;

	nh.setParam("GainSet", pGain);
	nh.setParam("DutySet", dutyCycle);

	ros::Subscriber ros_linetracer_sub = nh.subscribe("ros_linetracer_msg", 100, msgCallback);
	ros::Rate r(66); // 66Hz
	while (1)
	{
		/** Param Receive */
		nh.getParam("GainSet", pGain);
		nh.getParam("DutySet", dutyCycle);

		int feedbackTerm = (int) (pGain * positionError);
		softPwmWrite(SERVO_PIN, iConstrain(MID_WIDTH + feedbackTerm, LEFT_MAX_WIDTH, RIGHT_MAX_WIDTH));
		softPwmWrite(M0_PWM_PIN, dutyCycle);
		
		if ((prePGain != pGain) || (preDutyCycle != dutyCycle)) // Param change message
		{
			prePGain = pGain;
			preDutyCycle = dutyCycle;
			ROS_INFO("pGain : %f, duty : %d \n", pGain, dutyCycle);
		}

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
