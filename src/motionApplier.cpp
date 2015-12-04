#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <iostream>

#define minPWM 120
using namespace std;

float presentAngularPosition, previousAngularPosition;
float finalAngularPosition = 0;
float error, output;
bool initData =false;

std_msgs::Int32 pwm;
std_msgs::Int32 dir;//that we have to send

void turningOutputPWMMapping(float output){
	float maxOutput=120, minOutput=-120,scale;
	if(output > maxOutput)
		output = maxOutput;
	if(output < minOutput)
		output = minOutput;
	scale = 510.0/(maxOutput- minOutput);
	float temp;

	temp = output*scale;

	if(temp<0){
		pwm.data = (int)temp;
		dir.data = 3; //anti-clockwise
	}
	else{
		pwm.data = -1*(int)temp;
		dir.data = 4; //clockwise
	}
}

//This function is used to turn in the x-y plane we will use for this turn
void turnxyPlane(){

	int loopRate =10 ;
	ros::NodeHandle nh;
	ros::Publisher PWM=nh.advertise<std_msgs::Int32>("PWM",1000);
	ros::Publisher direction=nh.advertise<std_msgs::Int32>("direction",1000);
	ros::Rate loop_rate(loopRate);
	pwm.data=0;
	dir.data=5;

	while(!initData && ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

	float derivative=0,integral=0,dt=1.0/loopRate,p=1,i=0,d=0;
	bool reached=false;

	while(ros::ok()){
		error = finalAngularPosition - presentAngularPosition;
		integral+= (error*dt);
		derivative = (presentAngularPosition- previousAngularPosition)/dt;

		output= (p*error)+(i*integral)+(d*derivative);

		turningOutputPWMMapping(output);

		//this lower limit depends upon the bot itself
		if(pwm.data < minPWM)
			pwm.data= minPWM;


		PWM.publish(pwm);
		direction.publish(dir);
		ROS_INFO("pwm send to arduino %d in %d", pwm.data,dir.data);
		
		ros::spinOnce();
		loop_rate.sleep();

		if(error<5 && error >-5){//assuming that this angle is in degree
			//write something to calculate the time we will wait for and check if we are in the range of 5 degrees
			// we can also check that if we are in this range and the angular velocity is also small then we can assume
			// that we are stable and we can now start moving forward
			reached=true;
			pwm.data = 255;
			dir.data = 1;
			PWM.publish(pwm);
			direction.publish(dir);
			ROS_INFO("thrusters forward");			
		}

		if(reached){
			while(ros::ok()){
				PWM.publish(pwm);
				direction.publish(dir);
				ROS_INFO("thrusters forward");							
				loop_rate.sleep();
			}
			break;
		}	
	}
}

void yawCb(std_msgs::Float64 msg){
	previousAngularPosition = presentAngularPosition;
	presentAngularPosition = msg.data;

//	ROS_INFO("New angular postion %f", msg.data);

	if(initData == false){
		initData = true;
		presentAngularPosition = msg.data;
		previousAngularPosition = presentAngularPosition;
		ROS_INFO("Target recieved");
	}
}

int main(int argc, char** argv){

	ros::init(argc,argv,"motionApplier");
	ros::NodeHandle n;
	ros::Subscriber yaw=n.subscribe<std_msgs::Float64>("lineAngle",1000,&yawCb);
	turnxyPlane();
	ros::spin();
	return 0;
}