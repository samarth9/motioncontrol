#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
//#include <std_msgs/Char.h>
#include <iostream>
#define minPWM 220
using namespace std;

float presentAngularPosition, previousAngularPosition;
float finalAngularPosition, error, output;
bool initData =false;
float theta;

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
	//pwm= (int)temp;

	if(temp>0){
		pwm.data = (int)temp;
		dir.data = 3; 
	}
	else{
		pwm.data = -1*(int)temp;
		dir.data = 4;
	}
}

//This function is used to turn in the x-y plane we will use for this turn
void turnxyPlane(float theta){
	finalAngularPosition = presentAngularPosition + theta;
	if(finalAngularPosition >= 180)
		finalAngularPosition = finalAngularPosition -360;
	else if(finalAngularPosition <= -180)
		finalAngularPosition = finalAngularPosition +360;

	error=finalAngularPosition- presentAngularPosition;
	
	int loopRate =10 ;
	float derivative=0,integral=0,dt=1.0/loopRate,p=1,i=0,d=0;
	bool reached=false;
	
	ros::NodeHandle nh;
	ros::Publisher PWM=nh.advertise<std_msgs::Int32>("PWM",1000);
	ros::Publisher direction=nh.advertise<std_msgs::Int32>("direction",1000);
	ros::Rate loop_rate(loopRate);
	pwm.data=0;
	dir.data=5;
	while(ros::ok()){
		error = finalAngularPosition - presentAngularPosition;		
		integral+= (error*dt);
		derivative = (presentAngularPosition- previousAngularPosition)/dt;

		output= (p*error)+(i*integral)+(d*derivative);

		turningOutputPWMMapping(output);

		// //use for giving manual input
		// int temp;
		// cout << "give pwm input" << endl;
		// cin >> temp;
		// pwm.data = temp;

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
			dir.data = 2;
			PWM.publish(pwm);
			direction.publish(dir);
			ROS_INFO("thrusters forward");			
		}

		if(reached){
			int temp = 20;
			while(ros::ok() && temp--){
				PWM.publish(pwm);
				direction.publish(dir);
				ROS_INFO("thrusters forwards");
				ros::spinOnce();
				loop_rate.sleep();
			}
			pwm.data = 255;
			dir.data = 5;
			temp = 10;
			while(ros::ok() && temp--){
				PWM.publish(pwm);
				direction.publish(dir);
				ROS_INFO("thrusters stopped");
				ros::spinOnce();
				loop_rate.sleep();
			}
			break;
		}
	}
}


void yawCb(std_msgs::Float64 msg){
	previousAngularPosition = presentAngularPosition;
	presentAngularPosition= msg.data;
//	ROS_INFO("New angular postion %f", msg.data);
	if(initData == false){
		finalAngularPosition = presentAngularPosition + theta;
		initData = true;
		if(finalAngularPosition >= 180)
			finalAngularPosition = finalAngularPosition -360;
		else if(finalAngularPosition <= -180)
			finalAngularPosition = finalAngularPosition +360;
	}
}

int main(int argc, char** argv){
	// input theta will be positive if we have to rotate the bot ACW
	if(argc!=2){
		cout << "incorret number of arguments" << endl;
		return 1;
	}
	else
		theta = atof(argv[1]);
	cout << "theta is " <<argv[1] << endl;

	ros::init(argc,argv,"motionApplier");
	ros::NodeHandle n;
	ros::Subscriber yaw=n.subscribe<std_msgs::Float64>("yaw",1000,&yawCb);
	turnxyPlane(0);
	turnxyPlane(theta);
	turnxyPlane(theta);
	turnxyPlane(theta);
	//ros::spin();
	return 0;
}