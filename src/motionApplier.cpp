#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <iostream>

using namespace std;

float presentAngularPosition, previousAngularPosition;
std_msgs::Int32 pwm;

int turningOutputPWMMapping(float output){
	float maxOutput=120, minOutput=-120,scale;
	int pwm;
	if(output > maxOutput)
		output = maxOutput;
	if(output < minOutput)
		output = minOutput;
	scale = 510.0/(maxOutput- minOutput);
	float temp;

	temp = output*scale;
	pwm= (int)temp;

	return pwm;
}

//This function is used to turn in the x-y plane we will use for this turn
void turnxyPlane(float theta){
	float finalAngularPosition, error, output;
	finalAngularPosition = presentAngularPosition + theta;
	error=finalAngularPosition- presentAngularPosition;
	
	int loopRate =100 ;
	float derivative=0,integral=0,dt=1.0/loopRate,p=1,i=0,d=0;
	bool reached=false;
	
	ros::NodeHandle nh;
	ros::Publisher turningPWM=nh.advertise<std_msgs::Int32>("turningPWM",1000);
	ros::Rate loop_rate(loopRate);
	pwm.data=0;
	
	while(ros::ok()){
		error = finalAngularPosition - presentAngularPosition;		
		integral+= (error*dt);
		derivative = (presentAngularPosition- previousAngularPosition)/dt;

		output= (p*error)+(i*integral)+(d*derivative);

		pwm.data = turningOutputPWMMapping(output);

		// //use for giving manual input
		// int temp;
		// cout << "give pwm input" << endl;
		// cin >> temp;
		// pwm.data = temp;

		//this lower limit depends upon the bot itself
		if(pwm.data < 120 && pwm.data >0)
			pwm.data= 120;
		if(pwm.data > -120 && pwm.data <0)
			pwm.data= -120;	

		//this statement just for testing
		pwm.data = -pwm.data;
		turningPWM.publish(pwm);
		ROS_INFO("pwm send to arduino %d", pwm.data);
		ros::spinOnce();
		loop_rate.sleep();

		if(error<5){//assuming that this angle is in degree
			//write something to calculate the time we will wait for and check if we are in the range of 5 degrees
			// we can also check that if we are in this range and the angular velocity is also small then we can assume
			// that we are stable and we can now start moving forward
			reached=true;
			pwm.data = 0;
			turningPWM.publish(pwm);
			ROS_INFO("pwm send to arduino %d", pwm.data);					
		}

		if(reached)
			break;
	}
}


void yawCb(std_msgs::Float64 msg){
	previousAngularPosition = presentAngularPosition;
	presentAngularPosition= msg.data*180/3.14;
	ROS_INFO("New angular postion %f", msg.data);
}

int main(int argc, char** argv){
	float theta;// input theta will be positive if we have to rotate the bot ACW
	if(argc!=2){
		cout << "incorret number of arguments" << endl;
		return 1;
	}
	else
		theta=atof(argv[1]);
	cout << "theta is " <<argv[1] << endl;

	ros::init(argc,argv,"motionApplier");
	ros::NodeHandle n;
	ros::Subscriber yaw=n.subscribe<std_msgs::Float64>("yaw",1000,&yawCb);
	turnxyPlane(theta);
	ros::spin();
	return 0;
}