#include "youBotIOHandler.h"
#include <signal.h>
#include "control_toolbox/pid.h"

// Ref: http://ibotics.ucsd.edu/trac/stingray/wiki/ROSNodeTutorialC%2B%2B
/**
 * //TODO:
 * - PID visual control
 * - Implementation of PID output scaling: either saturation block or dynamic range compression
 * - make safe shutdown routine more neat
 * 
 * //DONE:
 * - Put PID gains as arguments in the .launch file
 * - Try ROS::control_toolbox
 * 
 * 
 */
 
 volatile sig_atomic_t g_shutdown_request = 0;
 
 void youBotSafeShutdown(int sig){
	g_shutdown_request = 1;
	ROS_WARN("Shutdown signal received");
 }
 
 void limiter(float* value){
	float limitHi = 1;
	float limitLo = 0.05;
	
	// limiter so the robot will not blow up
	if (*value > limitHi)
		*value = 1;
	else if (*value < -limitHi)
		*value = -1;
	
	// limiter so the robot will not stutter
	if (*value < limitLo && *value > -limitLo)
		*value = 0;
 }
 
 
float pid(float error){
	// PID gains
	float Kp = 1.2;
	float Ki = 0;
	float Kd = 0;
	
	// taken from rate = 50Hz
	//TODO: is it correct?
	float dt = 1/50;
	
	float integral, derivative;
	float output = 0;
	
	integral = 0;
	derivative = 0;
	
	output = Kp*error + Ki*integral + Kd*derivative;
	
	// output limiter
	if (output > 1)
		output = 1;
	else if (output < -1)
		output = -1;
		
	return output;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "makeitmove");
	ros::NodeHandle n;
	/**
	 * Safe Shutdown
	 * - Ref: http://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/
	 * - Ref: https://code.ros.org/trac/ros/ticket/3417
	 * //Todo: will there be conflict with ROS sigint handler?
	 */
	signal(SIGINT, youBotSafeShutdown);
	
	YouBotIOHandler* yb = new YouBotIOHandler();
	
	ros::Subscriber sub_ObjDetect = n.subscribe("object_tracking/object_detected", 1000, &YouBotIOHandler::callbackObjDetected, yb);
	ros::Subscriber sub_PosX = n.subscribe("object_tracking/x_pos", 1000, &YouBotIOHandler::callbackPosX, yb);
	ros::Subscriber sub_PosY = n.subscribe("object_tracking/y_pos", 1000, &YouBotIOHandler::callbackPosY, yb);
	ros::Subscriber sub_area = n.subscribe("object_tracking/area", 1000, &YouBotIOHandler::callbackArea, yb);
	ros::Rate r(50);
	
	ros::Publisher pub_moveit = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	
	float cam_x, cam_y, cam_area;
	float out_lin_y;
	
	int captureSizeX, captureSizeY;
	n.getParam("/object_tracking/captureSizeX", captureSizeX);
	n.getParam("/object_tracking/captureSizeY", captureSizeY);

	ROS_WARN("PID gain values are taken from the .launch file!");

	double speed = 0.3;
	// default PID gains, if no params are set
	double Kp = 1.2;
	double Ki = 0.1;
	double Kd = 0;
	double iLimitHi = 0.5;
	double iLimitLo = -0.5;
	
	double tmp;
	
	if (n.getParam("/youbotPID/Kp", tmp))
		n.getParam("/youbotPID/Kp", Kp);
	if (n.getParam("/youbotPID/Ki", tmp))
		n.getParam("/youbotPID/Ki", Ki);
	if (n.getParam("/youbotPID/Kd", tmp))
		n.getParam("/youbotPID/Kd", Kd);
	if (n.getParam("/youbotPID/iLimitHi", tmp))
		n.getParam("/youbotPID/iLimitHi", iLimitHi);
	if (n.getParam("/youbotPID/iLimitLo", tmp))
		n.getParam("/youbotPID/iLimitLo", iLimitLo);
	if (n.getParam("/youbotPID/speed", tmp))
		n.getParam("/youbotPID/speed", speed);
	
	ROS_INFO("Using PID gains: [Kp, Ki, Kd] = [%.2f, %.2f, %.2f]; Integral [hi,lo] limits = [%.2f, %.2f]", Kp, Ki, Kd, iLimitHi, iLimitLo);
	ROS_WARN("Using %d%% speed", (int)(speed*100));
	
	control_toolbox::Pid pid;
	pid.initPid(Kp, Ki, Kd, iLimitHi, iLimitLo);
	ros::Time last_time = ros::Time::now();
	ros::Time now_time = ros::Time::now();
	
	while(!g_shutdown_request && ros::ok() && n.ok()){
		if (yb->isObjectDetected()){
			// normalization of x and y values
			//NOTE: point (0,0) is in the middle of the image
			cam_x = (float)yb->getObjX() / (captureSizeX/2);
			cam_y = (float)yb->getObjY() / (captureSizeY/2);
			cam_area = (float)yb->getObjArea();
			
			//out_lin_y = pid(cam_x);
			now_time = ros::Time::now();
			out_lin_y = pid.updatePid(cam_x, now_time - last_time);
			//ROS_INFO("PID max val = %.2f", pid.updatePid(1, now_time - last_time));
			last_time = now_time;
			
			limiter(&out_lin_y);
			
			yb->setTwistToZeroes();
			yb->m_twist.linear.y = out_lin_y * speed;
			ROS_INFO("cam_x = %.2f, out_y = %.2f", cam_x, out_lin_y);
		} else {
			yb->setTwistToZeroes();
			ROS_INFO("nothing");
		}
		
		yb->publishTwist(&pub_moveit);
		
		ros::spinOnce();
		r.sleep();
		
	}
	
	// Shutdown routine
	ROS_WARN("Stopping the motors . . .");
	yb->setTwistToZeroes();
	yb->publishTwist(&pub_moveit);
	ros::spinOnce();
	r.sleep();
	ros::shutdown();
	ROS_INFO("Bye");
	
	return 0;
}
