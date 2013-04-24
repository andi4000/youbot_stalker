#include "youBotIOHandler.h"
#include <signal.h>
#include "control_toolbox/pid.h"

// Ref: http://ibotics.ucsd.edu/trac/stingray/wiki/ROSNodeTutorialC%2B%2B
/**
 * //TODO:
 * - PID visual control
 * - Implementation of PID output scaling: either saturation block or dynamic range compression
 * - make safe shutdown routine more neat
 * - make getPIDParameters less nasty
 * 
 * //DONE:
 * - Put PID gains as arguments in the .launch file
 * - Try ROS::control_toolbox
 * - initialize PIDParam_t with constructor
 * 
 * 
 */
 
 volatile sig_atomic_t g_shutdown_request = 0;
 
 void youBotSafeShutdown(int sig){
	g_shutdown_request = 1;
	ROS_WARN("Shutdown signal received");
 }
 
 struct PIDParam_t{
	double speed;
	double Kp;
	double Ki;
	double Kd;
	double iLimitHi;
	double iLimitLo;
	// Constructor
	PIDParam_t(){
		speed = 0;
		Kp = 0;
		Ki = 0;
		Kd = 0;
		iLimitHi = 0;
		iLimitLo = 0;
	}
 };
 
 void getPIDParameters(char* paramPrefix, PIDParam_t* axis){
	double tmp;
	char paramName[33];
	
	strcpy(paramName, paramPrefix);
	strcat(paramName, "/Kp");
	if (ros::param::get(paramName, tmp))
		axis->Kp = tmp;
		
	strcpy(paramName, paramPrefix);
	strcat(paramName, "/Ki");
	if (ros::param::get(paramName, tmp))
		axis->Ki = tmp;
	
	strcpy(paramName, paramPrefix);
	strcat(paramName, "/Kd");
	if (ros::param::get(paramName, tmp))
		axis->Kd = tmp;
	
	strcpy(paramName, paramPrefix);
	strcat(paramName, "/iLimitHi");
	if (ros::param::get(paramName, tmp))
		axis->iLimitHi = tmp;
	
	strcpy(paramName, paramPrefix);
	strcat(paramName, "/iLimitLo");
	if (ros::param::get(paramName, tmp))
		axis->iLimitLo = tmp;
	
	strcpy(paramName, paramPrefix);
	strcat(paramName, "/speed");
	if (ros::param::get(paramName, tmp))
		axis->speed = tmp;
	
	ROS_WARN("%s configurations below:", paramPrefix);
	ROS_WARN("PID gains: [Kp, Ki, Kd] = [%.2f, %.2f, %.2f];", axis->Kp, axis->Ki, axis->Kd);
	ROS_WARN("Integral [hi,lo] limits = [%.2f, %.2f]", axis->iLimitHi, axis->iLimitLo);
	ROS_WARN("%d%% speed", (int)(axis->speed*100));

 }
 
 void limiter(float* value){
	float limitHi = 1;
	float limitLo = 0.01;
	
	// limiter so the robot will not blow up
	if (*value > limitHi)
		*value = 1;
	else if (*value < -limitHi)
		*value = -1;
	
	// limiter so the robot will not stutter
	if (*value < limitLo && *value > -limitLo)
		*value = 0;
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
	
	int captureSizeX, captureSizeY;
	ros::param::get("/object_tracking/captureSizeX", captureSizeX);
	ros::param::get("/object_tracking/captureSizeY", captureSizeY);

	ROS_WARN("PID gain values are taken from the .launch file!");

	PIDParam_t pidParamLinearY;
	PIDParam_t pidParamAngularZ;
	
	getPIDParameters("/youbotPID/linear_y", &pidParamLinearY);
	getPIDParameters("/youbotPID/angular_z", &pidParamAngularZ);
	
	control_toolbox::Pid pidLinearY, pidAngularZ;
	pidLinearY.initPid(pidParamLinearY.Kp, pidParamLinearY.Ki, pidParamLinearY.Kd, pidParamLinearY.iLimitHi, pidParamLinearY.iLimitLo);
	pidAngularZ.initPid(pidParamAngularZ.Kp, pidParamAngularZ.Ki, pidParamAngularZ.Kd, pidParamAngularZ.iLimitHi, pidParamAngularZ.iLimitLo);
	
	ros::Time last_time = ros::Time::now();
	ros::Time now_time = ros::Time::now();
	
	float out_lin_y, out_ang_z;
	
	while(!g_shutdown_request && ros::ok() && n.ok()){
		if (yb->isObjectDetected()){
			// normalization of x and y values
			//NOTE: point (0,0) is in the middle of the image
			cam_x = (float)yb->getObjX() / (captureSizeX/2);
			cam_y = (float)yb->getObjY() / (captureSizeY/2);
			cam_area = (float)yb->getObjArea();
			
			now_time = ros::Time::now();
			out_lin_y = pidLinearY.updatePid(cam_x, now_time - last_time);
			out_ang_z = pidAngularZ.updatePid(cam_x, now_time - last_time);
			last_time = now_time;
			
			limiter(&out_lin_y);
			limiter(&out_ang_z);
			
			yb->setTwistToZeroes();
			//yb->m_twist.linear.y = out_lin_y * pidParamLinearY.speed;
			yb->m_twist.angular.z = out_ang_z * pidParamAngularZ.speed;
			ROS_INFO("cam_x = %.2f, out_y = %.2f, out_z = %.2f", cam_x, out_lin_y, out_ang_z);
		} else {
			yb->setTwistToZeroes();
			ROS_INFO("nothing");
		}
		
		yb->publishTwist(&pub_moveit);
		
		ros::spinOnce();
		r.sleep();
		
	}
	
	// Shutdown routine
	ROS_WARN("Stopping the motors...");
	yb->setTwistToZeroes();
	yb->publishTwist(&pub_moveit);
	ros::spinOnce();
	r.sleep();
	ROS_INFO("Bye");
	ros::shutdown();
	
	return 0;
}
