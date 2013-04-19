#include "youBotIOHandler.h"
#include <signal.h>

// Ref: http://ibotics.ucsd.edu/trac/stingray/wiki/ROSNodeTutorialC%2B%2B
/**
 * //TODO:
 * - Try ROS::control_toolbox
 * - PID visual control
 * - Put PID gains as arguments in the .launch file
 * - make safe shutdown routine more neat
 * 
 */
 
 volatile sig_atomic_t g_shutdown_request = 0;
 
 void youBotSafeShutdown(int sig){
	g_shutdown_request = 1;
	ROS_WARN("Shutdown signal received");
 }
 
float pid(float error){
	// PID gains
	float Kp = 2;
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
	float speed = 0.3;
	
	int captureSizeX, captureSizeY;
	n.getParam("/object_tracking/captureSizeX", captureSizeX);
	n.getParam("/object_tracking/captureSizeY", captureSizeY);
	
	while(!g_shutdown_request && ros::ok() && n.ok()){
		if (yb->isObjectDetected()){
			// normalization of x and y values
			//NOTE: point (0,0) is in the middle of the image
			cam_x = (float)yb->getObjX() / (captureSizeX/2);
			cam_y = (float)yb->getObjY() / (captureSizeY/2);
			cam_area = (float)yb->getObjArea();
			
			out_lin_y = pid(cam_x);
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
	ROS_INFO("Stopping the motors . . .");
	yb->setTwistToZeroes();
	yb->publishTwist(&pub_moveit);
	ros::spinOnce();
	r.sleep();
	ros::shutdown();
	ROS_INFO("Bye");
	
	return 0;
}
