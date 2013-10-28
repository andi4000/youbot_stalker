#include "youBotIOHandler.h"
#include "simpleMovingAverage.h"
#include <signal.h>
#include "control_toolbox/pid.h"

//TODO: this is copied from youbot_gesture, make it modular!
#define GESTURE_INACTIVE 0
#define GESTURE_ACTIVE_ONE_HAND 1
#define GESTURE_ACTIVE_TWO_HANDS 2

// Ref: http://ibotics.ucsd.edu/trac/stingray/wiki/ROSNodeTutorialC%2B%2B
/**
 * //TODO:
 * - when gesture active, turn off angular z control --> needs testing
 * 
 * - PID visual control
 * - fix derivative kick (sudden output spike due to aggresive derivative), by calculating own dError/dt
 * - Ref: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
 * - Implementation of PID output scaling: either saturation block or dynamic range compression
 * - make safe shutdown routine more neat
 * - make getPIDParameters less nasty --> use Pid::initParam(const std::string& prefix) --> but but, we have extra parameters such as speed and offset
 * 
 * 
 * //DONE:
 * - Put PID gains as arguments in the .launch file
 * - Try ROS::control_toolbox
 * - initialize PIDParam_t with constructor
 * - IDEA: do Simple Moving Average from the error_dot with last 5 inputs with vector
 * 
 * 
 */
 
#define SETPOINT_DISTANCE 1750

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
	strcat(paramName, "/p");
	if (ros::param::get(paramName, tmp))
		axis->Kp = tmp;
		
	strcpy(paramName, paramPrefix);
	strcat(paramName, "/i");
	if (ros::param::get(paramName, tmp))
		axis->Ki = tmp;
	
	strcpy(paramName, paramPrefix);
	strcat(paramName, "/d");
	if (ros::param::get(paramName, tmp))
		axis->Kd = tmp;
	
	strcpy(paramName, paramPrefix);
	strcat(paramName, "/i_clamp");
	if (ros::param::get(paramName, tmp)){
		axis->iLimitHi = tmp;
		axis->iLimitLo = -tmp;
	}
		
	strcpy(paramName, paramPrefix);
	strcat(paramName, "/speed");
	if (ros::param::get(paramName, tmp))
		axis->speed = tmp;
	
	/**	
	strcpy(paramName, paramPrefix);
	strcat(paramName, "/offset");
	if (); 
	*/
	
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
	
	ros::Subscriber sub_ObjDetect = n.subscribe("/youbotStalker/object_tracking/object_detected", 1000, &YouBotIOHandler::callbackObjDetected, yb);
	ros::Subscriber sub_PosX = n.subscribe("/youbotStalker/object_tracking/cam_x_pos", 1000, &YouBotIOHandler::callbackPosX, yb);
	ros::Subscriber sub_PosY = n.subscribe("/youbotStalker/object_tracking/cam_y_pos", 1000, &YouBotIOHandler::callbackPosY, yb);
	ros::Subscriber sub_area = n.subscribe("/youbotStalker/object_tracking/distance", 1000, &YouBotIOHandler::callbackArea, yb);
	
	// from gesture
	ros::Subscriber sub_robotOffsetX = n.subscribe("/youbotStalker/gesture_processor/offset_linear_x", 1000, &YouBotIOHandler::callbackOffsetRobotX, yb);
	ros::Subscriber sub_robotOffsetY = n.subscribe("/youbotStalker/gesture_processor/offset_linear_y", 1000, &YouBotIOHandler::callbackOffsetRobotY, yb);
	ros::Subscriber sub_gestureState = n.subscribe("/youbotStalker/gesture_processor/state", 1000, &YoubotIOHandler::callbackGestureState, yb);
	
	ros::Rate r(50);
	
	ros::Publisher pub_moveit = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	
	float cam_x, cam_y, cam_distance;
	
	int captureSizeX, captureSizeY;
	ros::param::get("/youbotStalker/object_tracking/captureSizeX", captureSizeX);
	ros::param::get("/youbotStalker/object_tracking/captureSizeY", captureSizeY);
	
	ROS_WARN("PID gain values are taken from the .launch file!");

	PIDParam_t pidParamLinearX;
	PIDParam_t pidParamLinearY;
	PIDParam_t pidParamAngularZ;
	
	getPIDParameters("/youbotStalker/PID/linear_x", &pidParamLinearX);
	getPIDParameters("/youbotStalker/PID/linear_y", &pidParamLinearY);
	getPIDParameters("/youbotStalker/PID/angular_z", &pidParamAngularZ);
	control_toolbox::Pid pidLinearX, pidLinearY, pidAngularZ;
	
	pidLinearX.initPid(pidParamLinearX.Kp, pidParamLinearX.Ki, pidParamLinearX.Kd, pidParamLinearX.iLimitHi, pidParamLinearX.iLimitLo);
	pidLinearY.initPid(pidParamLinearY.Kp, pidParamLinearY.Ki, pidParamLinearY.Kd, pidParamLinearY.iLimitHi, pidParamLinearY.iLimitLo);
	pidAngularZ.initPid(pidParamAngularZ.Kp, pidParamAngularZ.Ki, pidParamAngularZ.Kd, pidParamAngularZ.iLimitHi, pidParamAngularZ.iLimitLo);
	
	ros::Time last_time = ros::Time::now();
	ros::Time now_time = ros::Time::now();
	ros::Duration dt;
	
	float rob_z_last_error = 0;
	float rob_z_now_error = 0;
	float rob_z_error_dot = 0;
	// hack to surpress derivative kick
	float rob_z_error_dot_avg = 0;

	float rob_y_last_error = 0;
	float rob_y_now_error = 0;
	float rob_y_error_dot = 0;
	// hack to surpress derivative kick
	float rob_y_error_dot_avg = 0;
	
	float out_lin_y = 0;
	float out_lin_x = 0;
	float out_ang_z = 0;
		
	SimpleMovingAverage movingAverageX;
	SimpleMovingAverage movingAverageY;
	
	while(!g_shutdown_request && ros::ok() && n.ok()){
		if (yb->isObjectDetected()){
			// normalization of x and y values
			//NOTE: point (0,0) is in the middle of the image
			cam_x = (float)yb->getObjX() / (captureSizeX);
			//cam_y = (float)yb->getObjY() / (captureSizeY);
			cam_distance = (float)yb->getObjDistance();
			
			// PID begin
			now_time = ros::Time::now();
			dt = now_time - last_time;
			
			rob_z_now_error = cam_x;
			rob_z_error_dot = (rob_z_now_error - rob_z_last_error) / dt.toSec();
			rob_z_error_dot_avg = movingAverageX.getAverageExceptZero(rob_z_error_dot);
			
			rob_y_now_error = yb->getRobotOffsetY();
			rob_y_error_dot = (rob_y_now_error - rob_y_last_error) / dt.toSec();
			rob_y_error_dot_avg = movingAverageY.getAverageExceptZero(rob_y_error_dot);
			
			//TODO: needs testing
			out_lin_x = - pidLinearX.updatePid((cam_distance - SETPOINT_DISTANCE + 800*yb->getRobotOffsetX())/1000, dt);
			//TODO: this might be not the final version, since there's a plan to incorporate angular z gesture
			if (yb->getGestureState() == GESTURE_ACTIVE_ONE_HAND){
				out_lin_y = - pidLinearY.updatePid(yb->getRobotOffsetY(), rob_y_error_dot_avg, dt);
			} else {
				out_ang_z = - pidAngularZ.updatePid(cam_x, rob_z_error_dot_avg, dt);
			}
			
			last_time = now_time;
			rob_z_last_error = rob_z_now_error;
			rob_y_last_error = rob_y_now_error;
			// PID end
			
			limiter(&out_lin_x);
			limiter(&out_lin_y);
			limiter(&out_ang_z);
			//yb->setTwistToZeroes();
			yb->m_twist.linear.x = out_lin_x * pidParamLinearX.speed;
			yb->m_twist.linear.y = out_lin_y * pidParamLinearY.speed;
			yb->m_twist.angular.z = out_ang_z * pidParamAngularZ.speed;
			//ROS_INFO("cam_x = %.2f, out_y = %.2f, out_z = %.4f, err_dot = %.3f, err_dot_avg = %.3f", cam_x, out_lin_y, out_ang_z, error_dot, error_dot_avg);
			ROS_INFO("x = %.2f, dist = %.2f, out_x = %.2f, out_y = %.2f, out_z = %.2f, offset_x = %.2f, offset_y = %.2f", cam_x, cam_distance, out_lin_x, out_lin_y, out_ang_z, yb->getRobotOffsetX(), yb->getRobotOffsetY());
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
