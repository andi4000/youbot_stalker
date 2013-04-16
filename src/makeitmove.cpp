#include "youbotController.h"
// Ref: http://ibotics.ucsd.edu/trac/stingray/wiki/ROSNodeTutorialC%2B%2B

int main(int argc, char** argv)
{
	ros::init(argc, argv, "makeitmove");
	ROS_INFO("hello");
	ros::NodeHandle n;
	
	YoubotController* yb = new YoubotController();
	
	ros::Subscriber sub_ObjDetect = n.subscribe("object_tracking/object_detected", 1000, &YoubotController::callbackObjDetected, yb);
	ros::Subscriber sub_PosX = n.subscribe("object_tracking/x_pos", 1000, &YoubotController::callbackPosX, yb);
	ros::Subscriber sub_PosY = n.subscribe("object_tracking/y_pos", 1000, &YoubotController::callbackPosY, yb);
	ros::Subscriber sub_area = n.subscribe("object_tracking/area", 1000, &YoubotController::callbackArea, yb);
	ros::Rate r(50);
	
	ros::Publisher pub_moveit = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	
	geometry_msgs::Twist twist;
	float x, y;
	float speed = 0.3;
	
	while(n.ok() && ros::ok()){
		if (yb->isObjectDetected()){
			//ROS_INFO("(x, y) = (%d, %d)", yb->getObjX(), yb->getObjY());
			//ROS_INFO("area   = %.2f", yb->getObjArea());
			
			//TODO: convert camera x y values into youbot's angular speed
			x = (float)yb->getObjX()/320;
			ROS_INFO("x = %.2f", x);
			
			//TODO: calculate area then give value to move back and forth. or left and right

			twist.linear.x = 0;
			twist.linear.y = x * speed;
			twist.linear.z = 0;
			
			twist.angular.x = 0;
			twist.angular.y = 0;
			twist.angular.z = x * speed;
		} else {
			twist.linear.x = 0;
			twist.linear.y = 0;
			twist.linear.z = 0;
			
			twist.angular.x = 0;
			twist.angular.y = 0;
			twist.angular.z = 0;
			
			ROS_INFO("nothing");
		}
		
		pub_moveit.publish(twist);
		
		ros::spinOnce();
		r.sleep();
	}
	
	// have to make robot stop before quitting
	//TODO: is this the correct way?
	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;
	
	ROS_INFO("Quitting...");
	pub_moveit.publish(twist);
	ros::spinOnce();
	r.sleep();
	
	return 0;
}
