#include "youBotIOHandler.h"
// Ref: http://ibotics.ucsd.edu/trac/stingray/wiki/ROSNodeTutorialC%2B%2B
/**
 * //TODO:
 * - safe shutdown (send all zero to cmd_vel)
 * - Ref: http://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/
 * - PID visual control
 * 
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "makeitmove");
	ros::NodeHandle n;
	
	YouBotIOHandler* yb = new YouBotIOHandler();
	
	ros::Subscriber sub_ObjDetect = n.subscribe("object_tracking/object_detected", 1000, &YouBotIOHandler::callbackObjDetected, yb);
	ros::Subscriber sub_PosX = n.subscribe("object_tracking/x_pos", 1000, &YouBotIOHandler::callbackPosX, yb);
	ros::Subscriber sub_PosY = n.subscribe("object_tracking/y_pos", 1000, &YouBotIOHandler::callbackPosY, yb);
	ros::Subscriber sub_area = n.subscribe("object_tracking/area", 1000, &YouBotIOHandler::callbackArea, yb);
	ros::Rate r(50);
	
	ros::Publisher pub_moveit = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	
	float cam_x, cam_y, cam_area;
	float speed = 0.3;
	
	while(n.ok() && ros::ok()){
		if (yb->isObjectDetected()){
			
			//TODO: convert camera x y values into youbot's angular speed
			//TODO: make this 320x240 values into variables or ROS Parameter
			cam_x = (float)yb->getObjX()/320;
			cam_y = (float)yb->getObjY()/240;
			cam_area = (float)yb->getObjArea();
			ROS_INFO("x = %.2f", cam_x);
			
			yb->m_twist.linear.x = 0;
			//yb->m_twist.linear.y = cam_x * speed;
			yb->m_twist.linear.y = 0;
			yb->m_twist.linear.z = 0;
			
			yb->m_twist.angular.x = 0;
			yb->m_twist.angular.y = 0;
			yb->m_twist.angular.z = -cam_x * speed;
		} else {
			yb->setTwistToZeroes();
			
			ROS_INFO("nothing");
		}
		
		yb->publishTwist(&pub_moveit);
		
		ros::spinOnce();
		r.sleep();
		
	}
	
	
	return 0;
}
