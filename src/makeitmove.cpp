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
	
	
	while(n.ok()){
		if (yb->isObjectDetected()){
			ROS_INFO("(x, y) = (%d, %d)", yb->getObjX(), yb->getObjY());
			ROS_INFO("area   = %.2f", yb->getObjArea());
		} else {
			ROS_INFO("nothing");
		}		
		
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
