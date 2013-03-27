#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

#include "geometry_msgs/Twist.h"

class YoubotController
{
	public:
		YoubotController();
		~YoubotController();
		
		void callbackObjDetected(const std_msgs::Bool::ConstPtr& msg);
		void callbackPosX(const std_msgs::Int32::ConstPtr& msg);
		void callbackPosY(const std_msgs::Int32::ConstPtr& msg);
		void callbackArea(const std_msgs::Float32::ConstPtr& msg);
		
		bool isObjectDetected();
		int getObjX();
		int getObjY();
		double getObjArea();
		
	private:
		bool camObjDetected;
		int camPosX, camPosY;
		double camArea;
};
