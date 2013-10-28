#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

#include "geometry_msgs/Twist.h"

class YouBotIOHandler
{
	public:
		YouBotIOHandler();
		~YouBotIOHandler();
		
		void callbackObjDetected(const std_msgs::Bool::ConstPtr& msg);
		void callbackPosX(const std_msgs::Int32::ConstPtr& msg);
		void callbackPosY(const std_msgs::Int32::ConstPtr& msg);
		void callbackArea(const std_msgs::Float32::ConstPtr& msg);
		
		//TODO: added 24.10.2013, needs to be reviewed
		// from gesture
		void callbackOffsetRobotX(const std_msgs::Float32::ConstPtr& msg);
		void callbackOffsetRobotY(const std_msgs::Float32::ConstPtr& msg);
		void callbackGestureState(const std_msgs::Int32::ConstPtr& msg);

		// also from gesture
		float getRobotOffsetX();
		float getRobotOffsetY();
		int getGestureState();

		bool isObjectDetected();
		int getObjX();
		int getObjY();
		float getObjDistance();
		
		
		geometry_msgs::Twist m_twist;
		void setTwistToZeroes();
		void publishTwist(ros::Publisher* pub);

	private:
		bool m_camObjDetected;
		int m_camPosX, m_camPosY;
		float m_camDistance;
		
		float m_robotOffsetX, m_robotOffsetY;
		int m_gestureState;
};
