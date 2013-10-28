#include "youBotIOHandler.h"

YouBotIOHandler::YouBotIOHandler()
{
	m_camObjDetected = false;
	m_camPosX = 0;
	m_camPosY = 0;
	m_camDistance = 0;
	m_robotOffsetX = 0;
	m_robotOffsetY = 0;
	m_gestureState = 0;
	
	setTwistToZeroes();
}

YouBotIOHandler::~YouBotIOHandler(){
	setTwistToZeroes();
	
}

void YouBotIOHandler::callbackObjDetected(const std_msgs::Bool::ConstPtr& msg){
	m_camObjDetected = msg->data;
}
void YouBotIOHandler::callbackPosX(const std_msgs::Int32::ConstPtr& msg){
	m_camPosX = msg->data;
}
void YouBotIOHandler::callbackPosY(const std_msgs::Int32::ConstPtr& msg){
	m_camPosY = msg->data;
}
void YouBotIOHandler::callbackArea(const std_msgs::Float32::ConstPtr& msg){
	m_camDistance = msg->data;
}

void YouBotIOHandler::callbackOffsetRobotX(const std_msgs::Float32::ConstPtr& msg){
	m_robotOffsetX = msg->data;
}

void YouBotIOHandler::callbackOffsetRobotY(const std_msgs::Float32::ConstPtr& msg){
	m_robotOffsetY = msg->data;
}

void YouBotIOHandler::callbackGestureState(const std_msgs::Int32::ConstPtr& msg){
	m_gestureState = msg->data;
}

bool YouBotIOHandler::isObjectDetected(){
	return m_camObjDetected;
}
int YouBotIOHandler::getObjX(){
	return m_camPosX;
}
int YouBotIOHandler::getObjY(){
	return m_camPosY;
}
float YouBotIOHandler::getObjDistance(){
	return m_camDistance;
}

float YouBotIOHandler::getRobotOffsetX(){
	return m_robotOffsetX;
}

float YouBotIOHandler::getRobotOffsetY(){
	return m_robotOffsetY;
}

int YouBotIOHandler::getGestureState(){
	return m_gestureState;
}

void YouBotIOHandler::setTwistToZeroes(){
	m_twist.linear.x = 0;
	m_twist.linear.y = 0;
	m_twist.linear.z = 0;
	m_twist.angular.x = 0;
	m_twist.angular.y = 0;
	m_twist.angular.z = 0;
}

void YouBotIOHandler::publishTwist(ros::Publisher* pub){
	pub->publish(m_twist);
}
