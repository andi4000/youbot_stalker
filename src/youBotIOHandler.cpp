#include "youBotIOHandler.h"

YouBotIOHandler::YouBotIOHandler()
{
	m_camObjDetected = false;
	m_camPosX = 0;
	m_camPosY = 0;
	m_camArea = 0;
	
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
	m_camArea = msg->data;
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
double YouBotIOHandler::getObjArea(){
	return m_camArea;
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
