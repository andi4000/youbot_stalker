#include "youbotController.h"

YoubotController::YoubotController()
{
	camObjDetected = false;
	camPosX = 0;
	camPosY = 0;
	camArea = 0;
}

YoubotController::~YoubotController(){
	
}

void YoubotController::callbackObjDetected(const std_msgs::Bool::ConstPtr& msg){
	camObjDetected = msg->data;
}
void YoubotController::callbackPosX(const std_msgs::Int32::ConstPtr& msg){
	camPosX = msg->data;
}
void YoubotController::callbackPosY(const std_msgs::Int32::ConstPtr& msg){
	camPosY = msg->data;
}
void YoubotController::callbackArea(const std_msgs::Float32::ConstPtr& msg){
	camArea = msg->data;
}

bool YoubotController::isObjectDetected(){
	return camObjDetected;
}
int YoubotController::getObjX(){
	return camPosX;
}
int YoubotController::getObjY(){
	return camPosY;
}
double YoubotController::getObjArea(){
	return camArea;
}
