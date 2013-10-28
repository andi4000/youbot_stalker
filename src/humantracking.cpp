#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "simpleMovingAverage.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

/**
 * //TODO:
 * - recognise gestures
 * - 
 * 
 * //DONE:
 * - adapt to changing user_ID --> obsolete (23.10.2013)
 * - read /tf topic values for human skeleton
 * - Ref: http://answers.ros.org/question/10515/openni_trackertransform-depth/
 * - Ref: http://answers.ros.org/question/35327/accurate-distance-to-kinect-with-openni/
 * - calculate distance --> use translation of torso_1
 * - publish distance and left/right values
 * - make consistent naming conventions for topics and capSizeX etc
 * 
 */
 
int main(int argc, char** argv){
	ros::init(argc, argv, "humantracking");
	
	ros::NodeHandle node;
	
	tf::TransformListener tfListener;
	
	ros::Publisher pub_humanDetected = node.advertise<std_msgs::Bool>("/youbotStalker/object_tracking/object_detected", 1000);
	ros::Publisher pub_camX = node.advertise<std_msgs::Int32>("/youbotStalker/object_tracking/cam_x_pos", 1000);
	ros::Publisher pub_camY = node.advertise<std_msgs::Int32>("/youbotStalker/object_tracking/cam_y_pos", 1000);
	ros::Publisher pub_distance = node.advertise<std_msgs::Float32>("/youbotStalker/object_tracking/distance", 1000);
	ros::Rate rate(40); // 10Hz for now
	
	int capSizeX = 1000;
	int capSizeY = 1000;
	
	//TODO: obsolete
	bool isHumanDetected = false;
	
	SimpleMovingAverage movingAverage(5);
	float avgValueOld = 0;
	float avgValueNew = 0;

	node.setParam("/youbotStalker/object_tracking/captureSizeX", capSizeX);
	node.setParam("/youbotStalker/object_tracking/captureSizeY", capSizeY);
	
	// Ref: http://www.ros.org/wiki/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
	while(node.ok()){
		tf::StampedTransform transform;
		try{
			//tfListener.lookupTransform("/openni_depth_frame", "torso_1", ros::Time(0), transform);
			tfListener.lookupTransform("/openni_depth_frame", "torso", ros::Time(0), transform);
			avgValueNew = movingAverage.getAverage(transform.getOrigin().x());
			
			//TODO: this is obsolete, replaced by bool value from openni2_user_selection
			// although this still works fine
			if (avgValueNew == avgValueOld){
				isHumanDetected = false;
				throw tf::TransformException("active user lost");
			} else {
				isHumanDetected = true;
			}
			
			ROS_INFO("torso: [%.4f, %.4f, %.4f]", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
			
			avgValueOld = avgValueNew;
		}catch(tf::TransformException ex){
			ROS_ERROR("%s", ex.what());
			isHumanDetected = false;
		}
		
		// ROS MESSAGE BEGIN
		std_msgs::Bool msg_humanDetected;
		std_msgs::Int32 msg_camX, msg_camY;
		std_msgs::Float32 msg_distance;
		
		msg_humanDetected.data = isHumanDetected;
		//TODO: why did i write it wrong like this?
		/**
		msg_camX.data = (int)(transform.getOrigin().y() * capSizeX * 2);
		msg_camY.data = (int)(transform.getOrigin().z() * capSizeY * 2);
		msg_distance.data = transform.getOrigin().x() * capSizeX;
		*/
		msg_camX.data = (int)(transform.getOrigin().x() * capSizeX * 2);
		msg_camY.data = (int)(transform.getOrigin().y() * capSizeY * 2);
		msg_distance.data = transform.getOrigin().z() * capSizeX;
		
		pub_humanDetected.publish(msg_humanDetected);
		pub_camX.publish(msg_camX);
		pub_camY.publish(msg_camY);
		pub_distance.publish(msg_distance);
		ros::spinOnce();
		// ROS MESSAGE END
		
		rate.sleep();
	}
	
	return 0;
}
