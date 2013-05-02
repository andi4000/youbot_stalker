#include "ros/ros.h"
#include "tf/transform_listener.h"

/**
 * //TODO:
 * - read /tf topic values for human skeleton
 * - Ref: http://answers.ros.org/question/10515/openni_trackertransform-depth/
 * - Ref: http://answers.ros.org/question/35327/accurate-distance-to-kinect-with-openni/
 * - calculate distance --> use translation of torso_1
 * - publish distance and left/right values
 * 
 * - recognise gestures
 * 
 */
 
int main(int argc, char** argv){
	ros::init(argc, argv, "humantracking");
	
	ros::NodeHandle node;
	
	tf::TransformListener tfListener;
	ros::Rate rate(10); // 10Hz for now
	
	// Ref: http://www.ros.org/wiki/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
	while(node.ok()){
		tf::StampedTransform transform;
		try{
			tfListener.lookupTransform("/openni_depth_frame", "torso_1", ros::Time(0), transform);
		}catch(tf::TransformException ex){
			ROS_ERROR("%s", ex.what());
		}
		
		ROS_INFO("Translation of torso_1: [%.4f, %.4f, %.4f]", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
		
		
		rate.sleep();
	}
	
	return 0;
}
