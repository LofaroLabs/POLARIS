#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include <string>

ros::Publisher marker_pub;
int main(int argc, char **argv){
	ros::init(argc, argv, "robot_pose");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("robot_pose", 1000);
	tf::TransformListener listener;
	
	while(ros::ok()){
		tf::StampedTransform transform;
		try{
	        	listener.lookupTransform("/map", "/scanmatcher_frame",  ros::Time(0), transform);
       		}
       		catch (tf::TransformException ex){
        		 ROS_ERROR("%s",ex.what());
         		ros::Duration(1.0).sleep();
      		 }
		//listener.lookupTransform("map","scanmatcher_frame",ros::Time(0), transform);
		double x=transform.getOrigin().x();
		double y=transform.getOrigin().y();
		double z=tf::getYaw(transform.getRotation());
		std::string current_pose=std::to_string(x)+"||"+std::to_string(y)+"||"+std::to_string(z)+"||\n";
		printf("%s\n",current_pose.c_str());

		std_msgs::String msg;
		msg.data = current_pose;
		ROS_INFO("%s", msg.data.c_str());
		chatter_pub.publish(msg);
	}
	return 0;
}

	
