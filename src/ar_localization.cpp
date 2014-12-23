/* Author: Akhil Kumar Nagariya 
   RRC IIIT Hyderabad */

#include<ros/ros.h>
#include<ros/package.h>
#include<ar_pose/ARMarkers.h>
#include<ar_pose/ARMarker.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<stdlib.h>
#include<stdio.h>
#include<string>
#include "ar_localization/marker_location.h"
#include<math.h>
class ArLocalization{
	public:
		ArLocalization();
	private:
		//num of markers
		int markernum_;
	        LocationData_T *location;
		
		//ros node handle
		ros::NodeHandle nh_;
		
		//subsciber to get marker pose
		ros::Subscriber marker_sub_;
		
		//camera tranformation publisher 
		ros::Publisher pose_pub_;

		//ar marker location (with respect to map) publisher
		tf::TransformBroadcaster bf_;
		tf::Transform *transform_;
		
		//file containing marker locations
		char marker_locations_list_[FILENAME_MAX];
		
		//function to be called when we see markers
		void markerCallback_(const ar_pose::ARMarkers::ConstPtr& msg);

		//time informations
		ros::Time then_;
		bool first_publish_;

		//check range, if observed marker is within range then return true
		bool checkRange(ar_pose::ARMarker marker);
};

ArLocalization::ArLocalization(){
	//subscriber to get markers postions with respect to camera frame
	first_publish_ = 1;
	marker_sub_ = nh_.subscribe ("ar_pose_markers", 100, &ArLocalization::markerCallback_, this);
	
	//publisher to initialize particle filter 
	pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 2);

	std::string path;
	std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);

	//location of markers in the map 
	
	ros::NodeHandle private_nh_;

	//get the parameters
	if(!private_nh_.getParam ("marker_location_list", path)){
		sprintf (marker_locations_list_, "%s/data/marker_locations", package_path.c_str());
	}
	else{
		sprintf (marker_locations_list_, "%s", path.c_str());
	}

	//read marker location from location file
	location = read_LocData (marker_locations_list_, &markernum_);

	//create frames for each of these marker location for visualization in Rviz 

	transform_ = (tf::Transform *) malloc (sizeof(tf::Transform) * markernum_);
	for (int i=0; i<markernum_; i++){	
		transform_[i].setOrigin( tf::Vector3 (location[i].marker_coord[0], location[i].marker_coord[1], location[i].marker_coord[2]));
		transform_[i].setRotation (tf::Quaternion(location[i].marker_quat[0], location[i].marker_quat[1], location[i].marker_quat[2], location[i].marker_quat[3]));	
	}
	ros::Rate r(100);

	//publish Transform 
	while (ros::ok()){
		for (int i=0; i<markernum_; i++){
			bf_.sendTransform( tf::StampedTransform( transform_[i], ros::Time::now(), "map", location[i].name));
		}
		ros::spinOnce();
		r.sleep();
	}
}

bool ArLocalization::checkRange(const ar_pose::ARMarker marker){
	double x,y,z;
	x = marker.pose.pose.position.x;
	y = marker.pose.pose.position.y;
	z = marker.pose.pose.position.z;
	return (1.5 >= sqrt(x*x + y*y + z*z));
}

void ArLocalization::markerCallback_(const ar_pose::ARMarkers::ConstPtr& msg){

	//publish marker pose only after 2 sec from last publish 
	if(first_publish_){
		then_ = ros::Time::now();
		first_publish_ = 0;
		return;
	}
	else{
		ros::Duration delt = ros::Time::now() - then_;
		if( delt.toSec() <=2 ){
			return;
		}
	}
	int num_markers = msg->markers.size();
	for(int i = 0; i< num_markers; i++){

		//check if observed marker is in range 
		if(!checkRange(msg->markers[i]))
				continue;
		tf::Transform transform;

		//transformation of marker with respect to camera_depth_optical_frame
		transform.setOrigin(tf::Vector3 (msg->markers[i].pose.pose.position.x,msg->markers[i].pose.pose.position.y,msg->markers[i].pose.pose.position.z));
		transform.setRotation(tf::Quaternion (msg->markers[i].pose.pose.orientation.x,msg->markers[i].pose.pose.orientation.y,msg->markers[i].pose.pose.orientation.z,msg->markers[i].pose.pose.orientation.w));
		tf::Transform transform2;
		
		//transformation of camera_depth_optical_frame with respect to camera_depth_frame
		transform2.setOrigin( tf::Vector3(0,-0.045,0));
		transform2.setRotation( tf::Quaternion( -0.5, 0.5, -0.5,0.5));
		
		//transformation of marker with respect to camera_depth_frame
		tf::Transform transform3 = transform2 * (transform);
		
		//transformation4 --> transformation of camera with respect to map frame
		tf::Transform transform4 = transform_[msg->markers[i].id]*(transform3.inverse());

		//publish the transformation 
	        bf_.sendTransform( tf::StampedTransform( transform4, ros::Time::now(), "map", "camera_frame"));

		//create pose msg to publish it on initialpose to intialize the partical filter 
		geometry_msgs::PoseWithCovarianceStamped robot_pose;
		robot_pose.header.stamp = ros::Time::now();
		robot_pose.header.frame_id = std::string("map");
		tf::Vector3 origin = transform4.getOrigin();
		tf::Vector3 quat = (transform4.getRotation()).getAxis();
		robot_pose.pose.pose.position.x = origin[0];
		robot_pose.pose.pose.position.y = origin[1];
		robot_pose.pose.pose.position.z = origin[2];
		double yaw_temp = tf::getYaw(transform4.getRotation());
		robot_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_temp);

		//create covariance matirx
		robot_pose.pose.covariance[0] = 0.007;
		robot_pose.pose.covariance[7] = 0.007;
		robot_pose.pose.covariance[35] = 0.01;
		then_ = ros::Time::now();
		pose_pub_.publish(robot_pose);
	}
}

int main(int argc,char **argv){
	ros::init(argc,argv,"ar_localiazation");
	ArLocalization ar_localization;
	ros::spin();
}
