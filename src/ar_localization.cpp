#include<ros/ros.h>
#include<ros/package.h>
#include<ar_pose/ARMarkers.h>
#include<ar_pose/ARMarker.h>
#include<geometry_msgs/Pose.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<stdlib.h>
#include<stdio.h>
#include<string>
#include "ar_localization/marker_location.h"
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
		tf::TransformBroadcaster tf_pub_;

		//ar marker location publisher
		tf::TransformBroadcaster bf_;
		tf::Transform *transform_;
		
		//file containing marker locations
		char marker_locations_list_[FILENAME_MAX];
		
		//function to be called when we see markers
		void markerCallback_(const ar_pose::ARMarkers::ConstPtr& msg);
};

ArLocalization::ArLocalization(){
	marker_sub_ = nh_.subscribe ("ar_pose_markers", 100, &ArLocalization::markerCallback_, this);
	tf_pub_ = tf::TransformBroadcaster ();
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

	//create frames for each of these marker location

	transform_ = (tf::Transform *) malloc (sizeof(tf::Transform) * markernum_);
	for (int i=0; i<markernum_; i++){	
		transform_[i].setOrigin( tf::Vector3 (location[i].marker_coord[0], location[i].marker_coord[1], location[i].marker_coord[2]));
		transform_[i].setRotation (tf::Quaternion(location[i].marker_quat[0], location[i].marker_quat[1], location[i].marker_quat[2], location[i].marker_quat[3]));	
	}
	ros::Rate r(100);
	while (ros::ok()){
		for (int i=0; i<markernum_; i++){
			bf_.sendTransform( tf::StampedTransform( transform_[i], ros::Time::now(), "map", location[i].name));
		}
		ros::spinOnce();
		r.sleep();
	}
}
void ArLocalization::markerCallback_(const ar_pose::ARMarkers::ConstPtr& msg){
	int num_markers = msg->markers.size();
	for(int i = 0; i< num_markers; i++){
		tf::Transform transform;
		transform.setOrigin(tf::Vector3 (msg->markers[i].pose.pose.position.x,msg->markers[i].pose.pose.position.y,msg->markers[i].pose.pose.position.z));
		transform.setRotation(tf::Quaternion (msg->markers[i].pose.pose.orientation.x,msg->markers[i].pose.pose.orientation.y,msg->markers[i].pose.pose.orientation.z,msg->markers[i].pose.pose.orientation.w));
		tf::StampedTransform transform2;
		tf::TransformListener listener;
	//	listener.waitForTransform("/camera_link","/camera_rgb_optical_frame",ros::Time(0),ros::Duration(0.1));
	//	listener.lookupTransform("/camera_link","/camera_rgb_optical_frame",ros::Time(0),transform2);
		transform2.setOrigin( tf::Vector3(0,-0.045,0));
		transform2.setRotation( tf::Quaternion( -0.5, 0.5, -0.5,0.5));
		tf::Transform transform3 = transform2 * (transform.inverse());
		tf::Transform transform4 = transform_[msg->markers[i].id]*(transform3.inverse());
	        bf_.sendTransform( tf::StampedTransform( transform4, ros::Time::now(), "map", "camera_frame"));

	}
}

int main(int argc,char **argv){
	ros::init(argc,argv,"ar_localiazation");
	ArLocalization ar_localization;
	ros::spin();
}
