#include<ros/ros.h>
#include<ros/package.h>
#include<ar_pose/ARMarkers.h>
#include<ar_pose/ARMarker.h>
#include<geometry_msgs/Pose.h>
#include<tf/transform_broadcaster.h>
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
		//ros node handle
		ros::NodeHandle nh_;
		//subsciber to get marker pose
		ros::Subscriber marker_sub_;
		//tranformation publisher 
		tf::TransformBroadcaster tf_pub_;
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
	LocationData_T *location;
	
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
}

void ArLocalization::markerCallback_(const ar_pose::ARMarkers::ConstPtr& msg){
	int num_markers = msg->markers.size();
	for(int i=0;i<num_markers;i++){
		std::cout<<i<<" "<<msg->markers[i].id<<" ";
	}
	std::cout<<std::endl;
}

int main(int argc,char **argv){
	ros::init(argc,argv,"ar_localiazation");
	ArLocalization ar_localization;
	ros::spin();
}
