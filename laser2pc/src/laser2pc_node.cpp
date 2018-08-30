#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class Laser2PC {
public:
	Laser2PC()
	{
		pc_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/pointcloud", 10);
		laser_sub_ = n_.subscribe("scan", 1, &Laser2PC::scanCallback, this);
	};

	~Laser2PC() {};


	/* Public Members */


	/* Public Methods */

	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
	  if(!listener_.waitForTransform(
	        scan_in->header.frame_id,
	        "/base_link",
	        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
	        ros::Duration(1.0))){
	     return;
	  }

	  
	  projector_.transformLaserScanToPointCloud("/base_link",*scan_in,
	          cloud,listener_);

	  // Do something with cloud.
	  pc_pub_.publish(cloud);
	}

private:
	/* Private Members */
	laser_geometry::LaserProjection projector_;
	tf::TransformListener listener_;
	sensor_msgs::PointCloud2 cloud;
	

	ros::NodeHandle n_;

	ros::Publisher pc_pub_;
	ros::Subscriber laser_sub_;

	/* Private Methods */

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser2pc_node");

	Laser2PC l2pc;

	ROS_INFO("laser2pc_node is started");

	ros::Rate r(100.0);

	while ( ros::ok() ) {
		ros::spinOnce();
		r.sleep();
	}
}