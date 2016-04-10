#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <gazebo_msgs/GetModelState.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <cmath>
#include <algorithm>
#include <sstream>

#include <baxter_pickup_msgs/BlockArray.h>

class BlockTracker {

private:
  ros::NodeHandle nh_;
  ros::Publisher _pub_filtered;
  ros::Publisher _pub_pos;
  ros::Subscriber _sub_cloud;
  ros::Subscriber _sub_output;
  ros::Publisher block_pos;
  tf::TransformListener* tf_listener_;
  ros::NodeHandle* nh;

public:
   BlockTracker();
   void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg);
   void finished_cb(const std_msgs::String& msg);
};
