#include <ros/ros.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <baxter_pickup_msgs/BaxterIK.h>

#include <iostream>
#include <fstream>

#include <math.h>

struct coordinate
{
	float x;
	float y;
	float z;
};

class BaxterIK {

private:
  ros::NodeHandle nh_;

public: 
  BaxterIK();
  bool checkDistance(std::vector<coordinate> coordinates, coordinate coor);
  bool baxter_IK_left(baxter_pickup_msgs::BaxterIK::Request &req, baxter_pickup_msgs::BaxterIK::Response &res);
  bool baxter_IK_right(baxter_pickup_msgs::BaxterIK::Request &req, baxter_pickup_msgs::BaxterIK::Response &res);
  void spawnTable();
  void spawnBlock(std::string model_name, geometry_msgs::Pose model_pose);
  void spawnTrash();
  std::string file_prefix;
};


