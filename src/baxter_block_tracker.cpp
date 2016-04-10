#include <ros/ros.h>
#include <block.h>
#include <baxter_ik.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "baxter_block_tracker");
	BlockTracker baxter_block_tracker = BlockTracker();	
	return 0;
}
