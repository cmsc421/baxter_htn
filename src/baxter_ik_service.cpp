#include <ros/ros.h>
#include <block.h>
#include <baxter_ik.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "baxter_ik_service");
        BaxterIK baxter_ik = BaxterIK();
	return 0;
}
