#include "bev_converter/raycast_publisher.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "/bev_converter/raycast_publisher");

	RaycastPublisher raycast_publisher;
	raycast_publisher.executor();

	return 0;
}
