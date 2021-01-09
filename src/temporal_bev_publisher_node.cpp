#include "bev_converter/temporal_bev_publisher.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "/bev_converter/temporal_bev_publisher");

	TemporalBEV temporal_bev;
	temporal_bev.executor();

	return 0;
}
