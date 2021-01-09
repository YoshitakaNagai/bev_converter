#include "bev_converter/temporal_dynamic_bev_publisher.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "/bev_converter/temporal_dynamic_bev_publisher");

	TemporalDynamicBEV temporal_dynamic_bev;
	temporal_dynamic_bev.executor();

	return 0;
}
