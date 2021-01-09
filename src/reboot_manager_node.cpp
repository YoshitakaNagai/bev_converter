#include "bev_converter/reboot_manager.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv,"/bev/reboot_manager");

	RebootManager reboot_manager;
	reboot_manager.executor();
	
	return 0;
}
