#include "bev_converter/raycast_publisher.h"


RaycastPublisher::RaycastPublisher(void)
: nh("~")
{
    nh.param("RANGE", RANGE, {10.0});
    nh.param("GRID_NUM", GRID_NUM, {50});
    nh.param("Hz", Hz, {100.0});
	nh.param("FRAME_ID", FRAME_ID, {"base_scan"});
    // nh.param("");

	laserscan_subscriber = nh.subscribe("/scan", 10, &RaycastPublisher::laserscan_callback, this);
	raycast_image_publisher = nh.advertise<sensor_msgs::Image>("/bev/raycast_image", 10);
}


void RaycastPublisher::executor(void)
{
	is_first = true;

	ros::Rate r(Hz);
	while(ros::ok()){
		if(is_first){
			formatter();
		}

		if(){

			}
		}

		r.sleep();
		ros::spinOnce();
	}
}



void RaycastPublisher::formatter(void)
{
	/* std::cout << "formatter" << std::endl; */
	laserscan_callback_flag = false;
	is_first = false;
    dt = 1.0 / Hz;
    grid_size = RANGE / (double)GRID_NUM;
	angle_resolution = 2 * M_PI / ray_num;
	lidar.ranges.resize(0);
	Precast precast;
	precast.angle_id = 0;
	precast.range = 0.0;
	std::vector<Precast> row_gird;
	for(int ix = 0; ix < GRID_NUM; ix++){
		row_grid.push_back(precast)
	}
	for(int iy = 0; iy < GRID_NUM; iy++){
		precast_grid.push_back(precast);
	}
}


void RaycastPublisher::initializer(void)
{
	/* std::cout << "initializer" << std::endl; */
}

void RaycastPublisher::laserscan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
	lidar = *msg;
	ray_num = lidar.ranges.size();
	for(int i = 0; i < ray_num; i++){
		if(isfinite(lidar.range[i]) == false){
			lidar.ranges[i] = 0.0;
			lidar.intensities[i] = 0.0;
		}
	}

	laserscan_callback_flag = true;
}


void RaycastPublisher::precast(void)
{
	for(int ix = 0; ix < GRID_NUM; ix++){
		for(int iy = 0; iy < GRID_NUM; iy++){
			float relative_x = ix * grid_size + 0.5 * RANGE;
			float relative_y = iy * grid_size + 0.5 * RANGE;
			float relative_anglei_pi_pi = atan2(relative_y, relative_x);
			float relative_angle_0_2pi = 0.0;
			if(relative_anglei_pi_pi < 0){
				relative_angle_0_2pi += 2 * M_PI;
			}else{
				relative_angle_0_2pi = relative_anglei_pi_pi;
			}
			precast_grid[ix][iy].angle_id = get_angle_id(relative_angle_0_2pi);
			precast_grid[ix][iy].range = get_distance(relative_x, relative_y);
		}
	}
}


int RaycastPublisher::get_angle_id(float& relative_angle_0_2pi)
{
	int return_angle_id = 0;
	for(int angle_id = 0; angle_id < ray_num - 1; angle_id++){
		float radian = angle_id * angle_resolution;
		float next_radian = (angle_id + 1) * angle_resolution;
		if(radian <= relative_angle_0_2pi && relative_angle_0_2pi < next_radian){
			return_angle_id = angle_id;
			break;
		}
	}
	return return_angle_id;
}


float RaycastPublisher::get_distance(float& x, float& y)
{
	return sqrt(x * x + y * y);
}







