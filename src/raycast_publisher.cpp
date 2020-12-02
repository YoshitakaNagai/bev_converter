#include "bev_converter/raycast_publisher.h"


RaycastPublisher::RaycastPublisher(void)
: nh("~")
{
    nh.param("WIDTH", WIDTH, {8.0});
    nh.param("GRID_NUM", GRID_NUM, {40});
    nh.param("Hz", Hz, {100.0});
	nh.param("RAY_NUM", RAY_NUM, {360});
	nh.param("IS_USE_VELODYNE", IS_USE_VELODYNE, {true});
    // nh.param("");

	laserscan_subscriber = nh.subscribe("/scan", 10, &RaycastPublisher::laserscan_callback, this);
	pointcloud_subscriber = nh.subscribe("/velodyne_obstacles", 10, &RaycastPublisher::pointcloud_callback, this);
	raycast_image_publisher = nh.advertise<sensor_msgs::Image>("/bev/raycast_image", 10);
}


void RaycastPublisher::executor(void)
{
	is_first = true;
	formatter();
	initializer();
	ros::Rate r(Hz);
	while(ros::ok()){
		if(laserscan_callback_flag && pointcloud_callback_flag){
			ray_listup(IS_USE_VELODYNE);
			cv::Mat raycast_image32f = cv::Mat::zeros(GRID_NUM, GRID_NUM, CV_32FC1);
			raycast(raycast_image32f);
			cv::Mat raycast_image8u;
			raycast_image32f.convertTo(raycast_image8u, CV_8UC1, 255);
			cv::rotate(raycast_image8u, raycast_image8u, cv::ROTATE_90_CLOCKWISE);
			cv::flip(raycast_image8u, raycast_image8u, 1);
			// cv::rotate(raycast_image8u, raycast_image8u, cv::ROTATE_90_COUNTERCLOCKWISE);
            // cv::rotate(raycast_image8u, raycast_image8u, cv::ROTATE_180);

			sensor_msgs::ImagePtr raycast_image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", raycast_image8u).toImageMsg();
			raycast_image_msg->header = header_initializer(IS_USE_VELODYNE);
			raycast_image_publisher.publish(raycast_image_msg);

			std::cout << "velodyne_header.header.seq   : " << velodyne_header.seq << std::endl;
			std::cout << "raycast_image_msg.header.seq : " << raycast_image_msg->header.seq << std::endl;
			
			initializer();
		}

		r.sleep();
		ros::spinOnce();
	}
}


void RaycastPublisher::formatter(void)
{
	// std::cout << "formatter" << std::endl;
    dt = 1.0 / Hz;
    grid_size = WIDTH / (double)GRID_NUM;
	angle_resolution = 2 * M_PI / RAY_NUM;
	
	Cast init_cast;
	init_cast.angle_id = 0;
	init_cast.range = 2 * WIDTH;
	std::vector<Cast> row_grid;
	for(int row = 0; row < GRID_NUM; row++){
		row_grid.push_back(init_cast);
	}
	for(int col = 0; col < GRID_NUM; col++){
		precast_grid.push_back(row_grid);
	}
	for(int i = 0; i < RAY_NUM; i++){
		ray_list.push_back(init_cast);
	}

	precast();

	std::vector<bool> is_hit;
	for(int row = 0; row < GRID_NUM; row++){
		is_hit.push_back(false);
	}
	for(int col = 0; col < GRID_NUM; col++){
		is_hit_grid.push_back(is_hit);
	}
}


void RaycastPublisher::initializer(void)
{
	// std::cout << "initializer" << std::endl;
	if(IS_USE_VELODYNE){
		laserscan_callback_flag = true;
		pointcloud_callback_flag = false;
	}else{
		laserscan_callback_flag = false;
		pointcloud_callback_flag = true;
	}
	input_points->points.clear();
	for(auto& ray : ray_list){
		ray.range = 2 * WIDTH;
	}
	for(int col = 0; col < GRID_NUM; col++){
		for(int row = 0; row < GRID_NUM; row++){
			is_hit_grid[row][col] = false;
		}
	}
}

void RaycastPublisher::laserscan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
	lasers = *msg;

	laserscan_callback_flag = true;
}


void RaycastPublisher::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	sensor_msgs::PointCloud2 points_msg;
	points_msg = *msg;
	pcl::fromROSMsg(points_msg, *input_points);
	velodyne_header = points_msg.header;

	pointcloud_callback_flag = true;
}


void RaycastPublisher::precast(void)
{
	// std::cout << "precast_grid" << std::endl;
	for(int col = 0; col < GRID_NUM; col++){
		for(int row = 0; row < GRID_NUM; row++){
			// float relative_robotcs_x = -(iy * grid_size - 0.5 * WIDTH);
			float relative_robotcs_x = 0.5 * WIDTH - col * grid_size;
			float relative_robotcs_y = 0.5 * WIDTH - row * grid_size;
			float relative_robotcs_angle = atan2(relative_robotcs_y, relative_robotcs_x);
			precast_grid[row][col].angle_id = get_angle_id(relative_robotcs_angle);
			precast_grid[row][col].range = get_distance(relative_robotcs_x, relative_robotcs_y);
		}
	}
}


int RaycastPublisher::get_angle_id(float& angle)
{
	if(angle < 0){
		angle += 2 * M_PI;
	}

	int return_angle_id = 0;
	for(int angle_id = 0; angle_id < RAY_NUM; angle_id++){
		float radian = angle_id * angle_resolution;
		float next_radian = (angle_id + 1) * angle_resolution;
		if(radian <= angle && angle < next_radian){
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


void RaycastPublisher::ray_listup(bool is_use_velodyne)
{
	// std::cout << "ray_listup" << std::endl;
	if(is_use_velodyne){
		for(auto& pt : input_points->points){
			float pt_angle = atan2(pt.y, pt.x);
			int pt_angle_id = get_angle_id(pt_angle);
			float pt_range = get_distance(pt.x, pt.y);
			if(ray_list[pt_angle_id].range > pt_range){
				ray_list[pt_angle_id].range = pt_range;
			}

			float ray_robotcs_x = pt.x;
			float ray_robotcs_y = pt.y;
			float hit_gridcs_rowd = 0.5 * WIDTH - ray_robotcs_y;
			float hit_gridcs_cold = 0.5 * WIDTH - ray_robotcs_x;
			int hit_row = std::floor(hit_gridcs_rowd / grid_size);
			int hit_col = std::floor(hit_gridcs_cold / grid_size);
			if((0 < hit_row && hit_row < GRID_NUM) && (0 < hit_col && hit_col < GRID_NUM)){
				is_hit_grid[hit_row][hit_col] = true;
			}
		}
	}else{
		int lasers_size = lasers.ranges.size();
		for(int i = 0; i < lasers_size; i++){
			if(isfinite(lasers.ranges[i])){
				float laser_angle = lasers.angle_increment * i;
				int laser_angle_id = get_angle_id(laser_angle);
				ray_list[laser_angle_id].range = lasers.ranges[i];

				float ray_robotcs_x = lasers.ranges[i] * cos(laser_angle);
				float ray_robotcs_y = lasers.ranges[i] * sin(laser_angle);
				float hit_gridcs_rowd = 0.5 * WIDTH - ray_robotcs_y;
				float hit_gridcs_cold = 0.5 * WIDTH - ray_robotcs_x;
				int hit_row = std::floor(hit_gridcs_rowd / grid_size);
				int hit_col = std::floor(hit_gridcs_cold / grid_size);
				is_hit_grid[hit_row][hit_col] = true;
			}
		}
	}
}


void RaycastPublisher::raycast(cv::Mat& image32f)
{
	// std::cout << "raycast" << std::endl;
	for(int col = 0; col < GRID_NUM; col++){
		for(int row = 0; row < GRID_NUM; row++){
			int pcg_angle_id = precast_grid[row][col].angle_id;
			image32f.at<float>(row, col) = 0.5; // Initialize Unknown
			if(precast_grid[row][col].range < ray_list[pcg_angle_id].range){
				image32f.at<float>(row, col) = 0.0; // Unoccupied
			}else{
				image32f.at<float>(row, col) = 0.5; // Unknown
			}
			if(is_hit_grid[row][col]){
				image32f.at<float>(row, col) = 1.0; // Occupied
			}
		}
	}
}


std_msgs::Header RaycastPublisher::header_initializer(bool is_use_velodyne)
{
	std_msgs::Header return_header;
	if(is_use_velodyne){
		return_header = velodyne_header;
	}else{
		return_header = lasers.header;
	}

	return return_header;
}
