#include "bev_converter/bev_flow_estimator.h"

BEVFlowEstimator::BEVFlowEstimator(void)
: nh("~")
{
    nh.param("RANGE", RANGE, {10.0});
    nh.param("GRID_NUM", GRID_NUM, {50});
    nh.param("Hz", Hz, {100.0});
    nh.param("FLOW_IMAGE_SIZE", FLOW_IMAGE_SIZE, {50});
    nh.param("MANUAL_CROP_SIZE", MANUAL_CROP_SIZE, {5});
    nh.param("IS_DENSE", IS_SAVE_IMAGE, {false});
    nh.param("MAX_CORNERS", MAX_CORNERS, {20});
    nh.param("QUALITY_LEVEL", QUALITY_LEVEL, {0.05});
    nh.param("MIN_DISTANCE", MIN_DISTANCE, {5.0});
    nh.param("WIN_SIZE", WIN_SIZE, {3});
    nh.param("MAX_COUNT", MAX_COUNT, {30});
	nh.param("FRAME_ID", FRAME_ID, {"odom"});
	nh.param("CHILD_FRAME_ID", CHILD_FRAME_ID, {"base_footprint"});
    nh.param("STEP_BORDER", STEP_BORDER, {2});
    nh.param("IS_LOCAL", IS_LOCAL, {true});
    nh.param("IS_GAZEBO", IS_GAZEBO, {true});
    nh.param("USE_CMD_VEL", USE_CMD_VEL, {false});
    nh.param("CMD_VEL_TOPIC", CMD_VEL_TOPIC, {"/cmd_vel"});
    // nh.param("");
    nh.getParam("ROBOT_PARAM", ROBOT_PARAM);

    dynamic_grid_subscriber = nh.subscribe("/bev/grid", 10, &BEVFlowEstimator::dynamic_grid_callback, this);
    occupancy_grid_subscriber = nh.subscribe("/dynamic_cloud_detector/occupancy_grid", 10, &BEVFlowEstimator::occupancy_grid_callback, this);
	cmd_vel_subscriber = nh.subscribe(CMD_VEL_TOPIC, 10, &BEVFlowEstimator::cmd_vel_callback, this);
	odom_subscriber = nh.subscribe("/odom", 10, &BEVFlowEstimator::odom_callback, this);
	flow_image_publisher = nh.advertise<sensor_msgs::Image>("/bev/flow_image", 10);
	occupancy_image_publisher = nh.advertise<sensor_msgs::Image>("/bev/occupancy_image", 10);
}


void BEVFlowEstimator::executor(void)
{
    formatter();
    BEVImageGenerator bev_image_generator(RANGE, GRID_NUM, MANUAL_CROP_SIZE, ROBOT_PARAM, FRAME_ID, CHILD_FRAME_ID);
    bev_image_generator.formatter();

	ros::Rate r(Hz);
	while(ros::ok()){
        bev_image_generator.initializer();

		if((grid_callback_flag && occupancy_grid_callback_flag && cmd_vel_callback_flag) || (grid_callback_flag && occupancy_grid_callback_flag && odom_callback_flag)){
			cv::Mat cropped_current_grid_img = bev_image_generator.cropped_current_grid_img_generator(input_grid_img); // evry time newest
			cv::Mat cropped_occupancy_grid_img = bev_image_generator.cropped_current_grid_img_generator(input_occupancy_grid_img); // evry time newest
			/* cropped_occupancy_grid_img.convertTo(cropped_occupancy_grid_img, CV_8U, 255); */

			if(step % STEP_BORDER == STEP_BORDER - 1){
        		cv::Mat cropped_transformed_grid_img = bev_image_generator.cropped_transformed_grid_img_generator(pre_input_grid_img, current_position, current_yaw, pre_position, pre_yaw);
				if(cropped_transformed_grid_img.size() == cropped_current_grid_img.size()){
					cropped_transformed_grid_img.convertTo(cropped_transformed_grid_img, CV_8U, 255);
					cropped_current_grid_img.convertTo(cropped_current_grid_img, CV_8U, 255);
					
                    // cv::rotate(cropped_occupancy_grid_img, cropped_occupancy_grid_img, cv::ROTATE_180);
					cv::rotate(cropped_occupancy_grid_img, cropped_occupancy_grid_img, cv::ROTATE_90_COUNTERCLOCKWISE);
                    cv::Mat occupancy_img;
                    cropped_occupancy_grid_img.copyTo(occupancy_img);
                    sensor_msgs::ImagePtr occupancy_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", occupancy_img).toImageMsg();
                    occupancy_img_msg->header.seq = bev_seq;
                    occupancy_image_publisher.publish(occupancy_img_msg);

					// bev_flow = flow_estimator(cropped_transformed_grid_img, cropped_current_grid_img);
					bool flow_comp_flag = flow_estimator(cropped_transformed_grid_img, cropped_current_grid_img);
					std::cout << "flow comp!" << std::endl;

					if(flow_comp_flag){
							// cv::resize(bev_flow, bev_flow, cv::Size(FLOW_IMAGE_SIZE, FLOW_IMAGE_SIZE));
						if(IS_GAZEBO){
							cv::rotate(bev_flow, bev_flow, cv::ROTATE_90_COUNTERCLOCKWISE);
						}else{
							cv::rotate(bev_flow, bev_flow, cv::ROTATE_180);
							/* cv::rotate(bev_flow, bev_flow, cv::ROTATE_180); */
							/* cv::flip(bev_flow, bev_flow, 1); */
						}
						bev_flow.convertTo(bev_flow, CV_8U, 255);

						std::cout << "pub img" << std::endl;
						cv::Mat flow_img;
						bev_flow.copyTo(flow_img);
						// sensor_msgs::ImagePtr flow_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", flow_img).toImageMsg();
						sensor_msgs::ImagePtr flow_img_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", flow_img).toImageMsg();
						flow_img_msg->header.seq = bev_seq;
						flow_image_publisher.publish(flow_img_msg);
						// flow_img.release();
					}
					// cropped_current_grid_img.release();
					// cropped_transformed_grid_img.release();
					// bev_flow.release();
				}
			}
			
			grid_callback_flag = false;
			cmd_vel_callback_flag = false;
			odom_callback_flag = false;
			step++;
			std::cout << "step : " << step << std::endl;
			// pre_input_grid_img = input_grid_img;
		}

		r.sleep();
		ros::spinOnce();
	}
}



void BEVFlowEstimator::formatter(void)
{
	/* std::cout << "formatter" << std::endl; */

    dt = 1.0 / Hz;
    grid_size = RANGE / GRID_NUM;
	step = 0;

	odom_callback_flag = false;
	grid_callback_flag = false;
	occupancy_grid_callback_flag = false;
	cmd_vel_callback_flag = false;
}


void BEVFlowEstimator::initializer(void)
{
	/* std::cout << "initializer" << std::endl; */
	
	current_position = Eigen::Vector3d::Zero();
	pre_position = Eigen::Vector3d::Zero();
	current_yaw = 0.0;
	pre_yaw = 0.0;
}


void BEVFlowEstimator::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	if(USE_CMD_VEL){
		geometry_msgs::Twist cmd_vel = *msg;
		static bool first_flag = false;

		if(!first_flag){
			initializer();
			first_flag = true;
		}

		current_position.x() += cmd_vel.linear.x * dt * cos(current_position.z());
		current_position.y() += cmd_vel.linear.x * dt * sin(current_position.z());
		current_yaw += cmd_vel.angular.z * dt;

		while(current_yaw >= M_PI){
			current_yaw -= 2 * M_PI;
		}
		while(current_yaw <= -M_PI){
			current_yaw += 2 * M_PI;
		}

		if(step % STEP_BORDER == 0){
			initializer();
		}
	}
	cmd_vel_callback_flag = true;
}


void BEVFlowEstimator::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
	if(!USE_CMD_VEL){
		odom = *msg;
		static Eigen::Vector3d process_position;
		static double process_yaw;
		static bool first_flag = false;
		
		if(!first_flag){
			current_position = Eigen::Vector3d::Zero();
			pre_position = Eigen::Vector3d::Zero();
			process_position = Eigen::Vector3d::Zero();
			current_yaw = 0.0;
			pre_yaw = 0.0;
			process_yaw = 0.0;
			first_flag = true;
		}

		if(IS_LOCAL){
			current_position << odom.pose.pose.position.x - process_position.x(),
								odom.pose.pose.position.y - process_position.y(),
								odom.pose.pose.position.z - process_position.z();
			current_yaw = tf::getYaw(odom.pose.pose.orientation) - process_yaw;
			if(step % STEP_BORDER == 0){
				process_position << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
				process_yaw = tf::getYaw(odom.pose.pose.orientation);
			}
		}else{
			current_position << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
			current_yaw = tf::getYaw(odom.pose.pose.orientation);
			if(step % STEP_BORDER == 0){
				pre_position = current_position;
				pre_yaw = current_yaw;
			}
		}
	}
	odom_callback_flag = true;
}



void BEVFlowEstimator::occupancy_grid_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
	/* std::cout << "occupancy_grid_callback" << std::endl; */

	nav_msgs::OccupancyGrid occupancy_grid = *msg;
    bev_seq = occupancy_grid.header.seq;
    
    input_occupancy_grid_img = cv::Mat::zeros(cv::Size(occupancy_grid.info.width, occupancy_grid.info.height), CV_8UC1);

    for(unsigned int col = 0; col < occupancy_grid.info.height; col++){
        for(unsigned int row = 0; row < occupancy_grid.info.width; row++){
            unsigned int i = row + (occupancy_grid.info.height - col - 1) * occupancy_grid.info.width;
            input_occupancy_grid_img.at<unsigned char>(col, row) = occupancy_grid.data[i];
        }
    }

	occupancy_grid_callback_flag = true;
}


void BEVFlowEstimator::dynamic_grid_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
	/* std::cout << "dynamic_grid_callback" << std::endl; */

	nav_msgs::OccupancyGrid dynamic_grid = *msg;
    bev_seq = dynamic_grid.header.seq;
    
    input_grid_img = cv::Mat::zeros(cv::Size(dynamic_grid.info.width, dynamic_grid.info.height), CV_8UC1);

    for(unsigned int col = 0; col < dynamic_grid.info.height; col++){
        for(unsigned int row = 0; row < dynamic_grid.info.width; row++){
            unsigned int i = row + (dynamic_grid.info.height - col - 1) * dynamic_grid.info.width;
            input_grid_img.at<unsigned char>(col, row) = dynamic_grid.data[i];
        }
    }

    if(step % STEP_BORDER == 0){
        pre_input_grid_img = input_grid_img;
    }

	grid_callback_flag = true;
}


// cv::Mat BEVFlowEstimator::flow_estimator(cv::Mat pre_img, cv::Mat cur_img)
bool BEVFlowEstimator::flow_estimator(cv::Mat pre_img, cv::Mat cur_img)
{
	std::cout << "flow_estimator" << std::endl;
	flow_flag = false;

    cv::Mat flow_bgr;
	int img_size = GRID_NUM - 2 * MANUAL_CROP_SIZE;
	cv::Mat flow_x = cv::Mat::zeros(img_size, img_size, CV_32F);
	cv::Mat flow_y = cv::Mat::zeros(img_size, img_size, CV_32F);

	if(IS_DENSE){
		cv::Ptr<cv::superres::DenseOpticalFlowExt> optical_flow = cv::superres::createOptFlow_DualTVL1();
		/* std::cout << "pre_img.size() = " << pre_img.size() << std::endl; */
		/* std::cout << "cur_img.size() = " << cur_img.size() << std::endl; */
		optical_flow->calc(pre_img, cur_img, flow_x, flow_y);
		cv::Mat magnitude, angle;
		cv::cartToPolar(flow_x, flow_y, magnitude, angle, true);

		cv::Mat hsv_planes[3];
		hsv_planes[0] = angle;
		// cv::normalize(magnitude, magnitude, 0, 1, NORM_MINMAX);
		cv::normalize(magnitude, magnitude, 1.0, 0.0, cv::NORM_L1);
		hsv_planes[1] = magnitude;
		hsv_planes[2] = cv::Mat::ones(magnitude.size(), CV_32F);
		
		cv::Mat hsv;
		cv::merge(hsv_planes, 3, hsv);

		cv::cvtColor(hsv, flow_bgr, cv::COLOR_HSV2BGR);
	}else{
		std::vector<cv::Point2f> pre_corners;
		std::vector<cv::Point2f> cur_corners;
		cv::goodFeaturesToTrack(pre_img, pre_corners, MAX_CORNERS, QUALITY_LEVEL, MIN_DISTANCE);
		cv::goodFeaturesToTrack(cur_img, cur_corners, MAX_CORNERS, QUALITY_LEVEL, MIN_DISTANCE);
		/* std::cout << "pre_img.size() = " << pre_img.size() << std::endl; */
		/* std::cout << "cur_img.size() = " << cur_img.size() << std::endl; */
		/* std::cout << "pre_corners:" << pre_corners.size() << std::endl; */
		/* std::cout << "cur_corners:" << cur_corners.size() << std::endl; */
		if(pre_corners.size() > 0 && cur_corners.size() > 0){
			cv::cornerSubPix(pre_img, pre_corners, cv::Size(WIN_SIZE, WIN_SIZE), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, MAX_COUNT, QUALITY_LEVEL));
			cv::cornerSubPix(cur_img, cur_corners, cv::Size(WIN_SIZE, WIN_SIZE), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, MAX_COUNT, QUALITY_LEVEL));
			std::vector<uchar> features_found;
			std::vector<float> features_errors;

			size_t compare_features_num = 0;
			if(pre_corners.size() > cur_corners.size()){
				compare_features_num = cur_corners.size();
			}else if(pre_corners.size() <= cur_corners.size()){
				compare_features_num = pre_corners.size();
			}else{
			}

			cv::calcOpticalFlowPyrLK(pre_img, cur_img, pre_corners, cur_corners, features_found, features_errors);

			for(size_t i = 0; i < compare_features_num; i++){
				float flow_vector_x = (float)(cur_corners[i].x - pre_corners[i].x);
				float flow_vector_y = -(float)(cur_corners[i].y - pre_corners[i].y);
				if(cur_corners[i].x >= 0.0 && cur_corners[i].y >= 0.0){
					flow_x.at<float>(cur_corners[i].x, cur_corners[i].y) = flow_vector_x;
					flow_y.at<float>(cur_corners[i].x, cur_corners[i].y) = flow_vector_y;
				}
			}

			std::cout << "calculated flow" << std::endl;

			cv::Mat magnitude = cv::Mat::zeros(img_size, img_size, CV_32F);
			cv::Mat angle = cv::Mat::zeros(img_size, img_size, CV_32F);
			std::cout << "cartToPolar" << std::endl;
			cv::cartToPolar(flow_x, flow_y, magnitude, angle, true);

			// flow_x.release();
			// flow_y.release();

			cv::Mat hsv_planes[3];
			hsv_planes[0] = angle;
			cv::normalize(magnitude, magnitude, 0, 1, cv::NORM_MINMAX);
			/* cv::normalize(magnitude, magnitude, 1.0, 0.0, cv::NORM_L1); */
			hsv_planes[1] = magnitude;
			hsv_planes[2] = cv::Mat::ones(magnitude.size(), CV_32F);

			cv::Mat hsv;
			/* std::cout << "cv::merge" << std::endl; */
			cv::merge(hsv_planes, 3, hsv);
			/* std::cout << "cvtColor" << std::endl; */
			cv::cvtColor(hsv, flow_bgr, cv::COLOR_HSV2BGR);

			/* magnitude.release(); */
			/* angle.release(); */
			/* hsv_planes[0].release(); */
			/* hsv_planes[1].release(); */
			/* hsv_planes[2].release(); */
			/* hsv.release(); */
			/* features_found.clear(); */
			/* features_errors.clear(); */
		}
	}
	
	bool flow_comp = false;
	if(flow_bgr.size().width == img_size && flow_bgr.size().height == img_size){
		bev_flow = flow_bgr.clone();
		flow_comp = true;
		std::cout << "flow_bgr done" << std::endl;
	}else{
		std::cout << "flow_bgr miss" << std::endl;
	}
	// flow_bgr.release();

    /* return flow_bgr; */
	return flow_comp;
}




