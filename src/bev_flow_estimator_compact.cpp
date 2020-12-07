#include "bev_converter/bev_flow_estimator_compact.h"

BEVFlowEstimator::BEVFlowEstimator(void)
: nh("~"), listener_(tf_buffer_)
{
	// ros loop rate
    nh.param("Hz", Hz, {100.0});
	// grid
    nh.param("WIDTH", WIDTH, {10.0});
    nh.param("GRID_NUM", GRID_NUM, {50});
    // flow
	nh.param("MANUAL_CROP_SIZE", MANUAL_CROP_SIZE, {5});
    nh.param("IS_SPARCE", IS_SPARCE, {true});
    nh.param("MAX_CORNERS", MAX_CORNERS, {20});
    nh.param("QUALITY_LEVEL", QUALITY_LEVEL, {0.05});
    nh.param("MIN_DISTANCE", MIN_DISTANCE, {5.0});
    nh.param("WIN_SIZE", WIN_SIZE, {3});
    nh.param("MAX_COUNT", MAX_COUNT, {30});
    nh.param("MIN_HUMAN_VELOCITY", MIN_HUMAN_VELOCITY, {0.0});
    nh.param("MAX_HUMAN_VELOCITY", MAX_HUMAN_VELOCITY, {1.0});
	// frame
	nh.param("FRAME_ID", FRAME_ID, {"odom"});
	nh.param("CHILD_FRAME_ID", CHILD_FRAME_ID, {"base_footprint"});
    // sim or bag
	nh.param("IS_GAZEBO", IS_GAZEBO, {true});

    // nh.getParam("ROBOT_PARAM", ROBOT_PARAM);

    dynamic_pointcloud_subscriber = nh.subscribe("/cloud/dynamic", 10, &BEVFlowEstimator::dynamic_pointcloud_callback, this);
	odom_subscriber = nh.subscribe("/odom", 10, &BEVFlowEstimator::odom_callback, this);

	flow_image_publisher = nh.advertise<sensor_msgs::Image>("/bev/flow_image", 10);
	previous_image_publisher = nh.advertise<sensor_msgs::Image>("/bev/previous_image", 10);
	current_image_publisher = nh.advertise<sensor_msgs::Image>("/bev/current_image", 10);
	flow_multiarray_x_publisher = nh.advertise<std_msgs::Float32MultiArray>("/bev/flow_array_x", 10);
	flow_multiarray_y_publisher = nh.advertise<std_msgs::Float32MultiArray>("/bev/flow_array_y", 10);
}


void BEVFlowEstimator::executor(void)
{
    formatter();

	ros::Rate r(Hz);
	while(ros::ok()){
		geometry_msgs::TransformStamped transform;
		try{
			transform = tf_buffer_.lookupTransform(FRAME_ID, CHILD_FRAME_ID, ros::Time(0));
			tf_listen_flag = true;
		}catch(tf2::TransformException& ex){
			std::cout << ex.what() << std::endl;
			ros::Duration(1.0).sleep();
		}

		// if(dynamic_pointcloud_callback_flag && odom_callback_flag){
		if(dynamic_pointcloud_callback_flag && tf_listen_flag){
        	initializer();

			cuurent_dynamic_image_generator(current_dynamic_image);
			image_cropper(current_dynamic_image, cropped_current_dynamic_image); // cropp to estimate flow between image and transformed_cropped_image
			if(is_first){
				previous_dynamic_image = current_dynamic_image.clone();
				previous_position = current_position;
				previous_yaw = current_yaw;
				is_first = false;
			}
			image_transformer(previous_dynamic_image, transformed_dynamic_image, previous_position, current_position, previous_yaw, current_yaw);
			// image_transformer(previous_dynamic_image, transformed_dynamic_image, transform);
			image_cropper(transformed_dynamic_image, cropped_transformed_dynamic_image);
			cv::Mat previous_feature_image = cropped_transformed_dynamic_image.clone();

			previous_feature_image.convertTo(previous_feature_image, CV_8U, 255);
			cv::Mat current_feature_image = cropped_current_dynamic_image.clone();
			current_feature_image.convertTo(current_feature_image, CV_8U, 255);

			cv::Mat flow_image32f = cv::Mat::zeros(cv::Size(cropped_grid_num, cropped_grid_num), CV_32F);
			bool is_flow = flow_estimator(previous_feature_image, current_feature_image, flow_image32f, flow_multiarray_x, flow_multiarray_y);

			cv::Mat flow_image8u = cv::Mat::zeros(cv::Size(cropped_grid_num, cropped_grid_num), CV_32F);
			// cv::Mat flow_image8u;
			if(is_flow){
				flow_image32f.convertTo(flow_image8u, CV_8U, 255);
			}
			// if(IS_GAZEBO){
			// 	cv::flip(flow_image8u, flow_image8u, 1);
			// }

			// sensor_msgs::ImagePtr flow_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", flow_image8uc3).toImageMsg();
			sensor_msgs::ImagePtr flow_image_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", flow_image8u).toImageMsg();
			flow_image_msg->header.seq = bev_seq;
			flow_image_publisher.publish(flow_image_msg);

			flow_multiarray_x_publisher.publish(flow_multiarray_x);
			flow_multiarray_y_publisher.publish(flow_multiarray_y);

			

			// for debug
			sensor_msgs::ImagePtr previous_image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", previous_feature_image).toImageMsg();
			previous_image_msg->header.seq = bev_seq;
			previous_image_publisher.publish(previous_image_msg);
			sensor_msgs::ImagePtr current_image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", current_feature_image).toImageMsg();
			current_image_msg->header.seq = bev_seq;
			current_image_publisher.publish(current_image_msg);



			step++;
			std::cout << "step : " << step << std::endl;
			previous_dynamic_image = current_dynamic_image.clone();
			previous_position = current_position;
			previous_yaw = current_yaw;
		}

		r.sleep();
		ros::spinOnce();
	}
}



void BEVFlowEstimator::formatter(void)
{
	/* std::cout << "formatter" << std::endl; */

    dt = 1.0 / Hz;
	step = 0;
    grid_size = WIDTH / (float)GRID_NUM;
	cropped_grid_num = GRID_NUM - 2 * MANUAL_CROP_SIZE;

    src_euqlid_3pts.points.resize(0);
    pt0.x = 0.0;
    pt0.y = 0.0;
    pt0.z = 0.0;
    pt1.x = 0.5 * WIDTH;
    pt1.y = 0.0;
    pt1.z = 0.0;
    pt2.x = 0.0;
    pt2.y = 0.5 * WIDTH;
    pt2.z = 0.0;
    src_euqlid_3pts.points.push_back(pt0);
    src_euqlid_3pts.points.push_back(pt1);
    src_euqlid_3pts.points.push_back(pt2);


	flow_multiarray_x.data.resize(0);
	flow_multiarray_y.data.resize(0);
	int multiarray_size = cropped_grid_num * cropped_grid_num;
	for(int i = 0; i < multiarray_size; i++){
		flow_multiarray_x.data.push_back(0.0);
		flow_multiarray_y.data.push_back(0.0);
	}
	flow_multiarray_x.layout.data_offset = (uint32_t)cropped_grid_num;
	flow_multiarray_y.layout.data_offset = (uint32_t)cropped_grid_num;
	flow_multiarray_x.layout.dim.resize(1);
	flow_multiarray_y.layout.dim.resize(1);
	flow_multiarray_x.layout.dim[0].size = 2;
	flow_multiarray_y.layout.dim[0].size = 2;

	current_dynamic_image = cv::Mat::zeros(cv::Size(GRID_NUM, GRID_NUM), CV_32FC1);
	previous_dynamic_image = cv::Mat::zeros(cv::Size(GRID_NUM, GRID_NUM), CV_32FC1);
	transformed_dynamic_image = cv::Mat::zeros(cv::Size(GRID_NUM, GRID_NUM), CV_32FC1);
	cropped_current_dynamic_image = cv::Mat::zeros(cv::Size(cropped_grid_num, cropped_grid_num), CV_32FC1);
	cropped_transformed_dynamic_image = cv::Mat::zeros(cv::Size(cropped_grid_num, cropped_grid_num), CV_32FC1);

	is_first = true;
	odom_callback_flag = false;
	dynamic_pointcloud_callback_flag = false;
}


void BEVFlowEstimator::initializer(void)
{
	/* std::cout << "initializer" << std::endl; */

	int multiarray_size = cropped_grid_num * cropped_grid_num;
	for(int i = 0; i < multiarray_size; i++){
		flow_multiarray_x.data[i] = 0.0;
		flow_multiarray_y.data[i] = 0.0;
	}

	cv::Mat zero_image = cv::Mat::zeros(cv::Size(GRID_NUM, GRID_NUM), CV_32FC1);
	cv::Mat zero_cropped_image = cv::Mat::zeros(cv::Size(GRID_NUM, GRID_NUM), CV_32FC1);
	current_dynamic_image = zero_image.clone();
	transformed_dynamic_image = zero_image.clone();
	cropped_current_dynamic_image = zero_cropped_image.clone();
	cropped_transformed_dynamic_image = zero_cropped_image.clone();

	dynamic_pointcloud_callback_flag = false;
	odom_callback_flag = false;
}


void BEVFlowEstimator::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
	odom = *msg;

	current_position << odom.pose.pose.position.x,
						odom.pose.pose.position.y,
						odom.pose.pose.position.z;
	current_yaw = tf::getYaw(odom.pose.pose.orientation);
	if(current_yaw < 0){
		current_yaw += 2 * M_PI;
	}

	odom_callback_flag = true;
}


void BEVFlowEstimator::dynamic_pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout << "dynamic_pointcloud_callback" << std::endl;
	sensor_msgs::PointCloud2 import_pointcloud_msg = *msg;
	pcl::fromROSMsg(import_pointcloud_msg, *pcl_import_pointcloud);

	dynamic_pointcloud_callback_flag = true;
}


void BEVFlowEstimator::cuurent_dynamic_image_generator(cv::Mat& dst_image)
{
	for(auto& pt : pcl_import_pointcloud->points){
		float robotcs_x = pt.x;
		float robotcs_y = pt.y;
		float hit_gridcs_rowd = 0.5 * WIDTH - robotcs_y;
		float hit_gridcs_cold = 0.5 * WIDTH - robotcs_x;
		int hit_row = (int)std::floor(hit_gridcs_rowd / grid_size);
		int hit_col = (int)std::floor(hit_gridcs_cold / grid_size);
		if((0 < hit_row && hit_row < GRID_NUM) && (0 < hit_col && hit_col < GRID_NUM)){
			dst_image.at<float>(hit_row, hit_col) = 1.0;
		}
	}
}


bool BEVFlowEstimator::flow_estimator(cv::Mat& previous_image, cv::Mat& current_image, cv::Mat& dst_image, std_msgs::Float32MultiArray& dst_array_x, std_msgs::Float32MultiArray& dst_array_y)
{
	std::cout << "flow_estimator" << std::endl;
	flow_flag = false;

	cv::Size image_size = cv::Size(cropped_grid_num, cropped_grid_num);
	const cv::Mat pre_img = previous_image.clone();
	const cv::Mat cur_img = current_image.clone();
	cv::Mat flow_x = cv::Mat::zeros(image_size, CV_32FC1);
	cv::Mat flow_y = cv::Mat::zeros(image_size, CV_32FC1);
    cv::Mat flow_bgr = cv::Mat::zeros(image_size, CV_32F);;

	if(IS_SPARCE){
		std::vector<cv::Point2f> pre_corners;
		std::vector<cv::Point2f> cur_corners;
		cv::goodFeaturesToTrack(pre_img, pre_corners, MAX_CORNERS, QUALITY_LEVEL, MIN_DISTANCE);
		cv::goodFeaturesToTrack(cur_img, cur_corners, MAX_CORNERS, QUALITY_LEVEL, MIN_DISTANCE);
		if(pre_corners.size() > 0 && cur_corners.size() > 0){
			cv::cornerSubPix(pre_img, pre_corners, cv::Size(WIN_SIZE, WIN_SIZE), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, MAX_COUNT, QUALITY_LEVEL));
			cv::cornerSubPix(cur_img, cur_corners, cv::Size(WIN_SIZE, WIN_SIZE), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, MAX_COUNT, QUALITY_LEVEL));
			std::vector<uchar> features_found;
			std::vector<float> features_errors;

			size_t compare_features_num = 0;
			if(pre_corners.size() > cur_corners.size()){
				compare_features_num = cur_corners.size();
				/* std::cout << "compare_features_num = " << compare_features_num << std::endl; */
			}else if(pre_corners.size() <= cur_corners.size()){
				compare_features_num = pre_corners.size();
				/* std::cout << "compare_features_num = " << compare_features_num << std::endl; */
			}else{
				// std::cout << "compare_features_num = " << compare_features_num << std::endl;
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

			// make flow image
			std::cout << "calculated flow" << std::endl;
			cv::Mat magnitude = cv::Mat::zeros(image_size, CV_32F);
			cv::Mat angle = cv::Mat::zeros(image_size, CV_32F);
			std::cout << "cartToPolar" << std::endl;
			cv::cartToPolar(flow_x, flow_y, magnitude, angle, true);
			cv::Mat hsv_planes[3];
			hsv_planes[0] = angle;
			cv::normalize(magnitude, magnitude, MIN_HUMAN_VELOCITY, MAX_HUMAN_VELOCITY, cv::NORM_MINMAX, CV_32F);
			hsv_planes[1] = magnitude;
			hsv_planes[2] = cv::Mat::ones(image_size, CV_32F);
			cv::Mat hsv;
			cv::merge(hsv_planes, 3, hsv);
			cv::cvtColor(hsv, flow_bgr, cv::COLOR_HSV2BGR);
		}
	}else{
		cv::Ptr<cv::superres::DenseOpticalFlowExt> optical_flow = cv::superres::createOptFlow_DualTVL1();
		/* std::cout << "pre_img.size() = " << pre_img.size() << std::endl; */
		/* std::cout << "cur_img.size() = " << cur_img.size() << std::endl; */
		optical_flow->calc(pre_img, cur_img, flow_x, flow_y);
		cv::Mat magnitude, angle;
		cv::cartToPolar(flow_x, flow_y, magnitude, angle, true);

		cv::Mat hsv_planes[3];
		hsv_planes[0] = angle;
		cv::normalize(magnitude, magnitude, MIN_HUMAN_VELOCITY, MAX_HUMAN_VELOCITY, cv::NORM_L1, CV_32F);
		hsv_planes[1] = magnitude;
		hsv_planes[2] = cv::Mat::ones(image_size, CV_32F);
		
		cv::Mat hsv;
		cv::merge(hsv_planes, 3, hsv);

		cv::cvtColor(hsv, flow_bgr, cv::COLOR_HSV2BGR);
	}
	
	bool is_flow = false;
	if(flow_bgr.size() == image_size){
		dst_image = flow_bgr.clone();
		is_flow = true;
		std::cout << "flow_bgr done" << std::endl;
	}else{
		std::cout << "flow_bgr miss" << std::endl;
	}
	
	// make flow multiarray
	flow_x = flow_x * grid_size;
	flow_y = flow_y * grid_size;
	int i = 0;
	for(int col = 0; col < cropped_grid_num; col++){
		for(int row = 0; row < cropped_grid_num; row++){
			dst_array_x.data[i] = flow_x.at<float>(row, col);
			dst_array_y.data[i] = flow_y.at<float>(row, col);
			i++;
		}
	}

	return is_flow;
}


void BEVFlowEstimator::image_transformer(cv::Mat& src_image, cv::Mat& dst_image, Eigen::Vector3d last_position, Eigen::Vector3d now_position, double last_yaw, double now_yaw)
{
    /* std::cout << "BEVFlowEstimator::image_transformer" << std::endl; */
    double dyaw = now_yaw - last_yaw;
	double cv_movement_x, cv_movement_y;
	cv_movement_x = (now_position.x() - last_position.x()) / grid_size;
	cv_movement_y = (now_position.y() - last_position.y()) / grid_size;

	cv::Mat_<double> p0(3, 1), p1(3, 1);
	cv::Mat affine = (cv::Mat_<double>(3, 3) << cos(-dyaw), -sin(-dyaw), -cv_movement_x,
												sin(-dyaw),  cos(-dyaw), -cv_movement_y,
												0         ,          0,               1);
	
	for(int col = 0; col < GRID_NUM - 1; col++){
		for(int row = 0; row < GRID_NUM - 1; row++){
			p1(0, 0) = row;
			p1(1, 0) = col;
			p0 = affine.inv() * p1;
			int x2 = (int)p0(0, 0);
			int y2 = (int)p0(1, 0);
			double xr = p0(0, 0) - x2;
			double yr = p0(1, 0) - y2;
			dst_image.at<float>(col, row) = (1 - xr)*(1 - yr) * src_image.at<float>(y2, x2)
										  + xr*(1 - yr)       * src_image.at<float>(y2, x2 + 1)
										  + (1 - xr)* yr      * src_image.at<float>(y2 + 1, x2)
										  + xr* yr            * src_image.at<float>(y2 + 1, x2 + 1);
		}
	}
}


void BEVFlowEstimator::image_cropper(cv::Mat& src_image, cv::Mat& dst_image)
{
    /* std::cout << "BEVFlowEstimator::image_cropper" << std::endl; */
	cv::Mat original_image = src_image.clone();
    cv::Rect roi(cv::Point(MANUAL_CROP_SIZE, MANUAL_CROP_SIZE), cv::Size(GRID_NUM - 2 * MANUAL_CROP_SIZE, GRID_NUM - 2 * MANUAL_CROP_SIZE));
    cv::Mat roi_image = original_image(roi);
	dst_image = roi_image.clone();
}





