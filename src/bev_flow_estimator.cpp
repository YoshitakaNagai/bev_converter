#include "bev_converter/bev_flow_estimator.h"

BEVFlowEstimator::BEVFlowEstimator(void)
: nh("~")
{
    nh.param("RANGE", RANGE, {18.0});
    nh.param("GRID_NUM", GRID_NUM, {80});
    nh.param("Hz", Hz, {100.0});
    nh.param("FLOW_IMAGE_SIZE", FLOW_IMAGE_SIZE, {60});
    nh.param("SAVE_NUMBER", SAVE_NUMBER, {1});
    nh.param("MANUAL_CROP_SIZE", MANUAL_CROP_SIZE, {10});
    nh.param("PKG_PATH", PKG_PATH, {"/home/amsl/ros_catkin_ws/src/bev_converter/bev_img"});
    nh.param("IS_SAVE_IMAGE", IS_SAVE_IMAGE, {false});
    // nh.param("");
    nh.getParam("ROBOT_PARAM", ROBOT_PARAM);

    grid_subscriber = nh.subscribe("/bev/grid", 10, &BEVFlowEstimator::grid_callback, this);
	flow_image_publisher = nh.advertise<sensor_msgs::Image>("/bev/flow_image", 10);
    /* grid_subscriber = nh.subscribe("/dynamic_cloud_detector/occupancy_grid", 10, &BEVFlowEstimator::grid_callback, this); */
}


void BEVFlowEstimator::executor(void)
{
    formatter();
    BEVImageGenerator bev_image_generator(RANGE, GRID_NUM, MANUAL_CROP_SIZE, ROBOT_PARAM);
    bev_image_generator.formatter();
    int i = 0;

	ros::Rate r((int)Hz);
	while(ros::ok()){
        bev_image_generator.initializer();

		if(grid_callback_flag){
            initializer();
            const cv::Mat cropped_current_grid_img = bev_image_generator.cropped_current_grid_img_generator(input_grid_img);
            const cv::Mat cropped_transformed_grid_img = bev_image_generator.cropped_transformed_grid_img_generator(pre_input_grid_img);
            cv::Mat bev_flow = flow_estimator(cropped_transformed_grid_img, cropped_current_grid_img);

            cv::resize(bev_flow, bev_flow, cv::Size(FLOW_IMAGE_SIZE, FLOW_IMAGE_SIZE));
			cv::rotate(bev_flow, bev_flow, cv::ROTATE_90_COUNTERCLOCKWISE);
			bev_flow.convertTo(bev_flow, CV_8U, 255);

			/* std::cout << "imshow" << std::endl; */
			/* cv::namedWindow("bev_flow", CV_WINDOW_AUTOSIZE); */
			/* cv::imshow("bev_flow", bev_flow); */
			/* cv::waitKey(1); */
            
            if(IS_SAVE_IMAGE){
                std::vector<int> params(2);
                // .png
                const std::string folder_name = PKG_PATH + "/data_" + std::to_string(SAVE_NUMBER);
                params[0] = CV_IMWRITE_PNG_COMPRESSION;
                params[1] = 9;

                struct stat statBuf;
                if(stat(folder_name.c_str(), &statBuf) == 0){
                    std::cout << "exist dir" << std::endl;
                }else{
                    std::cout << "mkdir" << std::endl;
                    if(mkdir(folder_name.c_str(), 0755) != 0){
                        std::cout << "mkdir error" << std::endl;
                    }
                }
                /* cv::imwrite("/home/amsl/ros_catkin_ws/src/bev_converter/bev_img/data_" + std::to_string(SAVE_NUMBER) + "/" + "flow_" + std::to_string(i) + ".png", bev_flow, params); */
                cv::imwrite(folder_name + "/" + "flow_" + std::to_string(i) + ".png", bev_flow, params);
                /* std::cout << "SAVE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl; */
                i++;
            }

			std::cout << "pub img" << std::endl;
			cv::Mat flow_img;
			bev_flow.copyTo(flow_img);
			sensor_msgs::ImagePtr flow_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", flow_img).toImageMsg();
			flow_image_publisher.publish(flow_img_msg);
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
}


void BEVFlowEstimator::initializer(void)
{
	/* std::cout << "initializer" << std::endl; */

    first_flag = true;
    grid_callback_flag = false;
}


void BEVFlowEstimator::grid_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
	/* std::cout << "grid_callback" << std::endl; */

	nav_msgs::OccupancyGrid bev_grid = *msg;
    
    if(first_flag){
        pre_input_grid_img = input_grid_img;
    }
    
    input_grid_img = cv::Mat::zeros(cv::Size(bev_grid.info.width,bev_grid.info.height), CV_8UC1);

    for(unsigned int col = 0; col < bev_grid.info.height; col++){
        for(unsigned int row = 0; row < bev_grid.info.width; row++){
            unsigned int i = row + (bev_grid.info.height - col - 1) * bev_grid.info.width;
            int intensity = 205;
            if(0 <= bev_grid.data[i] && bev_grid.data[i] <= 100){
                intensity = round((float)(100.0 - bev_grid.data[i]) * 2.55);
            }
            input_grid_img.at<unsigned char>(col, row) = intensity;
        }
    }

    if(!first_flag){
        pre_input_grid_img = input_grid_img;
    }

    grid_callback_flag = true;
}


cv::Mat BEVFlowEstimator::flow_estimator(const cv::Mat &pre_img, const cv::Mat &cur_img)
{
	/* std::cout << "flow_estimator" << std::endl; */

	cv::Ptr<cv::superres::DenseOpticalFlowExt> optical_flow = cv::superres::createOptFlow_DualTVL1();
    cv::Mat flow_x, flow_y;
    optical_flow->calc(pre_img, cur_img, flow_x, flow_y);

    /*
    cv::Mat flow;
    cv::calcOpticalFlowFarneback(pre_img, cur_img, flow, 0.5, 10, 15, 3, 5, 1.1, 0);
    cv::Mat channels[2], flow_x, flow_y;
    cv::split(flow, channels);
    flow_x = channels[0];
    flow_y = channels[1];
    */

    cv::Mat magnitude, angle;
    cv::cartToPolar(flow_x, flow_y, magnitude, angle, true);
    cv::Mat hsv_planes[3];
    hsv_planes[0] = angle;
    // cv::normalize(magnitude, magnitude, 0, 1, NORM_MINMAX);
    cv::normalize(magnitude, magnitude, 0, 1, 32); // NORM_MINMAX    = 32 //!< flag
    // cv::normalize(magnitude, magnitude, 0, 1, NORM_L1);
    hsv_planes[1] = magnitude;
    hsv_planes[2] = cv::Mat::ones(magnitude.size(), CV_32F);
    
    cv::Mat hsv;
    cv::merge(hsv_planes, 3, hsv);

    cv::Mat flow_bgr;
	cv::cvtColor(hsv, flow_bgr, cv::COLOR_HSV2BGR);

    return flow_bgr;
}




