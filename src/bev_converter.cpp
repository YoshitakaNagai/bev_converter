#include "bev_converter/bev_converter.h"

BEVConverter::BEVConverter(void)
: nh("~")
{
    nh.param("RANGE", RANGE, {10.0});
    nh.param("GRID_NUM", GRID_NUM, {50});
    nh.param("Hz", Hz, {100.0});
    nh.param("CHILD_FRAME_ID", CHILD_FRAME_ID, {"base_link"});
    // nh.param("");

    // pc_subscriber = nh.subscribe("/velodyne_obstacles", 10, &BEVConverter::pc_callback, this);
    pc_subscriber = nh.subscribe("/cloud/dynamic", 10, &BEVConverter::pc_callback, this);
    odom_subscriber = nh.subscribe("/odom", 10, &BEVConverter::odom_callback, this);
    bev_grid_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/bev/grid", 10);
}


void BEVConverter::execution(void)
{
	std::cout << "formatter" << std::endl;
    formatter();

	ros::Rate r((int)Hz);
	while(ros::ok()){
		std::cout << "initializer" << std::endl;
        initializer();

		if(pc_callback_flag && odom_callback_flag){
			std::cout << "converter" << std::endl;
            converter();

    		bev_grid.header.stamp = ros::Time::now();
    		bev_grid.header.seq = pc_seq;
    		bev_grid.info.map_load_time = ros::Time::now();
    		bev_grid_publisher.publish(bev_grid);

			std::cout << bev_grid << std::endl;
		}
		tf_listen_flag = false;
		pc_callback_flag = false;
		odom_callback_flag = false;

		r.sleep();
		ros::spinOnce();
	}
}


void BEVConverter::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    sensor_msgs::PointCloud2 input_pc;
	pc_seq = input_pc.header.seq;

    input_pc = *msg;
	pcl::fromROSMsg(input_pc, *pcl_input_pc);
    // pcl_filtered_pc = pc_downsampling(pcl_input_pc);
    pc_callback_flag = true;
}


pcl::PointCloud<PointI>::Ptr BEVConverter::pc_downsampling(pcl::PointCloud<PointI>::Ptr pcl_input_pc_)
{
    CloudIPtr pcl_filtered_pc_ {new CloudI()};
    CloudIPtr pcl_filtered_pc__ {new CloudI()};
    CloudIPtr pcl_filtered_pc___ {new CloudI()};
    pcl::VoxelGrid<pcl::PointXYZI> sor;

    sor.setInputCloud(pcl_input_pc_);
    sor.setLeafSize(grid_size, grid_size, grid_size);
    sor.filter(*pcl_filtered_pc_);

	/* pcl::PassThrough<PointI> pass; */
	/* pass.setInputCloud(pcl_filtered_pc_); */
	/* pass.setFilterFieldName ("x"); */
	/* pass.setFilterLimits(-0.5 * WIDTH, 0.5 * WIDTH); */
	/* pass.setFilterLimitsNegative (true); */
	/* pass.filter(*pcl_filtered_pc__); */
	/* pass.setInputCloud(pcl_filtered_pc__); */
	/* pass.setFilterFieldName ("y"); */
	/* pass.setFilterLimits(-0.5 * HEIGHT, 0.5 * HEIGHT); */
	/* pass.setFilterLimitsNegative (true); */
	/* pass.filter(*pcl_filtered_pc___); */

    // return pcl_filtered_pc___;
    return pcl_filtered_pc_;
}


void BEVConverter::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
    odom = *msg;
    odom_callback_flag = true;
}


void BEVConverter::formatter(void)
{
	pc_seq = 0;
    bev_grid.header.seq = 0;
    bev_grid.header.frame_id = CHILD_FRAME_ID;
    bev_grid.info.resolution = (float)(RANGE / GRID_NUM);
    bev_grid.info.width = GRID_NUM;
    bev_grid.info.height = GRID_NUM;
    bev_grid.info.origin = odom.pose.pose;
    bev_grid.info.origin.position.x = odom.pose.pose.position.x - 0.5 * RANGE;
    bev_grid.info.origin.position.y = odom.pose.pose.position.y - 0.5 * RANGE;
    bev_grid.data.resize(GRID_NUM * GRID_NUM);

    grid_size = RANGE / GRID_NUM;
}


void BEVConverter::initializer(void)
{
    for(auto& data : bev_grid.data){
        data = (int)Free;
    }
}


void BEVConverter::converter(void)
{
    /* for(auto& pt : pcl_transformed_pc->points){ */
    /* for(auto& pt : pcl_filtered_pc->points){ */
    for(auto& pt : pcl_input_pc->points){
        int ix = floor((pt.x + 0.5 * RANGE) / grid_size);
        int iy = floor((pt.y + 0.5 * RANGE) / grid_size);
        int index = ix + iy * (RANGE / grid_size);
        /* int index = iy + ix * (RANGE / grid_size); */
        if((0 <= ix && ix < GRID_NUM) && (0 <= iy && iy < GRID_NUM)){
			/* std::cout << "[ix, iy] = [" << ix << ", " << iy << "]" << std::endl; */
            // bev_grid.data[index] = (int)Occupied;
            bev_grid.data[index] += 1;
        }
    }
}









