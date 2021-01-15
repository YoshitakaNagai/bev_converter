#ifndef __GROUNDTRUTH_H
#define __GROUNDTRUTH_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;

class GroundTruth
{
public:

	GroundTruth(void);

    typedef pcl::PointXYZI PointXYZIN;
    typedef pcl::PointCloud<PointXYZI> CloudXYZI;
    typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;
    typedef pcl::PointXYZ PointXYZ;
    typedef pcl::PointCloud<PointXYZ> CloudXYZ;
    typedef pcl::PointCloud<PointXYZ>::Ptr CloudXYZPtr;
    typedef pcl::PointCloud<PointN>::Ptr CloudNPtr;

    class GridCell
    {
    public:
        GridCell(void);

        bool people_existence = false;
        int people_hit[people_num] = {0};
        int people_id;

    private:
    };
    typedef std::vector<GridCell> OccupancyGridMap;

    input_cloud_to_occupancy_grid_map(const CloudXYZIPtr& cloud_ptr);
    transform_occupancy_grid_map(const Eigen::Vector2d& translation, double diff_yaw, OccupancyGridMap& map);
    is_valid_point(double x, double y);
    get_index_from_xy(const double x,const double y);
    get_x_index_from_index(const int index);
    get_y_index_from_index(const int index);
    get_x_from_index(const int index);
    get_y_from_index(const int index);



private:

    CloudXYZI pointcloud;
    double RESOLUTION;
    double WIDTH;people_num
    double WIDTH_2;
    int GRID_WIDTH;
    int GRID_WIDTH_2;
    int GRID_NUM;
    int PEOPLE_NUM;

}

#endif // __GROUNDTRUTH
