#include "bev_converter/groundtruth.h"

GroundTruth::GroundTruth(void)
:local_nh("~")
{
    local_nh.param("RESOLUTION", RESOLUTION, {});
    local_nh.param("WIDTH", WIDTH, {});
    local_nh.param("WIDTH_2", WIDTH_2,{});
    local_nh.param("GRID_WIDTH", GRID_WIDTH, {});
    local_nh.param("GRID_WIDTH", GRID_WIDTH_2, {});
    local_nh.param("GRID_NUM", GRID_NUM, {});
    local_nh.param("PEOPLE_NUM", PEOPLE_NUM, {});

}

void GroundTruth::executor(void)
{
    formatter();

    ros::Rate r(Hz);
	while(ros::ok()){
        
        OccupancyGridMap new_ogm(GRID_NUM);
        OccupancyGridMap old_ogm(GRID_NUM);

        input_cloud_to_occupancy_grid_map(cloud_ptr);
        generate_flow_picture(new_ogm, old_ogm);





		if(){


        
		}

		r.sleep();
		ros::spinOnce();
    }


}

void GroundTruth::formatter(void)
{
	/* std::cout << "formatter" << std::endl; */

    dt = 1.0 / Hz;
    grid_size = RANGE / GRID_NUM;
}


void GroundTruth::input_cloud_to_occupancy_grid_map(const CloudXYZIPtr& cloud_ptr)
{
    
    int cloud_size = cloud_ptr->points.size();

    for(int i=0;i<cloud_size;i++){
        auto p = cloud_ptr->points[i];
        int id = (int)cloud_ptr->intensty -1;  //id:0 start intensity:1 start

        if(!is_valid_point(p.x, p.y)){
            continue;
        }

        int index = get_index_from_xy(p.x, p.y);
        occupancy_grid_map[index].peple_hit[id] += 1;

    }

    for(int i=0;i<GRID_NUM;i++){
        
        int max = occupancy_grid_map[i].people_hit[0];
        int id = 0;
        
        for(int j=1;j<PEOPELE_NUM;j++){
            if(max < occupancy_grid_map[i].people_hit[j]){
                max = occupancy_grid_map[i].people_hit[j];
                id = j;
            }
        }

        if(max > 0){
            occupancy_grid_map[i].people_id = id;
            occupancy_grid_map[i].people_existence = true;
        }
    
    }

}

bool GroundTruth::is_valid_point(double x, double y)
{
    int index = get_index_from_xy(x, y);
    if(x < -WIDTH_2 || x > WIDTH_2 || y < -WIDTH_2 || y > WIDTH_2){
        return false;
    }else if(index < 0 || GRID_NUM <= index){
        return false;
    }else{
        return true;
    }
}

void GroundTruth::generate_flow_picture(OccupancyGridMap& old_map,OccupancyGridMap& new_map)
{
    //taiou grid tansaku
    //map_posi hiukizan


}
void GroundTruth::transform_occupancy_grid_map(const Eigen::Vector2d& translation, double diff_yaw, OccupancyGridMap& map)
{
    const double dx = translation(0);
    const double dy = translation(1);
    const double c_yaw = cos(diff_yaw);
    const double s_yaw = sin(diff_yaw);

    std::cout << "scrolling\n\tdx: " << dx << ", dy: " << dy << ", cos(theta): " << c_yaw << ", sin(theta)" << s_yaw << std::endl;

    const double dx_grid = dx / RESOLUTION;
    const double dy_grid = dy / RESOLUTION;
    std::cout << "dx_grid: " << dx_grid << std::endl;
    std::cout << "dy_grid: " << dy_grid << std::endl;

    Eigen::Matrix3d affine;
    affine << c_yaw, -s_yaw, dx_grid,
              s_yaw,  c_yaw, dy_grid,
                  0,      0,        1;
    std::cout << "forward affine:\n" << affine << std::endl;
    Eigen::Matrix3d affine_inverse = affine.inverse();
    std::cout << "reversed affine:\n" << affine_inverse << std::endl;

    OccupancyGridMap ogm(GRID_NUM);
    int show_i = GRID_NUM * 0.5 + GRID_WIDTH - 1;
    for(int i=0;i<GRID_NUM;i++){
        if(i == show_i)
            std::cout << "i: " << i << std::endl;
        double x_i = get_x_index_from_index(i) - GRID_WIDTH_2;
        double y_i = get_y_index_from_index(i) - GRID_WIDTH_2;
        Eigen::Vector3d ogm_i(x_i, y_i, 1);
        if(i == show_i)
            std::cout << "ogm_i.transpose(): " << ogm_i.transpose() << std::endl;
        if(i == show_i)
            std::cout << x_i * RESOLUTION << ", " << y_i * RESOLUTION << std::endl;
        Eigen::Vector3d map_i = affine_inverse * ogm_i;
        if(i == show_i)
            std::cout << "map_i.transpose(): " << map_i.transpose() << std::endl;
        if(i == show_i)
            std::cout << map_i(0) * RESOLUTION << ", " << map_i(1) * RESOLUTION << std::endl;

        // bilinear interpolation
        int x_0 = std::floor(map_i(0));
        int x_1 = x_0 + 1;
        int y_0 = std::floor(map_i(1));
        int y_1 = y_0 + 1;
        if(i == show_i)
            std::cout << x_0 << ", " << x_1 << ", " << y_0 << ", " << y_1 << std::endl;
        if(x_0 <= -GRID_WIDTH_2 || GRID_WIDTH_2 <= x_1){
            continue;
        }
        if(y_0 <= -GRID_WIDTH_2 || GRID_WIDTH_2 <= y_1){
            continue;
        }
        int index_0_0 = (y_0 + GRID_WIDTH_2) * GRID_WIDTH + x_0 + GRID_WIDTH_2;
        int index_0_1 = (y_1 + GRID_WIDTH_2) * GRID_WIDTH + x_0 + GRID_WIDTH_2;
        int index_1_0 = (y_0 + GRID_WIDTH_2) * GRID_WIDTH + x_1 + GRID_WIDTH_2;
        int index_1_1 = (y_1 + GRID_WIDTH_2) * GRID_WIDTH + x_1 + GRID_WIDTH_2;
        if(i == show_i)
            std::cout << index_0_0 << ", " << index_0_1 << ", " << index_1_0 << ", " << index_1_1 << std::endl;

        Eigen::Vector2d y_vec(y_1 - map_i(1), map_i(1) - y_0);
        Eigen::Vector2d x_vec(x_1 - map_i(0), map_i(0) - x_0);
        Eigen::Matrix2d value_mat;
        // value_mat << map[index_0_0].get_log_odds(), map[index_1_0].get_log_odds(),
        //              map[index_0_1].get_log_odds(), map[index_1_1].get_log_odds();
        value_mat << map[index_0_0].get_occupancy(), map[index_1_0].get_occupancy(),
                     map[index_0_1].get_occupancy(), map[index_1_1].get_occupancy();

        double ogm_value = y_vec.transpose() * value_mat * x_vec;
        ogm[i].log_odds = std::log(ogm_value / (1 - ogm_value));
        if(i == show_i)
            std::cout << "\033[31m" << ogm_value << "\033[0m" << std::endl;
    }
    map.clear();
    map = ogm;
}


int GroundTruth::get_index_from_xy(const double x,const double y)
{
	int _x = floor(x / RESOLUTION + 0.5) + GRID_WIDTH_2;
	int _y = floor(y / RESOLUTION + 0.5) + GRID_WIDTH_2;
	return _y * GRID_WIDTH_2 + _x;
}

int GroundTruth::get_x_index_from_index(const int index)
{
    return index % GRID_WIDTH;
}

int GroundTruth::get_y_index_from_index(const int index)
{
    return index / GRID_WIDTH;
}

double GroundTruth::get_x_from_index(const int index)
{
    return (get_x_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}

double GroundTruth::get_y_from_index(const int index)
{
    return (get_y_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}
