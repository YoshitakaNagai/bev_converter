#include "bev_converter/reboot_manager.h"

RebootManager::RebootManager(void)
	: nh("~")
{
	nh.param("Hz", Hz, {100.0});
	nh.param("BORDER_TIME", BORDER_TIME, {5.0});
	nh.param("COUNT_TIME", COUNT_TIME, {5});
	nh.param("LAUNCH_PATH", LAUNCH_PATH, {"/home/amsl/ros_catkin_ws/src/bev_converter/launch/gazebo_bev_flow_estimator.launch"});
	nh.param("BIN_PATH", BIN_PATH, {"/opt/ros/melodic/bin/roslaunch"});

	flow_image_subscriber = nh.subscribe("/bev/flow_image", 10, &RebootManager::flow_image_callback, this);
}


void RebootManager::executor(void)
{
	formatter();

	ros::Rate loop_rate(Hz);
	while(ros::ok())
	{
		if(flow_image_callback_flag){
			double current_time = ros::Time::now().toSec();
			double duration_time = std::fabs(current_time - callback_time);
			
			if(duration_time > BORDER_TIME){
				reboot();
			}
		}

		initializer();

		loop_rate.sleep();
		ros::spinOnce();
	}
}


void RebootManager::formatter(void)
{
	flow_image_callback_flag = false;
	is_first = true;
	READ = 0;
	WRITE = 1;
}


void RebootManager::initializer(void)
{
	flow_image_callback_flag = false;
	is_first = false;
}


void RebootManager::reboot(void)
{
	kill(pid_gazebo_bev_flow_estimator, SIGINT);
	for(int i = 0; i <= COUNT_TIME; i++){
		std::cout << "Please waite for " << COUNT_TIME - i << "[sec] for rebooting..." << std::endl;
		sleep(1.0);
	}
	std::cout << "RESTART!" << std::endl;
	pid_gazebo_bev_flow_estimator = roslauncher(LAUNCH_PATH, BIN_PATH);
}


void RebootManager::flow_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	flow_image_callback_flag = true;
	callback_time = ros::Time::now().toSec();
}


pid_t RebootManager::roslauncher(const std::string& launch_path, const std::string& bin_path)
{
	int *infp, *outfp;
	int p_stdin[2], p_stdout[2];
	const char *char_launch_path = launch_path.c_str();
	const char *char_bin_path = bin_path.c_str();
	pid_t pid;

	if(pipe(p_stdin) != 0 || pipe(p_stdout) != 0){
		return -1;
	}

	pid = fork();

	if(pid < 0){
		return pid;
	}else if(pid == 0){
		close(p_stdin[WRITE]);
		dup2(p_stdin[READ], READ);
		close(p_stdout[READ]);
		dup2(p_stdout[WRITE], WRITE);
		execl(char_bin_path, "--wait", char_launch_path, NULL);
		perror("execl");
		exit(1);
	}

	if(infp == NULL){
		close(p_stdin[WRITE]);
	}else{
		*infp = p_stdin[WRITE];
	}

	if(outfp == NULL){
		close(p_stdout[READ]);
	}else{
		*outfp = p_stdout[READ];
	}

	return pid;
}



