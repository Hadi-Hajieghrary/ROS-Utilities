
#include "ros/ros.h"
#include "ros/console.h"
#include "yaml-cpp/yaml.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include "sensor_msgs/LaserScan.h"

#include "utilities.hpp"


#include <sys/stat.h>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <memory>


int main(int argc, char** argv){

    ros::init(argc, argv, "map_validation_tool");
    ros::NodeHandle nh("~");
    
    // Get the name of the map file either as a parameter;
    // or as 1st input argument
    std::string file_address_yaml;
    std::string file_name_yaml;
    std::string file_address_pgm;
    std::string file_name_pgm;
    std::string map_file_name_input;
    if(!(nh.getParam("map_file_name", map_file_name_input))){
        map_file_name_input = std::string(argv[1]);
    }
    if ( *(map_file_name_input.rbegin()) =='/' ) map_file_name_input.erase(map_file_name_input.end()-1);
    //Check if the entered address points to a directory or a file
    struct stat s;
    if ( lstat(map_file_name_input.c_str(), &s) == 0 ) {
        if(S_ISDIR(s.st_mode)){
            // If the input argument is a folder address
            std::string file_name = map_file_name_input.substr(map_file_name_input.find_last_of('/')+1);
            file_address_pgm = map_file_name_input;
            file_address_yaml = map_file_name_input;
            file_name_pgm = file_name + ".pgm";
            file_name_yaml = file_name + ".yaml";
            
        }else{
            // If the input argument is a file address
            file_address_yaml = map_file_name_input.substr(0,map_file_name_input.find_last_of('/'));
            file_address_pgm = file_address_yaml;
            std::string file_name = map_file_name_input.substr(map_file_name_input.find_last_of('/')+1);
            if(file_name.substr(file_name.find_last_of('.')+1) == "yaml"){
                file_name_yaml = file_name;
                file_name_pgm = file_name.substr(0, map_file_name_input.find_last_of('.')+1) + "pgm";
            }else if(file_name.substr(file_name.find_last_of('.')+1) == "pgm"){
                file_name_pgm = file_name;
                file_name_yaml = file_name.substr(0, map_file_name_input.find_last_of('.')+1) + "yaml";
            }else{
                ROS_ERROR("%s is neither a YAML or a PGM file.", map_file_name_input.c_str());
                exit(-1);
            }
        }
    }
    // map_folder_address is the address of the folder containing yaml file
    // map_file_address_yaml is the address of the yaml file 
    // map_file_address_pgm is "likely" address of the pgm file. The name of the pgm file should be 
    // extracted from the YAML file.

    // Get the name of the ROS bag file which includes the 
    // Estimated Trajectory of the Robot;
    // either as a parameter or as 2nd input argument
    std::string traj_bag_file_name;
    if(!(nh.getParam("traj_bag_file_name", traj_bag_file_name))){
        traj_bag_file_name = std::string(argv[2]);
    }
    if ( *(traj_bag_file_name.rbegin()) =='/' ) traj_bag_file_name.erase(traj_bag_file_name.end()-1);
    if(traj_bag_file_name.substr(traj_bag_file_name.find_last_of('.')+1) != "bag"){
        ROS_ERROR("%s is not a ROS bag file.", traj_bag_file_name.c_str());
        exit(-1);
    }
    // Get the name of topic which caries the odometry 
    // estimations
    std::string odometry_topic_name;
    std::string laser_frame;
    std::string map_frame;
    if(!(nh.getParam("odometry_topic_name", odometry_topic_name))){
        if(!(nh.getParam("laser_frame", laser_frame))){
            laser_frame = "laser";
        }
        if(!(nh.getParam("map_frame", map_frame))){
            map_frame = "map";
        }
    }

    // Get the name of the ROS bag file which includes the 
    // Laser Scan of the Robot;
    // either as a parameter or as 3rd input argument
    std::string lscan_bag_file_name;
    if(!(nh.getParam("lscan_bag_file_name", lscan_bag_file_name))){
        lscan_bag_file_name = std::string(argv[3]);
    }
    if ( *(lscan_bag_file_name.rbegin()) =='/' ) lscan_bag_file_name.erase(lscan_bag_file_name.end()-1);
    if(lscan_bag_file_name.substr(lscan_bag_file_name.find_last_of('.')+1) != "bag"){
        ROS_ERROR("%s is not a ROS bag file.", lscan_bag_file_name.c_str());
        exit(-1);
    }
    // Get the name of topic which caries the laser scanners 
    // measurements
    std::string lscan_topic_name;
    if(!(nh.getParam("lscan_topic_name", lscan_topic_name))){
        ROS_ERROR("Laser Scanner Topic name cannot be empty");
        exit(-1);
    }
    // Get the name of base_link
    std::string base_frame_name;
    if(!(nh.getParam("base_frame", base_frame_name))){
       base_frame_name = "base_link";
    }
    // Get the name of base_link
    std::string map_frame_name;
    if(!(nh.getParam("map_frame", map_frame_name))){
       map_frame_name = "map";
    }

    // Cache the time stamps of the laser scan measurments
    std::vector<ros::Time> lscan_time_stamps;
    rosbag::Bag lscan_bag;
    lscan_bag.open(lscan_bag_file_name, rosbag::bagmode::Read);
    rosbag::View lscan_view(lscan_bag , rosbag::TopicQuery(lscan_topic_name));
    if(lscan_view.size()>0){
        lscan_time_stamps.reserve(lscan_view.size());
        for(const rosbag::MessageInstance msg: lscan_view){
            if(msg.getDataType() == "sensor_msgs/LaserScan"){
                sensor_msgs::LaserScan::ConstPtr lscan = msg.instantiate<sensor_msgs::LaserScan>();
                lscan_time_stamps.push_back(lscan->header.stamp);
            }
        }
        lscan_time_stamps.shrink_to_fit();
    }
    lscan_bag.close();
    ros::Duration max_cache_duration = ros::Duration(0);
    for(int i = 0; i<lscan_time_stamps.size()-1; i++){
        max_cache_duration = (lscan_time_stamps[i+1]-lscan_time_stamps[i]>max_cache_duration?
                                lscan_time_stamps[i+1]-lscan_time_stamps[i]:max_cache_duration);
    }
    rosbag::Bag traj_bag;
    max_cache_duration = ros::Duration(1000,0);
    std::unique_ptr<tf2::BufferCore> buffer(new tf2::BufferCore(max_cache_duration));
    traj_bag.open(traj_bag_file_name, rosbag::bagmode::Read);
    rosbag::View traj_view(traj_bag , rosbag::TopicQuery("/tf"));
    int ls = 10;
    if(traj_view.size()>0){
        for(const rosbag::MessageInstance msg: traj_view){
            if(msg.getDataType() == "tf2_msgs/TFMessage"){
                tf2_msgs::TFMessage::ConstPtr tf_msgs = msg.instantiate<tf2_msgs::TFMessage>();
                for(auto tf: tf_msgs->transforms){
                    buffer->setTransform(tf, msg.getCallerId());
                    if (buffer->canTransform(map_frame_name, base_frame_name, ros::Time(lscan_time_stamps[ls]))){
                        geometry_msgs::TransformStamped ts = buffer->lookupTransform(map_frame_name, base_frame_name, ros::Time(lscan_time_stamps[ls]));

                        ls++;
                    }
                }
            }
        }
    }
    traj_bag.close();    

MapInfo map_info;
readYAML(file_address_yaml, file_name_yaml, map_info);
nav_msgs::OccupancyGrid::Ptr map;
loadMap(file_address_pgm, map_info.image_name, map, map_info);
saveMap(map, file_address_pgm, "HH_OF", MapMode::RAW);

    return 0;
}