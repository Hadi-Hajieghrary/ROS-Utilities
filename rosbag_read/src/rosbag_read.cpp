#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

#include <cstring>
#include <fstream>
#include <map>
#include <algorithm>
#include <array>
#include <sstream>
#include <string>


// Check if the file is a ROS bag file
std::pair<std::string, bool> isValidFileName(char* file_name, size_t n){
    const char ext[]{"bag"};
    char buf[4]{0};
    std::strncpy(buf, file_name+(n-3),3);
        if (strcmp(buf,ext)==0){
            return std::make_pair<std::string, bool>(std::string(file_name, file_name+n), true);
        }else{
            return std::make_pair<std::string, bool>(std::string(""), false);
        }
}

// Create the file names to save the topics based on the name of the topics
std::string createFileName(std::string topic_name, char filler = '|'){
    std::string file_name = topic_name;
    size_t pos = file_name.find('/');
    if(pos == 0){
        file_name.erase(pos,1);
        pos = file_name.find('/');
    }
    while(pos != std::string::npos){
        file_name[pos] = filler;
        pos = file_name.find('/');
    }
    file_name +=".dat";
    return file_name;
}

// Write the content of the nav_msgs::Odometry message into a file
bool writeToFile(std::ofstream& ofs, nav_msgs::Odometry::ConstPtr odom){
    bool result = true;
    if(ofs.is_open()){
        ofs<<"header: "<<std::endl;
        ofs<<"  seq: "<<odom->header.seq<<std::endl;
        ofs<<"  time: "<<std::endl;
        ofs<<"      sec: "<<odom->header.stamp.sec<<std::endl;
        ofs<<"      nsec: "<<odom->header.stamp.nsec<<std::endl;
        ofs<<"  frame_id: "<<odom->header.frame_id<<std::endl;
        ofs<<"child_frame_id: "<<odom->child_frame_id<<std::endl;
        ofs<<"pose: "<<std::endl;
        ofs<<"  pose: "<<std::endl;
        ofs<<"      position: "<<std::endl;
        ofs<<"          x: "<<odom->pose.pose.position.x<<std::endl;
        ofs<<"          y: "<<odom->pose.pose.position.y<<std::endl;
        ofs<<"          z: "<<odom->pose.pose.position.z<<std::endl;
        ofs<<"      orientation: "<<std::endl;
        ofs<<"          x: "<<odom->pose.pose.orientation.x<<std::endl;
        ofs<<"          y: "<<odom->pose.pose.orientation.y<<std::endl;
        ofs<<"          z: "<<odom->pose.pose.orientation.z<<std::endl;
        ofs<<"          w: "<<odom->pose.pose.orientation.w<<std::endl;
        ofs<<"  covariance: "<<std::endl;
        std::stringstream ss;
        for(int i = 0; i<6; i++){
            for(int j = 0; j<6; j++){
                if(odom->pose.covariance[i*6+j]<1000){
                    if(odom->pose.covariance[i*6+j]>1e-4){
                        ss.precision(7);
                        ss.width(15);
                        ss << std::left << odom->pose.covariance[i*6+j];
                    }else{
                        ss.precision(7);
                        ss.width(15);
                        ss << std::left << 0.0;  
                    }
                }else{
                    ss.width(15);
                    ss << std::left << "inf";
                }
            }
            ofs<<"             "<<ss.str()<<std::endl;
            ss.str(std::string());
        }
        ofs<<"twist: "<<std::endl;
        ofs<<"  twist: "<<std::endl;
        ofs<<"      linear: "<<std::endl;
        ofs<<"          x: "<<odom->twist.twist.linear.x<<std::endl;
        ofs<<"          y: "<<odom->twist.twist.linear.y<<std::endl;
        ofs<<"          z: "<<odom->twist.twist.linear.z<<std::endl;
        ofs<<"      angular: "<<std::endl;
        ofs<<"      x: "<<odom->twist.twist.angular.x<<std::endl;
        ofs<<"      y: "<<odom->twist.twist.angular.y<<std::endl;
        ofs<<"      z: "<<odom->twist.twist.angular.z<<std::endl;
        ofs<<"  covariance: "<<std::endl;
        ss.str(std::string());
        for(int i = 0; i<6; i++){
            for(int j = 0; j<6; j++){
                if(odom->twist.covariance[i*6+j]<1000){
                    if(odom->twist.covariance[i*6+j]>1e-4){
                        ss.precision(7);
                        ss.width(15);
                        ss << std::left << odom->twist.covariance[i*6+j];
                    }else{
                        ss.precision(7);
                        ss.width(15);
                        ss << std::left << 0.0;  
                    }
                }else{
                    ss.width(15);
                    ss << std::left << "inf";
                }
            }
            ofs<<"             "<<ss.str()<<std::endl;
            ss.str(std::string());
        }
        ofs<<"---"<<std::endl;
    }else{
        result = false;
    }
    return result;

}

// Write the content of the sensor_msgs::Imu message into a file
bool writeToFile(std::ofstream& ofs, sensor_msgs::Imu::ConstPtr imu){
    bool result = true;
    if(ofs.is_open()){
        ofs<<"header: "<<std::endl;
        ofs<<"  seq: "<<imu->header.seq<<std::endl;
        ofs<<"  time: "<<std::endl;
        ofs<<"      sec: "<<imu->header.stamp.sec<<std::endl;
        ofs<<"      nsec: "<<imu->header.stamp.nsec<<std::endl;
        ofs<<"  frame_id: "<<imu->header.frame_id<<std::endl;
        ofs<<"orientation: "<<std::endl;
        ofs<<"  x: "<<imu->orientation.x<<std::endl;
        ofs<<"  y: "<<imu->orientation.y<<std::endl;
        ofs<<"  z: "<<imu->orientation.z<<std::endl;
        ofs<<"  w: "<<imu->orientation.w<<std::endl;
        ofs<<"orientation_covariance: "<<std::endl;   
        std::stringstream ss;
        ss.str(std::string());
        for(int i = 0; i<3; i++){
            for(int j = 0; j<3; j++){
                if(imu->orientation_covariance[i*3+j]<1000){
                    if(imu->orientation_covariance[i*3+j]>1e-4){
                        ss.precision(7);
                        ss.width(15);
                        ss << std::left << imu->orientation_covariance[i*3+j];
                    }else{
                        ss.precision(7);
                        ss.width(15);
                        ss << std::left << 0.0;  
                    }
                }else{
                    ss.width(15);
                    ss << std::left << "inf";
                }
            }
            ofs<<"                       "<<ss.str()<<std::endl;
            ss.str(std::string());
        }
        ofs<<"angular_velocity: "<<std::endl;
        ofs<<"  x: "<<imu->angular_velocity.x<<std::endl;
        ofs<<"  y: "<<imu->angular_velocity.y<<std::endl;
        ofs<<"  z: "<<imu->angular_velocity.z<<std::endl;
        ofs<<"angular_velocity_covariance: "<<std::endl;   
        ss.str(std::string());
        for(int i = 0; i<3; i++){
            for(int j = 0; j<3; j++){
                if(imu->angular_velocity_covariance[i*3+j]<1000){
                    if(imu->angular_velocity_covariance[i*3+j]>1e-4){
                        ss.precision(7);
                        ss.width(15);
                        ss << std::left << imu->angular_velocity_covariance[i*3+j];
                    }else{
                        ss.precision(7);
                        ss.width(15);
                        ss << std::left << 0.0;  
                    }
                }else{
                    ss.width(15);
                    ss << std::left << "inf";
                }
            }
            ofs<<"                       "<<ss.str()<<std::endl;
            ss.str(std::string());
        }
        ofs<<"linear_acceleration: "<<std::endl;
        ofs<<"  x: "<<imu->linear_acceleration.x<<std::endl;
        ofs<<"  y: "<<imu->linear_acceleration.y<<std::endl;
        ofs<<"  z: "<<imu->linear_acceleration.z<<std::endl;
        ofs<<"linear_acceleration_covariance: "<<std::endl;   
        ss.str(std::string());
        for(int i = 0; i<3; i++){
            for(int j = 0; j<3; j++){
                if(imu->linear_acceleration_covariance[i*3+j]<1000){
                    if(imu->linear_acceleration_covariance[i*3+j]>1e-4){
                        ss.precision(7);
                        ss.width(15);
                        ss << std::left << imu->linear_acceleration_covariance[i*3+j];
                    }else{
                        ss.precision(7);
                        ss.width(15);
                        ss << std::left << 0.0;  
                    }
                }else{
                    ss.width(15);
                    ss << std::left << "inf";
                }
            }
            ofs<<"                       "<<ss.str()<<std::endl;
            ss.str(std::string());
        }
        ofs<<"---"<<std::endl;
    }else{
        result = false;
    }
    return result;

}


int main(int argc, char** argv){
    std::string bag_file_name;
    bool is_valid_file_name = false;
    if (argc == 2){
        std::tie(bag_file_name, is_valid_file_name) = isValidFileName(argv[1], std::strlen(argv[1]));
        rosbag::Bag bag;
        if(is_valid_file_name){
            try{
                bag.open(bag_file_name, rosbag::bagmode::Read);
            }catch(std::exception& e){
                ROS_FATAL("Invalid File Name!");
                throw e;
            }
        }else{
            ROS_FATAL("Invalid File Name!");
            throw "Invalid File Name!";
        }
        rosbag::View view(bag);
        if(view.size()>0){
            std::map<std::string, std::ofstream> files;
            for(const rosbag::MessageInstance msg: view){
                if(msg.getDataType() == "nav_msgs/Odometry"){
                    nav_msgs::Odometry::ConstPtr odom = msg.instantiate<nav_msgs::Odometry>();
                    if(odom != nullptr){
                        // In case there is more than one topic with message type of 
                        // nav_msgs/Odometry, files which opened to save the messages 
                        // of the topics are stored in a map in correlation to 
                        // the name of the topic.
                        std::string topic_name = msg.getTopic();
                        auto ptr_to_file = files.find(topic_name);
                        if(ptr_to_file == files.end()){
                            std::string file_name  = createFileName(topic_name);
                            std::ofstream ofs(file_name, std::ofstream::out);
                            auto result = files.insert(std::make_pair(topic_name, std::move(ofs)));
                            if(result.second){
                                ptr_to_file = result.first;
                            }else{
                                ROS_FATAL("Couldn't open file!");
                                bag.close();
                                throw "Couldn't open file!";
                            }
                        }
                        writeToFile(ptr_to_file->second, odom);
                    }
                }
                if(msg.getDataType() == "sensor_msgs/Imu"){
                    sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
                    if(imu != nullptr){
                        // In case there is more than one topic with message type of 
                        // nav_msgs/Odometry, files which opened to save the messages 
                        // of the topics are stored in a map in correlation to 
                        // the name of the topic.
                        std::string topic_name = msg.getTopic();
                        auto ptr_to_file = files.find(topic_name);
                        if(ptr_to_file == files.end()){
                            std::string file_name  = createFileName(topic_name);
                            std::ofstream ofs(file_name, std::ofstream::out);
                            auto result = files.insert(std::make_pair(topic_name, std::move(ofs)));
                            if(result.second){
                                ptr_to_file = result.first;
                            }else{
                                ROS_FATAL("Couldn't open file!");
                                bag.close();
                                throw "Couldn't open file!";
                            }
                        }
                        writeToFile(ptr_to_file->second, imu);
                    }
                }
            }

        }    
    }else{
        ROS_FATAL("Enter adress to a bag file!");
        throw "Enter adress to a bag file!";
    }
    
}

