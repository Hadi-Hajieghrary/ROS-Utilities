#ifndef UTILITIES_HPP
#define UTILITIES_HPP


#include "nav_msgs/OccupancyGrid.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovariance.h"

#include <string>
#include <fstream>
#include <array>
#include <vector>
#include <SDL/SDL_image.h>

enum class MapMode {TRINARY, SCALE, RAW};

struct MapInfo{
    std::string image_name = "";
    double resolution;
    std::array<double,3> origin;
    double occupied_thresh;
    double free_thresh;
    bool negate;
    MapMode mode = MapMode::TRINARY;
};

void readYAML(const std::string file_address, const std::string file_name, MapInfo& map_info){
    
    const std::string yaml_file_ext(".yaml");
    std::string yaml_file_name("");
    if(file_name.substr(file_name.length() - yaml_file_ext.length()) != yaml_file_ext){
        yaml_file_name  = file_name + yaml_file_ext;
    }else{
        yaml_file_name  = file_name;
    }
    std::string yaml_file("");
    if (file_address.back() != '/'){
        yaml_file= file_address + '/' + yaml_file_name;
    }else{
        yaml_file= file_address + yaml_file_name;
    }
    std::ifstream yaml_ifs(yaml_file.c_str());
    if (!(yaml_ifs.good())) {
        ROS_ERROR("Map_server could not open %s.", yaml_file.c_str());
        exit(-1);
    }
    YAML::Node map_param = YAML::Load(yaml_ifs);

    try {
        map_info.image_name = map_param["image"].as<std::string>();
    } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a image_path tag or it is invalid.");
        exit(-1);
    }

    try {
        map_info.resolution = map_param["resolution"].as<double>();
    } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
        exit(-1);
    }

    try {
        map_info.origin = std::array<double,3>{map_param["origin"][0].as<double>(),
                                                map_param["origin"][1].as<double>(),
                                                map_param["origin"][2].as<double>()};
    } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a origin tag or it is invalid.");
        exit(-1);
    }

    try {
        map_info.occupied_thresh = map_param["occupied_thresh"].as<double>();
    } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a occupied_thresh tag or it is invalid.");
        exit(-1);
    }

    try {
        map_info.free_thresh = map_param["free_thresh"].as<double>();
    } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
        exit(-1);
    }

    try {
        map_info.negate = (map_param["negate"].as<int>() == 0?false:true);
    } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a negate tag or it is invalid.");
        exit(-1);
    }

    try {
        std::string entry = "";
        entry = map_param["mode"].as<std::string>();
        if(entry=="trinary"){
            map_info.mode = MapMode::TRINARY;
        }else if(entry=="scale"){
            map_info.mode = MapMode::SCALE;
        }else if(entry=="raw"){
            map_info.mode = MapMode::RAW;
        }else{
            ROS_ERROR("Invalid mode tag \"%s\".", entry.c_str());
            exit(-1);
        } 
    }catch (YAML::Exception &) {
        ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
        map_info.mode = MapMode::TRINARY;
    }

}


void saveYAML(const MapInfo& map_info, const std::string file_address, const std::string file_name)
{
    const std::string yaml_file_ext(".yaml");
    std::string yaml_file_name("");
    if(file_name.substr(file_name.length() - yaml_file_ext.length()) != yaml_file_ext){
        yaml_file_name  = file_name + yaml_file_ext;
    }else{
        yaml_file_name  = file_name;
    }
    std::string yaml_file("");
    if (file_address.back() != '/'){
        yaml_file= file_address + '/' + yaml_file_name;
    }else{
        yaml_file= file_address + yaml_file_name;
    }

    FILE* file = fopen(yaml_file.c_str(), "w");
    if (!file)
    {
        ROS_ERROR("Couldn't save yaml file to %s", yaml_file.c_str());
        return;
    }
    fprintf(file, "image: %s\n", map_info.image_name.c_str());
    fprintf(file, "resolution: %.6f\n", map_info.resolution);
    fprintf(file, "origin: [%.6f, %.6f, %.6f]\n", map_info.origin[0], map_info.origin[1], map_info.origin[2]);
    fprintf(file, "negate: %d\n", (map_info.negate?1:0));
    fprintf(file, "occupied_thresh: %.6f\n", map_info.occupied_thresh);
    fprintf(file, "free_thresh: %.6f\n", map_info.free_thresh);
    if(map_info.mode==MapMode::TRINARY){
        fprintf(file, "mode: %s\n", "trinary");
    }else if (map_info.mode==MapMode::SCALE){
        fprintf(file, "mode: %s\n", "scale");
    }else if (map_info.mode==MapMode::RAW){
        fprintf(file, "mode: %s\n", "raw");
    }
    fclose(file);

}


void loadMap(const std::string file_address, const std::string file_name, nav_msgs::OccupancyGrid::Ptr& map, const MapInfo& map_info)
{
    if(map == nullptr){
        map = nav_msgs::OccupancyGrid::Ptr(new nav_msgs::OccupancyGrid);
    }

    const std::string pgm_file_ext(".pgm");
    std::string pgm_file_name("");
    if(file_name.substr(file_name.length() - pgm_file_ext.length()) != pgm_file_ext){
        pgm_file_name  = file_name + pgm_file_ext;
    }else{
        pgm_file_name  = file_name;
    }
    std::string pgm_file("");
    if (file_address.back() != '/'){
        pgm_file= file_address + '/' + pgm_file_name;
    }else{
        pgm_file= file_address + pgm_file_name;
    }


    SDL_Surface* img;
    if(!(img = IMG_Load(pgm_file.c_str()))){
        std::string errmsg = std::string("failed to open image file \"") +
                                            pgm_file + std::string("\": ") + IMG_GetError();
        throw std::runtime_error(errmsg);
    }

    tf2::Quaternion coordinates_orientation;
    coordinates_orientation.setRPY(map_info.origin[2], 0, 0);
    coordinates_orientation.normalize();
    map->info.resolution = map_info.resolution;
    map->info.width = img->w;
    map->info.height = img->h;
    map->info.origin.position.x = map_info.origin[0];
    map->info.origin.position.y = map_info.origin[1];
    map->info.origin.position.z = 0.0;
    map->info.origin.orientation.x = coordinates_orientation.x();
    map->info.origin.orientation.y = coordinates_orientation.y();
    map->info.origin.orientation.z = coordinates_orientation.z();
    map->info.origin.orientation.w = coordinates_orientation.w();
    map->data.resize(map->info.width * map->info.height);

    unsigned char* pixels = static_cast<unsigned char*>(img->pixels);
    int number_of_channels = img->format->BytesPerPixel;
    int rowstride = img->pitch;
    int number_of_color_channels;
    if (map_info.mode==MapMode::TRINARY || !img->format->Amask){
        number_of_color_channels = number_of_channels;
    }else{
        number_of_color_channels = number_of_channels - 1;
    }

    for(int j = 0; j < map->info.height; j++){
        
        for (int i = 0; i < map->info.width; i++){

            int color_sum = 0;
            for(int k = 0; k<number_of_color_channels; k++){
                color_sum += *(pixels + j*rowstride + i*number_of_channels + k);
            }
            double average_color = static_cast<double>(color_sum) / static_cast<double>(number_of_color_channels);
            double alpha;
            if (number_of_channels == 1)
                alpha = 1.0;
            else
                alpha = *(pixels + j*rowstride + i*number_of_channels + number_of_channels - 1);

            if(map_info.negate){
                average_color = 255 - average_color;
            }
            if(map_info.mode == MapMode::RAW){
                map->data[map->info.width * (map->info.height - j - 1) + i] = average_color;
            }else{
                double occ = average_color / 255.0;
                if (map_info.negate) occ = 1 - occ;
                if(occ > map_info.occupied_thresh){
                    map->data[map->info.width * (map->info.height - j - 1) + i] = 100;
                }else if (occ < map_info.free_thresh){
                    map->data[map->info.width * (map->info.height - j - 1) + i] = 0;
                }else if(map_info.mode==MapMode::TRINARY || alpha < 1.0){
                    map->data[map->info.width * (map->info.height - j - 1) + i] = -1;
                }else{
                    map->data[map->info.width * (map->info.height - j - 1) + i] = 1 + 98 * (occ - map_info.free_thresh) / (map_info.occupied_thresh - map_info.free_thresh);
                }
            }
        }
    }

}


void saveMap(const nav_msgs::OccupancyGrid::Ptr& map, const std::string file_address, const std::string file_name,
                MapMode map_mode, double free_thresh = 0.65, double occupied_thresh = 0.15, bool negate = false )
{

    const std::string pgm_file_ext(".pgm");
    std::string pgm_file_name("");
    if(file_name.substr(file_name.length() - pgm_file_ext.length()) != pgm_file_ext){
        pgm_file_name  = file_name + pgm_file_ext;
    }else{
        pgm_file_name  = file_name;
    }
    std::string pgm_file("");
    if (file_address.back() != '/'){
        pgm_file= file_address + '/' + pgm_file_name;
    }else{
        pgm_file= file_address + pgm_file_name;
    }

    std::array<double,3> origin;
    origin[0] = map->info.origin.position.x;
    origin[1] = map->info.origin.position.y;
    geometry_msgs::Quaternion orientation = map->info.origin.orientation;
    tf2::Matrix3x3 mat(tf2::Quaternion(
                                    map->info.origin.orientation.x,
                                    map->info.origin.orientation.y,
                                    map->info.origin.orientation.z,
                                    map->info.origin.orientation.w
                                    )
                        );
    double yaw{0.0}, pitch{0.0}, roll{0.0};
    mat.getEulerYPR(yaw, pitch, roll);
    origin[2] = yaw;
    MapInfo map_info{pgm_file_name, map->info.resolution, origin, 
                        occupied_thresh, free_thresh, negate, map_mode};

    FILE* file = fopen(pgm_file.c_str(), "w");
    if (!file)
    {
        ROS_ERROR("Couldn't save map file to %s", pgm_file.c_str());
        return;
    }
    fprintf(file, "P5\n# CREATOR: HH_OF %.3f m/pix\n%d %d\n255\n",
            map->info.resolution, map->info.width, map->info.height);

    if(map_info.mode == MapMode::TRINARY){
        for(int x = 0; x < map->info.width; x++) {
            for(int y = 0; y < map->info.height; y++) {
                int i = x + (map->info.height - y - 1) * map->info.width;
                double map_value;
                map_value = map->data[i];
                if (negate) map_value = 255 - map_value;
                if (map_value >= 0 && map_value <= 100*free_thresh) { 
                    fputc(negate?100:0, file);
                } else if (map_value > 100*occupied_thresh) {
                    fputc(negate?0:100, file);
                } else { 
                    fputc(-1, file);
                }
            }
        }
    }else if(map_info.mode == MapMode::RAW){
        for(int y = 0; y < map->info.height; y++) {
            for(int x = 0; x < map->info.width; x++) {
                int i = x + (map->info.height - y - 1) * map->info.width;
                double map_value;
                map_value = map->data[i];
                if (negate) map_value = 255 - map_value;
                fputc(map_value, file);
            }
        } 
    }else if(map_info.mode == MapMode::SCALE){
        for(int y = 0; y < map->info.height; y++) {
            for(int x = 0; x < map->info.width; x++) {
                int i = x + (map->info.height - y - 1) * map->info.width;
                double map_value;
                map_value = map->data[i] / 255.0;
                if (negate) map_value = 1 - map_value;
                if(map_value > map_info.occupied_thresh){
                    fputc(100, file);
                }else if (map_value < map_info.free_thresh){
                    fputc(0, file);
                }else{
                    map_value = 1 + 98 * (map_value - map_info.free_thresh) 
                                        / (map_info.occupied_thresh - map_info.free_thresh);
                    fputc(map_value, file);
                }
            }
        } 
    }

    fclose(file);

    saveYAML(map_info, file_address, file_name);
}


void inflateMap(nav_msgs::OccupancyGrid::ConstPtr original_map, 
                nav_msgs::OccupancyGrid::Ptr& inflated_map,
                const unsigned int inflation_rate = 10)
{
    if(inflated_map == nullptr){
        inflated_map = nav_msgs::OccupancyGrid::Ptr(new nav_msgs::OccupancyGrid);
    }
    ROS_INFO("I am here 111111111111");
    inflated_map->info.height = (original_map->info.height - 1) / inflation_rate + 1;
    inflated_map->info.width = (original_map->info.width - 1) / inflation_rate + 1;
    inflated_map->info.resolution = original_map->info.resolution * static_cast<double>(inflation_rate);
    inflated_map->info.origin = original_map->info.origin;
    ROS_INFO("I am here 22222222222");

    inflated_map->data.reserve(inflated_map->info.width * inflated_map->info.height);
    ROS_INFO("I am here 3333333333333");

    for (int x = 0; x < inflated_map->info.width - 1 ; x++){
            ROS_INFO("I am here 4444444444444444");

        for (int y = 0; y < inflated_map->info.height - 1; y++){
                ROS_INFO("I am here 5555555555555555");

            inflated_map->data[x + y * inflated_map->info.width] = -1;
            for(int i = x * inflation_rate; i< (x + 1) * inflation_rate; i++){
                    ROS_INFO("I am here 666666666666666");

                for(int j = y * inflation_rate; j< (y + 1) * inflation_rate; j++){
                        ROS_INFO("I am here 777777777777777777");

                    if (original_map->data[i + j * original_map->info.width]  == 1 ){
                            ROS_INFO("I am here 88888888888888888");

                        inflated_map->data[x + y * inflated_map->info.width] = 1;
                        goto next_inflated_cell; 
                    }
                }
            }
            next_inflated_cell: ;
        }
    }
    int y = inflated_map->info.height - 1;
    for (int x = 0; x < inflated_map->info.width - 1 ; x++){
        inflated_map->data[x + y * inflated_map->info.width] = -1;
        for(int i = x * inflation_rate; i< (x + 1) * inflation_rate; i++){
            for(int j = y * inflation_rate; j< original_map->info.height; j++){
                if (original_map->data[i + j * original_map->info.width]  == 1 ){
                    inflated_map->data[x + y * inflated_map->info.width] = 1;
                    goto next_inflated_cell_in_the_row; 
                }
            }
        }
        next_inflated_cell_in_the_row: ;
    }
    int x = inflated_map->info.width - 1;
    for (int y = 0; y < inflated_map->info.height - 1 ; y++){
        inflated_map->data[x + y * inflated_map->info.width] = -1;
        for(int i = x * inflation_rate; i< original_map->info.width; i++){
            for(int j = y * inflation_rate; j< (y + 1) * inflation_rate; j++){
                if (original_map->data[i + j * original_map->info.width]  == 1 ){
                    inflated_map->data[x + y * inflated_map->info.width] = 1;
                    goto next_inflated_cell_in_the_column; 
                }
            }
        }
        next_inflated_cell_in_the_column: ;
    }

}


void rayTraceOnMap(nav_msgs::OccupancyGrid::ConstPtr map
                        , geometry_msgs::PoseWithCovariance::ConstPtr current_pose
                        , sensor_msgs::LaserScan::Ptr laser_scan)
{
    

}



#endif