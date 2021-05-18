#ifndef UTILITIES_HPP
#define UTILITIES_HPP


#include "nav_msgs/OccupancyGrid.h"
#include <tf2/LinearMath/Quaternion.h>
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
        //HH_OF: map_file_address_pgm = map_folder_address + "/" + image_name;       
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
        if(entry=="trinary") map_info.mode = MapMode::TRINARY;
        else if(entry=="scale") map_info.mode = MapMode::SCALE;
        else if(entry=="raw") map_info.mode = MapMode::RAW;
        else{
        ROS_ERROR("Invalid mode tag \"%s\".", entry.c_str());
        exit(-1);
        } 
    }catch (YAML::Exception &) {
        ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
        map_info.mode = MapMode::TRINARY;
    }

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
    map->info.origin.position.y = 0.0;
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
            for(int k = 0; k<number_of_color_channels; k++)
                color_sum += *(pixels + j*rowstride + i*number_of_channels + k);
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
                MapMode map_mode, double free_thresh = 0.65, double occupied_thresh = 0.15 )
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

    FILE* map_file = fopen(pgm_file.c_str(), "w");
    if (!map_file)
    {
        ROS_ERROR("Couldn't save map file to %s", pgm_file.c_str());
        return;
    }

    fprintf(map_file, "P5\n# CREATOR: HH_OF %.3f m/pix\n%d %d\n255\n",
            map->info.resolution, map->info.width, map->info.height);
    for(int y = 0; y < map->info.height; y++) {
        for(int x = 0; x < map->info.width; x++) {
            int i = x + (map->info.height - y - 1) * map->info.width;
            if (map->data[i] >= 0 && map->data[i] <= 100*free_thresh) { 
                fputc(0, map_file);
            } else if (map->data[i] > 100*occupied_thresh) {
                fputc(255, map_file);
            } else { 
                fputc(map->data[i], map_file);
          }
        }
      }
      fclose(map_file);
/*
std::string mapmetadatafile = mapname_ + ".yaml";
ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

geometry_msgs::Quaternion orientation = map->info.origin.orientation;
tf2::Matrix3x3 mat(tf2::Quaternion(
orientation.x,
orientation.y,
orientation.z,
orientation.w
));
double yaw, pitch, roll;
mat.getEulerYPR(yaw, pitch, roll);

fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
        mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

fclose(yaml);

ROS_INFO("Done\n");
saved_map_ = true;

*/
}




void inflateMap(nav_msgs::OccupancyGrid::ConstPtr original_map, 
                nav_msgs::OccupancyGrid::Ptr inflated_map,
                const unsigned int inflation_rate = 10)
{
    inflated_map->info.height = (original_map->info.height - 1) / inflation_rate + 1;
    inflated_map->info.width = (original_map->info.width - 1) / inflation_rate + 1;
    inflated_map->info.resolution = original_map->info.resolution * static_cast<double>(inflation_rate);
    inflated_map->info.origin = original_map->info.origin;

    inflated_map->data.reserve(inflated_map->info.width * inflated_map->info.height);

    for (int x = 0; x < inflated_map->info.width - 1 ; x++){
        for (int y = 0; y < inflated_map->info.height - 1; y++){
            inflated_map->data[x + y * inflated_map->info.width] = -1;
            for(int i = x * inflation_rate; i< x + inflation_rate; i++){
                for(int j = y * inflation_rate; j< y + inflation_rate; j++){
                    if (original_map->data[i + j * original_map->info.width]  == 1 ){
                        inflated_map->data[x + y * inflated_map->info.width] = 1;
                        goto next_inflated_cell; 
                    }
                }
            }
            next_inflated_cell: ;
        }
    }
    for(int i = (inflated_map->info.width - 1) * inflation_rate; i < original_map->info.width; i++){
        for(int j = (inflated_map->info.height - 1) * inflation_rate; j < original_map->info.height; j++){
            
        }
    }
}

void rayTraceOnMap(nav_msgs::OccupancyGrid::ConstPtr map
                        , geometry_msgs::PoseWithCovariance::ConstPtr current_pose
                        , sensor_msgs::LaserScan::Ptr laser_scan)
{
    

}



#endif