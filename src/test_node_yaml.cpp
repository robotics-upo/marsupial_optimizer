#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <string>
#include <ctime>

using namespace std;
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "testing_yaml");

	ros::NodeHandlePtr nh;
    ros::NodeHandle pnh("~");

    time_t ttime = time(0);
    cout << "Number of seconds elapsed since January 1, 1990:" << ttime << endl;
    tm *local_time = localtime(&ttime);
    
    cout << "Year: "<< 1900 + local_time->tm_year << endl;
    cout << "Month: "<< 1 + local_time->tm_mon<< endl;
    cout << "Day: "<< local_time->tm_mday << endl;
    cout << "Time: "<< 1 + local_time->tm_hour << ":";
    cout << 1 + local_time->tm_min << ":";
    cout << 1 + local_time->tm_sec << endl;

    string year_, month_, day_, hour_, min_, sec_;
    year_ = to_string(1900 + local_time->tm_year);
    month_ = to_string(1 + local_time->tm_mon);
    day_ = to_string(local_time->tm_mday);
    hour_ = to_string(1 + local_time->tm_hour); 
    min_ = to_string(1 + local_time->tm_min); 
    sec_ = to_string(1 + local_time->tm_sec);

    // Root of our file
    YAML::Node root_ugv, root_uav;

    // Populate emitter
    YAML::Emitter emitter;

    // // Create a node listing some values
    root_ugv["marsupial_ugv"] = YAML::Node(YAML::NodeType::Map);
    // // We now will write our values under root["MyNode"]
    YAML::Node node = root_ugv["marsupial_ugv"];
    // YAML::Node node;
    // Write some values
    node["header"] = "marsupial_ugv";
    node["seq"] = 1;
    node["stamp"] = hour_+min_+sec_;
    node["frame_id"] = "ugv";
    
    for(int i=0 ; i < 8; i++){
        node["poses"+to_string(i)]["header"] = "ugv"+to_string(i);
        node["poses"+to_string(i)]["seq"] = i;
        // node["poses"+std::to_string(i)]["stamp"] = i;
        node["poses"+to_string(i)]["frame_id"] = "ugv";  
        node["poses"+to_string(i)]["pose"]["position"]["x"] = 1;
        node["poses"+to_string(i)]["pose"]["position"]["y"] = 2;
        node["poses"+to_string(i)]["pose"]["position"]["z"] = 2;
        node["poses"+to_string(i)]["pose"]["orientation"]["x"] = 2;
        node["poses"+to_string(i)]["pose"]["orientation"]["y"] = 2;
        node["poses"+to_string(i)]["pose"]["orientation"]["z"] = 2;
        node["poses"+to_string(i)]["pose"]["orientation"]["w"] = 2;
    }

    emitter << root_ugv;

    // // Create a node listing some values
    root_uav["marsupial_uav"] = YAML::Node(YAML::NodeType::Map);
    // // We now will write our values under root["MyNode"]
    node = root_uav["marsupial_uav"];
    // YAML::Node node;
    // Write some values
    node["header"] = "marsupial_uav";
    node["seq"] = 1;
    node["stamp"] = hour_+min_+sec_ ;
    node["frame_id"] = "uav";
    
    for(int i=0 ; i < 8; i++){
        node["poses"+to_string(i)]["header"] = "uav"+to_string(i);
        node["poses"+to_string(i)]["seq"] = i;
        // node["poses"+std::to_string(i)]["stamp"] = i;
        node["poses"+to_string(i)]["frame_id"] = "uav";  
        node["poses"+to_string(i)]["pose"]["position"]["x"] = 1;
        node["poses"+to_string(i)]["pose"]["position"]["y"] = 2;
        node["poses"+to_string(i)]["pose"]["position"]["z"] = 2;
        node["poses"+to_string(i)]["pose"]["orientation"]["x"] = 2;
        node["poses"+to_string(i)]["pose"]["orientation"]["y"] = 2;
        node["poses"+to_string(i)]["pose"]["orientation"]["z"] = 2;
        node["poses"+to_string(i)]["pose"]["orientation"]["w"] = 2;
    }

    // Populate emitter
    emitter << root_uav;

    // Write to file
    std::ofstream fout("optimized_path_"+year_+"_"+month_+"_"+day_+"_"+hour_+min_+sec_ +".yaml");
    fout << emitter.c_str();

    return 0;
}
