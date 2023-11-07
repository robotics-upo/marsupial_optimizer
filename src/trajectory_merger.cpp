#include <ctime>
#include <string>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

using namespace std;

int main(int argc, char **argv){
	if (argc < 3) {
		std::cout << "Usage: " << argv[0] << " <out_file> <in_file1> <in_file2> ...." << std::endl;
		return -1;
	}

	std::vector<YAML::Node> inputs;
	for (int i = 2; i < argc; i++) {
		inputs.push_back(YAML::LoadFile(argv[i]));
		std::cout << "YAML FILE readed. YAML FILE NAME: " << argv[i] << std::endl;
	}

	YAML::Node ugv = YAML::Node(YAML::NodeType::Map);
  	YAML::Node uav = YAML::Node(YAML::NodeType::Map);
  	YAML::Node tether = YAML::Node(YAML::NodeType::Map);

	int ts = 0;
	for (size_t i = 0; i < inputs.size(); i++) {
		int s = inputs[i]["marsupial_ugv"]["size"].as<int>();
		for (int j = 0; j < s; j++) {
			std::string ugv_pos_data = "poses" + std::to_string(j);
    		std::string uav_pos_data = "poses" + std::to_string(j);
    		std::string tether_data = "length" + std::to_string(j);
    
			std::string new_ugv_pos = "poses" + std::to_string(j+ts);
			std::string new_uav_pos = new_ugv_pos;
			std::string new_tether_data = "length" + std::to_string(j+ts);

			ugv[new_ugv_pos] = inputs[i]["marsupial_ugv"][ugv_pos_data];
			uav[new_uav_pos] = inputs[i]["marsupial_uav"][uav_pos_data];
			tether[new_tether_data] = inputs[i]["tether"][tether_data];
		}
		ts += s;
	}
	ugv["size"] = ts;
	uav["size"] = ts;
	tether["size"] = ts;


	YAML::Node merged = YAML::Node(YAML::NodeType::Map);
	merged["size"]= ts;	
	merged["marsupial_ugv"] = ugv;	
	merged["marsupial_uav"] = uav;
	merged["tether"] = tether;

    // Write to file
    std::ofstream fout;
	fout.open(argv[1]);
    fout << merged;
	fout.close();

	return 0;
}

