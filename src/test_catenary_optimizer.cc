/*
Optimizer Local Planner based in g2o for Marsupial Robotics COnfiguration 
Simon Martinez Rozas, 2020 
Service Robotics Lab, University Pablo de Olavide , Seville, Spain 
 */

#include "marsupial_optimizer/test_ceres_constraint_catenary.hpp"
#include "marsupial_optimizer/test_ceres_constraint_catenary_length.hpp"

#include "misc/catenary_solver_ceres.hpp"
#include "misc/data_management.hpp"
	
#include "catenary_checker/grid3d.hpp"
#include "catenary_checker/bisection_catenary_3D.h"
#include "catenary_checker/near_neighbor.hpp"

#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>


class TestOptimizerForCatenary{

public:
	TestOptimizerForCatenary();
	void initializeSubscribers();
	void initializePublishers();
	void setupOptimizer();
	void readOctomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void readPointCloudTraversabilityUGVCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void readPointCloudObstaclesUGVCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg);
	void traversableMapCallBack(const octomap_msgs::OctomapConstPtr &msg);
	void executeOptimizer();
	void initializingParametersblock();
	void finishigOptimizationAndGetDataResult();
	void computeInitialCatenary();

	double parameterCatenary[1];
	ros::NodeHandlePtr nh;
  	std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  	std::unique_ptr<tf2_ros::TransformListener> tf2_list;
  	std::unique_ptr<tf::TransformListener> tf_list_ptr;

	Solver::Options options;
  	Solver::Summary summary;

	NearNeighbor nn_uav; // Kdtree used for Catenary and UAV
	NearNeighbor nn_trav, nn_ugv_obs;
	MarkerPublisher mp_;
	DataManagement dm_;
	Grid3d* grid_3D;

	ros::Subscriber octomap_ws_sub_,point_cloud_ugv_traversability_sub_, point_cloud_ugv_obstacles_sub_, local_map_sub, local_trav_map_sub;
	ros::Publisher catenary_initial_marker_pub_, catenary_final_marker_pub_, initial_final_pub_;

	double map_resolution;
	float step , step_inv;
	double ws_x_max; 
  	double ws_y_max; 
  	double ws_z_max;
  	double ws_x_min;
  	double ws_y_min;
  	double ws_z_min;

	double x_init, y_init, z_init, x_final, y_final, z_final;

	std::string ugv_base_frame, uav_base_frame, reel_base_frame, world_frame;
	std::string path, name_output_file;
	double w_eta_1, w_eta_2, w_eta_;
	double distance_catenary_obstacle, length_tether_max;
	float len_cat_init;
	int scenario_number, num_pos_initial, num_goal, num_test;
	bool write_data_residual, verbose_optimizer;
  bool mapReceivedFull, mapReceivedTrav, use_distance;
	
	int n_iter_opt;	//Iterations Numbers of Optimizer

	octomap::OcTree *mapFull_msg , *mapTrav_msg;
	geometry_msgs::Vector3 pos_init, pos_final;

	visualization_msgs::MarkerArray catenary_marker;

protected:

};

TestOptimizerForCatenary::TestOptimizerForCatenary()
{
	nh.reset(new ros::NodeHandle("~"));

    tfBuffer.reset(new tf2_ros::Buffer);
    tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));
    tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));

	nh->param<int>("num_pos_initial", num_pos_initial, 1);
	nh->param<int>("num_test", num_test, 1);

	std::string n_, n_test;
	n_ = std::to_string(num_test);
	n_test = "test_catenary_"+n_+"/";

	nh->param<double>("map_resolution", map_resolution,0.05);
	nh->param<double>("ws_x_max", ws_x_max, 10.0);
    nh->param<double>("ws_y_max", ws_y_max, 10.0);
    nh->param<double>("ws_z_max", ws_z_max, 20.0);
    nh->param<double>("ws_x_min", ws_x_min, -10.0);
    nh->param<double>("ws_y_min", ws_y_min, -10.0);
    nh->param<double>("ws_z_min", ws_z_min, 0.00);

	nh->param<double>(n_test + "x_init", x_init, 0.0);
	nh->param<double>(n_test + "y_init", y_init, 0.0);
	nh->param<double>(n_test + "z_init", z_init, 0.0);
	nh->param<double>(n_test + "x_final", x_final, 1.0);
	nh->param<double>(n_test + "y_final", y_final, 1.0);
	nh->param<double>(n_test + "z_final", z_final, 1.0);

    nh->param("world_frame", world_frame, (string) "/map");
	
	nh->param<int>("n_iter_opt", n_iter_opt,200);
	nh->param<double>("w_eta_1", w_eta_1,0.1);
	nh->param<double>("w_eta_2", w_eta_2,0.1);

	nh->param<double>("length_tether_max", length_tether_max,20.0);

	nh->param<bool>("write_data_residual",write_data_residual, false);
	nh->param<bool>("verbose_optimizer",verbose_optimizer, true);
	nh->param<bool>("use_distance",use_distance, false);
	
	nh->param("path", path, (std::string) "~/");
	nh->param("name_output_file", name_output_file, (std::string) "optimization_test");
	nh->param<int>("scenario_number", scenario_number, 1);
	

	ROS_INFO(PRINTF_BLUE "Optimizer Local Planner 3D Node Configuration:\n");
		
	step = map_resolution;
	step_inv = 1.0 / step;

	pos_init.x = x_init; 	pos_init.y = y_init; 	pos_init.z = z_init+0.5;
	pos_final.x = x_final; 	pos_final.y = y_final; 	pos_final.z = z_final;
	
	initializeSubscribers();
	initializePublishers();
	setupOptimizer();
	ROS_INFO(PRINTF_BLUE"eta_1=[%f] eta_2=[%f]", w_eta_1, w_eta_2);
	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Parameters loaded, ready for Optimization Process. Waiting for Action!!");

	std::string node_name_ = "grid3D_optimizer_node";
	grid_3D = new Grid3d(node_name_);
	grid_3D->computeTrilinearInterpolation();
}

void TestOptimizerForCatenary::initializeSubscribers()
{
	octomap_ws_sub_ = nh->subscribe( "/octomap_point_cloud_centers", 1,  &TestOptimizerForCatenary::readOctomapCallback, this);
    local_map_sub = nh->subscribe<octomap_msgs::Octomap>("/octomap_binary_local", 1, &TestOptimizerForCatenary::collisionMapCallBack, this);
    local_trav_map_sub = nh->subscribe<octomap_msgs::Octomap>("/oct_trav/octomap_binary", 1, &TestOptimizerForCatenary::traversableMapCallBack, this);
    point_cloud_ugv_obstacles_sub_ = nh->subscribe( "/region_growing_obstacles_pc_map", 1,  &TestOptimizerForCatenary::readPointCloudObstaclesUGVCallback, this);
    point_cloud_ugv_traversability_sub_ = nh->subscribe( "/region_growing_traversability_pc_map", 1,  &TestOptimizerForCatenary::readPointCloudTraversabilityUGVCallback, this);

	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Subscribers Initialized");
}

void TestOptimizerForCatenary::initializePublishers()
{
	catenary_initial_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_initial_marker", 100);
	catenary_final_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_final_marker", 100);
	initial_final_pub_ = nh->advertise<visualization_msgs::MarkerArray>("initial_final_points_cat_", 100);

  	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Publishers Initialized");
}

void TestOptimizerForCatenary::setupOptimizer()
{
	options.max_num_iterations = n_iter_opt;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.minimizer_progress_to_stdout = true;
}

void TestOptimizerForCatenary::readOctomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO(PRINTF_MAGENTA "KDTree : Preparing KDTree UAV");
	nn_uav.setInput(*msg);
  	ROS_INFO(PRINTF_BLUE "Local Planner: Received Point Cloud for KDTree UAV");
}

void TestOptimizerForCatenary::readPointCloudTraversabilityUGVCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO(PRINTF_MAGENTA "KDTree : Preparing KDTree UGV Traversability");
	nn_trav.setInput(*msg);
  	ROS_INFO(PRINTF_BLUE "Local Planner: Received Point Cloud for KDTree traversability UGV");
}

void TestOptimizerForCatenary::readPointCloudObstaclesUGVCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO(PRINTF_MAGENTA "KDTree : Preparing KDTree UGV Obstacles");
	nn_ugv_obs.setInput(*msg);
  	ROS_INFO( PRINTF_BLUE "Local Planner: Received Point Cloud for KDTree obstacles UGV");
}

void TestOptimizerForCatenary::collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg)
{
    mapReceivedFull = true;
	mapFull_msg = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*msg);
}

void TestOptimizerForCatenary::traversableMapCallBack(const octomap_msgs::OctomapConstPtr &msg)
{
    mapReceivedTrav = true;
	mapTrav_msg = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*msg);
}

void TestOptimizerForCatenary::executeOptimizer()
{
	mp_.initialAndFinalPointsMarker(pos_init, pos_final, initial_final_pub_);
	computeInitialCatenary();
  	ROS_INFO( PRINTF_GREEN "Optimizer Local Planner : Initial Length Computed L[%.4f] for p_init[%.2f %.2f %.2f] p_final[%.2f %.2f %.2f]",len_cat_init,pos_init.x, pos_init.y, pos_init.z, pos_final.x, pos_final.y, pos_final.z);
	std::cout << std::endl <<  "==================================================="  << std::endl << "\tPreparing to execute Optimization: Creating Parameter Blocks!!" << std::endl;

  	// Initialize optimizer
  	Problem problem;
    initializingParametersblock();

	/****************************   Catenary Constraints  ****************************/	
	ROS_INFO(PRINTF_ORANGE" PREPARING  CATENARY  PARAMETER  BLOCKS  TO  OPTIMIZE:");
	CostFunction* cost_function_cat_1  = new NumericDiffCostFunction<TestCatenaryFunctor, RIDDERS, 1, 1>
										(new TestCatenaryFunctor(w_eta_1, distance_catenary_obstacle, len_cat_init, write_data_residual, pos_init, pos_final,
															     nh, mapFull_msg, nn_uav.kdtree, nn_uav.obs_points, grid_3D, nn_trav.kdtree, nn_trav.obs_points)); 
	problem.AddResidualBlock(cost_function_cat_1, NULL, parameterCatenary);

	CostFunction* cost_function_cat_2  = new AutoDiffCostFunction<TestCatenaryLengthFunctor, 1, 1>
										(new TestCatenaryLengthFunctor(w_eta_2, len_cat_init, pos_init, pos_final, write_data_residual)); 
	problem.AddResidualBlock(cost_function_cat_2, NULL, parameterCatenary);
	
	std::cout <<  "==================================================="  << std::endl << "Optimization  started !!" << std::endl;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";
	std::cout << std::endl <<"Optimization Proccess Completed !!!" << std::endl << "===================================================" << std::endl << "Saving Temporal data in txt file ..." << std::endl;

	finishigOptimizationAndGetDataResult();
	std::cout <<"Optimization Local Planner Proccess ended !!!!!!!!!!!!!!!!!!!!!!!" << std::endl << "==================================================="<< std::endl << std::endl << std::endl;
}

void TestOptimizerForCatenary::initializingParametersblock()
{
	parameterCatenary[0] = len_cat_init;
	printf("Initial States.Parameter[%f]\n",parameterCatenary[0]);
	mp_.clearMarkers(catenary_marker, 150, catenary_initial_marker_pub_);  //To delete possibles outliers points 
	mp_.clearMarkers(catenary_marker, 150, catenary_final_marker_pub_);  //To delete possibles outliers points 

	std::vector<geometry_msgs::Point> points_catenary_final;
	bisectionCatenary bc;
	points_catenary_final.clear();

	bc.configBisection(parameterCatenary[0] , pos_init.x, pos_init.y, pos_init.z, pos_final.x, pos_final.y, pos_final.z, false);
	bc.getPointCatenary3D(points_catenary_final);
		
	double _d_ = sqrt(pow(pos_init.x - pos_final.x,2) + pow(pos_init.y - pos_final.y,2) + pow(pos_init.z - pos_final.z,2));
	//Graph final catenary states
	mp_.markerPoints(catenary_marker, points_catenary_final, 0, 1, catenary_initial_marker_pub_);	
}

void TestOptimizerForCatenary::finishigOptimizationAndGetDataResult()
{
	double vLC = parameterCatenary[0] ;


	std::vector<geometry_msgs::Point> points_catenary_final;
	bisectionCatenary bc;
	points_catenary_final.clear();

	bc.configBisection(parameterCatenary[0] , pos_init.x, pos_init.y, pos_init.z, pos_final.x, pos_final.y, pos_final.z, false);
	bc.getPointCatenary3D(points_catenary_final);
		
	double _d_ = sqrt(pow(pos_init.x - pos_final.x,2) + pow(pos_init.y - pos_final.y,2) + pow(pos_init.z - pos_final.z,2));
	//Graph final catenary states
	mp_.markerPoints(catenary_marker, points_catenary_final, 2, 1, catenary_final_marker_pub_);	

	printf(PRINTF_REGULAR"Optimized States.Parameter [%.6f/points=%lu] d=[%.6f]\n", parameterCatenary[0],points_catenary_final.size(),_d_);
}

void TestOptimizerForCatenary::computeInitialCatenary()
{
	double delta_ = 0.0;	//Initial Value
    bool check_catenary = true;
    bool increase_catenary;
    double l_cat_;
    double security_dis_ca_ = distance_catenary_obstacle;
    std::vector<geometry_msgs::Point> p_catenary_;
	bisectionCatenary bc;

	double dist_init_final_ = sqrt(pow(pos_init.x - pos_final.x,2) + pow(pos_init.y - pos_final.y,2) + pow(pos_init.z - pos_final.z,2));

	if (use_distance)
		len_cat_init = dist_init_final_ * 1.005;
    else{
		do{
		    increase_catenary = false;
		    p_catenary_.clear();
		    l_cat_ = dist_init_final_* (1.01 + delta_);
		    if (l_cat_ > length_tether_max){
		        check_catenary = false;
		        break;
		    }
		    bool just_one_axe = bc.configBisection(l_cat_, pos_init.x, pos_init.y, pos_init.z, pos_final.x, pos_final.y, pos_final.z, false);
		    bc.getPointCatenary3D(p_catenary_);
		    double d_min_point_cat = 100000;
		    if (p_catenary_.size() > 5){
		        for (size_t i = 0 ; i < p_catenary_.size() ; i++){
		            geometry_msgs::Point point_cat;
		            Eigen::Vector3d p_in_cat_, obs_to_cat_;
		            if (p_catenary_[i].z < ws_z_min*map_resolution + ((1*map_resolution)+security_dis_ca_)){
		                check_catenary = false;
		                break;
		           }
		            p_in_cat_.x() = p_catenary_[i].x;
		            p_in_cat_.y() = p_catenary_[i].y;
		             p_in_cat_.z() = p_catenary_[i].z;
		            double dist_cat_obs;
		            bool is_into_ = grid_3D->isIntoMap(p_in_cat_.x(),p_in_cat_.y(),p_in_cat_.z());
		            if(is_into_)
		                dist_cat_obs =  grid_3D->getPointDist((double)p_in_cat_.x(),(double)p_in_cat_.y(),(double)p_in_cat_.z()) ;
		            else
		                dist_cat_obs = -1.0;
		            if (d_min_point_cat > dist_cat_obs){
		                d_min_point_cat = dist_cat_obs;
		            }
		            if (dist_cat_obs < security_dis_ca_){
		                delta_ = delta_ + 0.005;
		                increase_catenary = true;
		                break;
		            }
		            point_cat.x = p_catenary_[i].x;
		            point_cat.y = p_catenary_[i].y;
		            point_cat.z = p_catenary_[i].z;
		        }
		        if (check_catenary && !increase_catenary){
		            check_catenary = false;
		        }
		    }
		    else{
		        check_catenary = false;
		    }
		}while (check_catenary);
		len_cat_init = l_cat_;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "optmizer_publisher_tf_node");
	// ros::Rate r(2);

    TestOptimizerForCatenary t_op;
	// while (ros::ok()) {
    //     ros::spinOnce();
		ros::Duration(4).sleep();
		t_op.executeOptimizer();
        // r.sleep();
    // }	
	
	return 0;
}
