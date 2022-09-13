#include <string>
#include <boost/algorithm/string.hpp>

//ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>


//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#define PRINTF_REGULAR "\x1B[0m"
#define PRINTF_RED "\x1B[31m"
#define PRINTF_GREEN "\x1B[32m"
#define PRINTF_YELLOW "\x1B[33m"
#define PRINTF_BLUE "\x1B[34m"
#define PRINTF_MAGENTA "\x1B[35m"
#define PRINTF_CYAN "\x1B[36m"
#define PRINTF_WHITE "\x1B[37m"

std::string world_frame, ugv_base_frame;

ros::Publisher segmented_traversable_pc_pub, segmented_obstacles_pc_pub, reduced_map_pub;
ros::Subscriber new_octomap_sub;
sensor_msgs::PointCloud2 data_in;
std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::unique_ptr<tf2_ros::TransformListener> tf2_list;
std::unique_ptr<tf::TransformListener> tf_list_ptr;
std::string pc_sub;

bool debug_rgs, latch_topic;

pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
pcl::PointXYZ searchPoint;

void startRegionGrowing(sensor_msgs::PointCloud2 in_cloud_);
void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void getPositionUGV();
void publishTopicsPointCloud();
void pointCloudToOctomap(sensor_msgs::PointCloud2 msg);

sensor_msgs::PointCloud2 pc_obstacles_out;
sensor_msgs::PointCloud2 pc_traversable_out;
pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud_traversable (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud_obstacles (new pcl::PointCloud<pcl::PointXYZ>);

const double res = 0.1;

void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    data_in = *msg;
    ROS_INFO_COND(debug_rgs,PRINTF_CYAN"  RegionGrowing:  pcCallback() --> data_in.size() = %i [%i / %i]",data_in.height *data_in.width, data_in.height , data_in.width);
	startRegionGrowing(data_in);
}

void startRegionGrowing(sensor_msgs::PointCloud2 in_cloud_)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(in_cloud_, *cloud);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	Eigen::Vector3d init_point_;
    kdtree.setInputCloud (cloud);

    ROS_INFO_COND(debug_rgs,PRINTF_CYAN"  RegionGrowing:  world_frame= %s  ,  ugv_base_frame= %s",world_frame.c_str() ,ugv_base_frame.c_str());

    getPositionUGV();   

	ROS_INFO_COND(debug_rgs,PRINTF_CYAN"  RegionGrowing:  Initial Pos: searchPoint=[%f %f %f]",searchPoint.x , searchPoint.y, searchPoint.z);

    // K nearest neighbor from initial point search 
    int K = 20;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    init_point_.x() = cloud->points[pointIdxNKNSearch[0]].x;
	init_point_.y() = cloud->points[pointIdxNKNSearch[0]].y;
	init_point_.z() = cloud->points[pointIdxNKNSearch[0]].z;
    int id_nearest1_ = pointIdxNKNSearch[0];
    for (int i=0 ; i < K ; i++)
    {
	    ROS_INFO_COND(debug_rgs,PRINTF_CYAN"  RegionGrowing:  Nearest Pos: pointIdxNKNSearch[%i]= [%i]", i, pointIdxNKNSearch[i]);
    }
	ROS_INFO_COND(debug_rgs,PRINTF_CYAN"  RegionGrowing:  Nearest Pos: pointIdxNKNSearch[0]= [%i] , pointIdxNKNSearch[1]=[%i] , pointIdxNKNSearch[2]=[%i] ",pointIdxNKNSearch[0], pointIdxNKNSearch[1], pointIdxNKNSearch[2]);
	ROS_INFO_COND(debug_rgs,PRINTF_CYAN"  RegionGrowing:  Nearest Pos: init_point= [%f %f %f]",init_point_.x(), init_point_.y(), init_point_.z());

    
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    //   pcl::PassThrough<pcl::PointXYZ> pass;
    //   pass.setInputCloud (cloud);
    //   pass.setFilterFieldName ("z");º
    //   pass.setFilterLimits (0.0, 1.0);
    //   pass.filter (*indices);

    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (60);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (4.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (10.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    ROS_INFO_COND(debug_rgs,PRINTF_CYAN"      RegionGrowing: Number of clusters is equal to: %lu .", clusters.size() );
    ROS_INFO_COND(debug_rgs,PRINTF_CYAN"      RegionGrowing: First cluster has: %lu  points.", clusters[0].indices.size() );
    ROS_INFO_COND(debug_rgs,PRINTF_CYAN"      RegionGrowing: These are the indices of the points of the initial.");

	
    std::vector<Eigen::Vector3f> mean_normals(clusters.size(), Eigen::Vector3f({ 0.,0.,0. }));
    int cluster_idx_key = 93;

    for (size_t cluster_idx = 0 ; cluster_idx< clusters.size() ; cluster_idx ++)
    {
        // std::cout << "Indices in cluster " << cluster_idx << " has " << clusters[cluster_idx].indices.size () << " points." << std::endl;
        int counter = 0;
        while (counter < clusters[cluster_idx].indices.size()){  
            int index_ = clusters[cluster_idx].indices[counter];
            // if (index_ == id_nearest1_ || index_ == id_nearest2_ || index_ == id_nearest3_){
            if (cloud->points[index_].y == init_point_.y()&& cloud->points[index_].z == init_point_.z()){
                printf("index_=[%i] , cluster_idx=[%lu] , clusters.size()=[%lu] ,  counter=[%i]\n", index_,cluster_idx,clusters.size(), counter);
                printf("\t cloud -> points =[%f %f %f]\n", cloud->points[index_].x, cloud->points[index_].y,cloud->points[index_].z);
            }
            if (cloud->points[index_].x == init_point_.x() && cloud->points[index_].y == init_point_.y() && cloud->points[index_].z == init_point_.z()){
                cluster_idx_key = cluster_idx;
                ROS_INFO_COND(debug_rgs,PRINTF_MAGENTA"      RegionGrowing: Number of traversability cluster: %i.", cluster_idx_key);
                ROS_INFO_COND(debug_rgs,PRINTF_MAGENTA"      RegionGrowing: Numer elements Point Cloud that belong to the first cluster: %lu .", clusters[cluster_idx_key].indices.size());

            }
            // std::cout << index_ << ", ";
            mean_normals[cluster_idx] += normals->points[counter].getNormalVector3fMap();
            counter++;
            // if (counter % 10 == 0)
            //     std::cout << std::endl;
        }
        mean_normals[cluster_idx].normalize();
        // std::cout << std::endl << "mean_normals[cluster_idx].normalize(): "<< std::endl << mean_normals[cluster_idx] << std:: endl;
    }

    // Fill in the cloud data
    segmented_cloud_traversable->width = clusters[cluster_idx_key].indices.size();
    segmented_cloud_traversable->height = 1;
    segmented_cloud_traversable->is_dense = false;
    segmented_cloud_traversable->header.frame_id = cloud->header.frame_id;
    int count = 0;
    while (count < clusters[cluster_idx_key].indices.size())
    {
        int index_ = clusters[cluster_idx_key].indices[count];
        segmented_cloud_traversable->push_back (cloud->points[index_]);             
        // mean_normals[cluster_idx] += normals->points[count].getNormalVector3fMap();
        count++;
    }
    // std::cout << "mean_normals[cluster_idx].normalize(): " << mean_normals[cluster_idx].normalize() << std:: endl;
    // std::cout << std::endl;


    std::cout << "Preparing obtacles point cloud: cloud->size()= " << cloud->size() << " , clusters[cluster_idx_key].indices.size()= " << clusters[cluster_idx_key].indices.size() << std::endl;
    std::cout << "Preparing obtacles point cloud: size()= " << cloud->size() - clusters[cluster_idx_key].indices.size() << std::endl;


    segmented_cloud_obstacles->width = (cloud->size() - clusters[cluster_idx_key].indices.size());
    segmented_cloud_obstacles->height = 1;
    segmented_cloud_obstacles->is_dense = false;
    segmented_cloud_obstacles->header.frame_id = cloud->header.frame_id;
    for (size_t cluster_idx = 0 ; cluster_idx< clusters.size() ; cluster_idx ++)
    {
        if(cluster_idx_key != cluster_idx){
            int counter2 = 0;
            while (counter2 < clusters[cluster_idx].indices.size()){  
                int index_ = clusters[cluster_idx].indices[counter2];
                segmented_cloud_obstacles->push_back (cloud->points[index_]);    
                counter2++;
            }
        // std::cout << "Number of cluster= [" << cluster_idx << "/" <<clusters.size() <<"]" <<std::endl;
        }
    }

    publishTopicsPointCloud();
    // pointCloudToOctomap(pc_obstacles_out);
}

void publishTopicsPointCloud()
{
    ROS_INFO_COND(debug_rgs,PRINTF_CYAN"  RegionGrowing:  segmented_cloud_obstacles->size() = %lu", segmented_cloud_obstacles->size());
    pcl::toROSMsg(*segmented_cloud_obstacles, pc_obstacles_out);
    segmented_obstacles_pc_pub.publish(pc_obstacles_out);


    ROS_INFO_COND(debug_rgs,PRINTF_CYAN"  RegionGrowing:  segmented_cloud_traversable->size() = %lu", segmented_cloud_traversable->size());
    pcl::toROSMsg(*segmented_cloud_traversable, pc_traversable_out);
    segmented_traversable_pc_pub.publish(pc_traversable_out);

}

void pointCloudToOctomap(sensor_msgs::PointCloud2 msg)
{
	//********* Retirive and process raw pointcloud************
	std::cout<<"Recieved cloud"<<std::endl;
	std::cout<<"Create Octomap"<<std::endl;
	octomap::OcTree tree(res);
	std::cout<<"Load points "<<std::endl;
	pcl::PCLPointCloud2 cloud;
	pcl_conversions::toPCL(msg,cloud);
	pcl::PointCloud<pcl::PointXYZ> pcl_pc;
    pcl::fromPCLPointCloud2(cloud, pcl_pc);
    std::cout<<"Filter point clouds for NAN"<<std::endl;
	std::vector<int> nan_indices;
	pcl::removeNaNFromPointCloud(pcl_pc,pcl_pc,nan_indices);
	octomap::Pointcloud oct_pc;
	octomap::point3d origin(0.0f,0.0f,0.0f);
	std::cout<<"Adding point cloud to octomap"<<std::endl;
	//octomap::point3d origin(0.0f,0.0f,0.0f);
	for(int i = 0;i<pcl_pc.points.size();i++){
		oct_pc.push_back((float) pcl_pc.points[i].x,(float) pcl_pc.points[i].y,(float) pcl_pc.points[i].z);
    }
	tree.insertPointCloud(oct_pc,origin,-1,false,false);
	
	/*
	//******************Traverse the tree ********************
	for(octomap::OcTree::tree_iterator it =tree.begin_tree(), end = tree.end_tree();it!= end;it++){
		 //manipulate node, e.g.:
		std::cout << "_____________________________________"<<std::endl;
		std::cout << "Node center: " << it.getCoordinate() << std::endl;
		std::cout << "Node size: " << it.getSize() << std::endl;
		std::cout << "Node depth: "<<it.getDepth() << std::endl;
		std::cout << "Is Leaf : "<< it.isLeaf()<< std::endl;
		std::cout << "_____________________________________"<<std::endl;
		}
	//**********************************************************	
	*/
	std::cout<<"finished"<<std::endl;
	std::cout<<std::endl;

    octomap_msgs::Octomap octomap_reduced;
    octomap_reduced.binary = false;
    octomap_reduced.id = 1 ;
    octomap_reduced.resolution =0.1;
    octomap_reduced.header.frame_id = "/map";
    octomap_reduced.header.stamp = ros::Time::now();
    octomap_msgs::fullMapToMsg(tree, octomap_reduced);
    reduced_map_pub.publish(octomap_reduced);

}

void getPositionUGV()
{
    geometry_msgs::TransformStamped ret;

	try
	{
		ret = tfBuffer->lookupTransform(world_frame, ugv_base_frame, ros::Time(0));
		ROS_INFO_COND(debug_rgs,PRINTF_CYAN"  RegionGrowing: getPositionUGV() :  Got lookupTransform ");
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("  RegionGrowing: Region Growing Segmentation: Couldn't get UGV Pose (frame: %s), so not possible to set UGV start point; tf exception: %s", ugv_base_frame.c_str(),ex.what());
	}

	searchPoint.x = ret.transform.translation.x;
	searchPoint.y = ret.transform.translation.y;
	searchPoint.z = ret.transform.translation.z;
	ROS_INFO_COND(debug_rgs,PRINTF_CYAN"  RegionGrowing: getPositionUGV() --> Initial Pos ret.transform.translation=[%f %f %f]",ret.transform.translation.x, ret.transform.translation.y, ret.transform.translation.z);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "region_growing_segmentation_node");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

    ROS_INFO(PRINTF_CYAN"Region Growing Segmentation: Node Initialed!!");
  
	pn.param<std::string>("world_frame",  world_frame, "map");
  	pn.param<std::string>("ugv_base_frame", ugv_base_frame, "ugv_base_link");
  	pn.param<std::string>("pc_sub", pc_sub, "/octomap_point_cloud_centers");
  	pn.param<bool>("debug_rgs", debug_rgs, true);
  	pn.param<bool>("latch_topic", latch_topic, true);
	
    tfBuffer.reset(new tf2_ros::Buffer);
	tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));
	tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));

    new_octomap_sub = n.subscribe<sensor_msgs::PointCloud2>(pc_sub, 1000, pcCallback);

	segmented_traversable_pc_pub = n.advertise<sensor_msgs::PointCloud2>("region_growing_traversability_pc_map", 10,latch_topic);
	segmented_obstacles_pc_pub = n.advertise<sensor_msgs::PointCloud2>("region_growing_obstacles_pc_map", 10,latch_topic);
    reduced_map_pub = n.advertise<octomap_msgs::Octomap>("region_growing_octomap_reduced", 100, latch_topic);

    
	while (ros::ok()) {
        // ros::spinOnce();
        // r.sleep();
        // segmented_obstacles_pc_pub.publish(pc_obstacles_out);
        // segmented_traversable_pc_pub.publish(pc_traversable_out);
        // ros::Rate r(0.5);
        // publishTopicsPointCloud();
        ros::spin();
    }	
	
	return 0;
}
