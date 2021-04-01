#include <string>
#include <boost/algorithm/string.hpp>

//ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl_conversions/pcl_conversions.h>

#define PRINTF_REGULAR "\x1B[0m"
#define PRINTF_RED "\x1B[31m"
#define PRINTF_GREEN "\x1B[32m"
#define PRINTF_YELLOW "\x1B[33m"
#define PRINTF_BLUE "\x1B[34m"
#define PRINTF_MAGENTA "\x1B[35m"
#define PRINTF_CYAN "\x1B[36m"
#define PRINTF_WHITE "\x1B[37m"

std::string world_frame, ugv_base_frame;

ros::Publisher segmented_traversable_pc_pub, segmented_obstacles_pc_pub;
ros::Subscriber new_octomap_sub;
sensor_msgs::PointCloud2 data_in;
std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::unique_ptr<tf2_ros::TransformListener> tf2_list;
std::unique_ptr<tf::TransformListener> tf_list_ptr;


pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
pcl::PointXYZ searchPoint;

void startRegionGrowing(sensor_msgs::PointCloud2 in_cloud_);
void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void getPositionUGV();

void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    data_in = *msg;
    ROS_INFO(PRINTF_CYAN"pcCallback() :  data_in.size() = %i [%i / %i]",data_in.height *data_in.width, data_in.height , data_in.width);
	startRegionGrowing(data_in);
}

void startRegionGrowing(sensor_msgs::PointCloud2 in_cloud_)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(in_cloud_, *cloud);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	Eigen::Vector3d init_point_;
    kdtree.setInputCloud (cloud);

    ROS_INFO(PRINTF_CYAN"startRegionGrowing() :  world_frame= %s  ,  ugv_base_frame= %s",world_frame.c_str() ,ugv_base_frame.c_str());

    getPositionUGV();   

	ROS_INFO(PRINTF_CYAN"startRegionGrowing() :  Initial Pos: searchPoint=[%f %f %f]",searchPoint.x , searchPoint.y, searchPoint.z);

    // K nearest neighbor from initial point search 
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    init_point_.x() = cloud->points[pointIdxNKNSearch[0]].x;
	init_point_.y() = cloud->points[pointIdxNKNSearch[0]].y;
	init_point_.z() = cloud->points[pointIdxNKNSearch[0]].z;
	ROS_INFO(PRINTF_CYAN"startRegionGrowing() :  Nearest Pos: init_point= [%f %f %f]",init_point_.x(), init_point_.y(), init_point_.z());

    
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
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (5.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
    std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud_traversable (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud_obstacles (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<Eigen::Vector3f> mean_normals(clusters.size(), Eigen::Vector3f({ 0.,0.,0. }));
    int cluster_idx_key = 0;

    for (size_t cluster_idx = 0 ; cluster_idx< clusters.size () ; cluster_idx ++)
    {
        // std::cout << "Indices in cluster " << cluster_idx << " has " << clusters[cluster_idx].indices.size () << " points." << std::endl;
        int counter = 0;
        while (counter < clusters[cluster_idx].indices.size()){  
            int index_ = clusters[cluster_idx].indices[counter];
            if (cloud->points[index_].x == init_point_.x() && cloud->points[index_].y == init_point_.y() && cloud->points[index_].z == init_point_.z()){
                cluster_idx_key = cluster_idx;
                std::cout << "Number cluster for traversability = " << cluster_idx_key <<std::endl;
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


    // std::cout << "Preparing obtacles point cloud: cloud->size()= " << cloud->size() << " , clusters[cluster_idx_key].indices.size()= " << clusters[cluster_idx_key].indices.size() << std::endl;
    // std::cout << "Preparing obtacles point cloud: size()= " << cloud->size() - clusters[cluster_idx_key].indices.size() << std::endl;


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

    ROS_INFO(PRINTF_CYAN"startRegionGrowing() :  segmented_cloud_obstacles->size() = %lu", segmented_cloud_obstacles->size());
	sensor_msgs::PointCloud2 pc_obstacles_out;
	pcl::toROSMsg(*segmented_cloud_obstacles, pc_obstacles_out);
    segmented_obstacles_pc_pub.publish(pc_obstacles_out);


    ROS_INFO(PRINTF_CYAN"startRegionGrowing() :  segmented_cloud_traversable->size() = %lu", segmented_cloud_traversable->size());
	sensor_msgs::PointCloud2 pc_traversable_out;
	pcl::toROSMsg(*segmented_cloud_traversable, pc_traversable_out);
    segmented_traversable_pc_pub.publish(pc_traversable_out);
}


void getPositionUGV()
{
    geometry_msgs::TransformStamped ret;

	try
	{
		ret = tfBuffer->lookupTransform(world_frame, ugv_base_frame, ros::Time(0));
		ROS_INFO(PRINTF_CYAN"getPositionUGV() :  Got lookupTransform ");
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("Region Growing Segmentation: Couldn't get UGV Pose (frame: %s), so not possible to set UGV start point; tf exception: %s", ugv_base_frame.c_str(),ex.what());
	}

	searchPoint.x = ret.transform.translation.x;
	searchPoint.y = ret.transform.translation.y;
	searchPoint.z = ret.transform.translation.z;
	ROS_INFO(PRINTF_CYAN"getPositionUGV() :  Initial Pos ret.transform.translation=[%f %f %f]",ret.transform.translation.x, ret.transform.translation.y, ret.transform.translation.z);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "region_growing_segmentation_node");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

    ROS_INFO(PRINTF_CYAN"Initialazing region_growing_segmentation node !!");
  
	pn.param<std::string>("world_frame",  world_frame, "map");
  	pn.param<std::string>("ugv_base_frame", ugv_base_frame, "ugv_base_link");
	
    tfBuffer.reset(new tf2_ros::Buffer);
	tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));
	tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));

	segmented_traversable_pc_pub = n.advertise<sensor_msgs::PointCloud2>("region_growing_traversability_pc_map", 10);
	segmented_obstacles_pc_pub = n.advertise<sensor_msgs::PointCloud2>("region_growing_obstacles_pc_map", 10);
    new_octomap_sub = n.subscribe<sensor_msgs::PointCloud2>("/octomap_point_cloud_centers", 1000, pcCallback);
    
	while (ros::ok()) {
        // ros::spinOnce();
        // r.sleep();
    ros::spin();
    }	
	
	return 0;
}
