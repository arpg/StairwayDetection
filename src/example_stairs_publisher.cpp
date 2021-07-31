// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <stair_detection/preanalysis.h>
#include <stair_detection/regions.h>
#include <stair_detection/regiongrowing.h>
#include <stair_detection/voxSAC.h>
#include <stair_detection/splitmerge.h>
#include <stair_detection/planeshape.h>
#include <stair_detection/recognition.h>
#include <stair_detection/StairVector.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

int main (int argc, char *argv[])
{
    if(argc < 2)
	{
		std::cerr << "Not enough arguments - " << argv[0] << " <input pcd> <input_pcd_frame_id>=stairs" << std::endl;
		return 1;
	}

	std::string input_pcd_file = argv[1];
	std::string input_pcd_frame_id;
	if(argc > 2){ 
		input_pcd_frame_id = argv[2]; 
	}
	else
	{
		input_pcd_frame_id = "stairs";
	}
	

    ros::init(argc, argv, "example_stairs_publisher");
    ros::NodeHandle n;
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud", 1);

// Loading input point cloud //

	int return_status;

	std::cout<<"Starting loading point cloud"<<std::endl;
	double loadS = pcl::getTime();

	PointCloudT::Ptr mainCloud;
	mainCloud.reset (new PointCloudT);

	return_status = pcl::io::loadPCDFile (input_pcd_file, *mainCloud);
	if (return_status != 0)
	{
		PCL_ERROR("Error reading point cloud %s\n", argv[1]);
		return -1;
	}
	double loadE = pcl::getTime();
	std::cout<<"Loading took: "<<loadE-loadS<<std::endl;

// Publishing input point cloud //

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*mainCloud, cloud_msg);
    cloud_msg.header.frame_id = input_pcd_frame_id;
    while(ros::ok())
    {
        cloud_msg.header.stamp = ros::Time::now();
        cloud_pub.publish(cloud_msg);
        ros::spinOnce();
        ros::Rate(1).sleep();
    }
}

