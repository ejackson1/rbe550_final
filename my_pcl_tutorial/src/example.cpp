#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/extract_indices.h>
ros::Publisher pub;
ros::Publisher pub3;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (*inliers, coefficients); 
  
  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub.publish (ros_coefficients);

  // Publish the model inliers
  pcl_msgs::PointIndices ros_inliers;
  pcl_conversions::fromPCL(*inliers, ros_inliers);
  pub.publish (ros_inliers);

  // Publish the model outliers
  pcl::PointIndices::Ptr inlier_prt;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ> cloud_f;
  extract.setInputCloud (cloud.makeShared ());
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (cloud_f);

  pub3.publish (cloud_f);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/panda_camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("plc_coeeficients", 1);

  // Create a ROS publisher for the output model inliers
  pub = nh.advertise<pcl_msgs::PointIndices> ("plc_inliers", 1);

  // Create a ROS publisher for the output model inliers
  pub3 = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("plc_outliers", 1);

  // Spin
  ros::spin ();
}