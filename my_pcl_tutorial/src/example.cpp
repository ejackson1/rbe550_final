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
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
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
  pub1.publish (ros_coefficients);

  // Publish the model inliers
  pcl_msgs::PointIndices ros_inliers;
  pcl_conversions::fromPCL(*inliers, ros_inliers);
  pub2.publish (ros_inliers);

  // Calculate and Publish the model outliers
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ> cloud_f;
  extract.setInputCloud (cloud.makeShared ());
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (cloud_f);
  pub3.publish (cloud_f);
  
  // centroid calculation of outliers
  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  for(int i = 0; i < cloud_f.points.size(); i++)
  {
    centroid.add(cloud_f.points[i]);
  }
  pcl::PointXYZ c1;
  centroid.get (c1);
  //pub5.publish (c1);
  std::cout << "Centroid c1: " << c1 << "\n";

  // Calculate surface normals on outliers
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_f.makeShared ());

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);
  
  // cloud_normals->size () should have the same size as the input cloud->size ()*
  
  // Publish Surface Normals
  pub4.publish (cloud_normals);
  
  std::cout << "Single Outlier x: " << cloud_f[0].x << "\n";

  //pub5.publish (cloud_normals->points[1].normal_x);
  return;
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
  pub1 = nh.advertise<pcl_msgs::ModelCoefficients> ("plc_coeeficients", 1);

  // Create a ROS publisher for the output model inliers
  pub2 = nh.advertise<pcl_msgs::PointIndices> ("plc_inliers", 1);

  // Create a ROS publisher for the output model inliers
  pub3 = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("plc_outliers", 1);

  // Create a ROS publisher for the output model inliers
  pub4 = nh.advertise<pcl::PointCloud<pcl::Normal>> ("plc_normals", 1);
  
  //pub5 = nh.advertise<pcl::PointXYZ> ("plc_centroid", 1);

  // Spin
  ros::spin ();
}