#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
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
#include <pcl_ros/transforms.h>
#include <gazebo_msgs/LinkStates.h>

// #include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>


ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
pcl::PointXYZ c1;


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

  // Transform
  try
  {
    tf::StampedTransform transform;
    tf::tfMessage tfMsg;
    geometry_msgs::Transform geoMsg;
    // listener1.lookupTransform();
    
    // listener_.lookupTransform("world", "panda_camera_optical_link",  
    //                       ros::Time(0), transform);
    pcl::PointCloud<pcl::PointXYZ> cloud_y;
    // tfMsg.transforms.header = transform.
    // transform.getOrigin()
    // cout <<  transform;
    geoMsg.translation.x = transform.getOrigin().getX();
    geoMsg.translation.y = transform.getOrigin().getY();
    geoMsg.translation.z = transform.getOrigin().getZ();
    geoMsg.rotation.x = transform.getRotation().getX();
    geoMsg.rotation.y = transform.getRotation().getY();
    geoMsg.rotation.z = transform.getRotation().getZ();
    geoMsg.rotation.w = transform.getRotation().getW();
    // geoMsg.header.frame_id = "world";
    // cout << transform;
    
    
    // pcl_ros::transformPointCloud(cloud_f, cloud_y, geoMsg);

  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  
  
  // centroid calculation of outliers
  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  for(int i = 0; i < cloud_f.points.size(); i++)
  {
    centroid.add(cloud_f.points[i]);
  }
  
  centroid.get (c1);
  geometry_msgs::Point ros_c;
  ros_c.x = c1.x;
  ros_c.y = c1.y;
  ros_c.z = c1.z;
  pub4.publish (ros_c);
  return;
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl");
  ros::NodeHandle nh;

  tf::TransformListener listener_;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/panda_camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub1 = nh.advertise<pcl_msgs::ModelCoefficients> ("plc_coefficients", 1);

  // Create a ROS publisher for the output model inliers
  pub2 = nh.advertise<pcl_msgs::PointIndices> ("plc_inliers", 1);

  // Create a ROS publisher for the output model inliers
  pub3 = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("plc_outliers", 1);
  
  // Create a ROS publisher for the centroids
  pub4 = nh.advertise<geometry_msgs::Point> ("plc_centroid", 1);
  
  // Spin
  ros::spin ();
}