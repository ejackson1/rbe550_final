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
#include <gazebo_msgs/LinkStates.h>

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;
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

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void transform (gazebo_msgs::LinkStates link_states)
{
  geometry_msgs::Pose panda_link7_pose;
  int panda_link_index = getIndex(link_states.name, "robot::panda_link7");
  panda_link7_pose = link_states.pose[panda_link_index];
  //std::cout << "Link7Pose:" << panda_link7_pose.position.x << "\n";
  geometry_msgs::Point ros_c;
  ros_c.x = c1.x+panda_link7_pose.position.x;
  ros_c.y = c1.y+panda_link7_pose.position.y;
  ros_c.z = panda_link7_pose.position.z-c1.z;
  pub5.publish (ros_c);
  return;
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/panda_camera/depth/points", 1, cloud_cb);
  ros::Subscriber sub1 = nh.subscribe ("/gazebo/link_states", 1, transform);

  // Create a ROS publisher for the output model coefficients
  pub1 = nh.advertise<pcl_msgs::ModelCoefficients> ("plc_coefficients", 1);

  // Create a ROS publisher for the output model inliers
  pub2 = nh.advertise<pcl_msgs::PointIndices> ("plc_inliers", 1);

  // Create a ROS publisher for the output model inliers
  pub3 = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("plc_outliers", 1);
  
  // Create a ROS publisher for the centroids
  pub4 = nh.advertise<geometry_msgs::Point> ("plc_centroid", 1);

  // Create a ROS publisher for the centroids
  pub5 = nh.advertise<geometry_msgs::Point> ("plc_world_centroid", 1);

  // Spin
  ros::spin ();
}