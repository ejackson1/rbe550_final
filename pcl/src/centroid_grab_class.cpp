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


class PclNode
{
private:
  ros::NodeHandle nh_;
  ros::Publisher pub1_;
  ros::Publisher pub2_;
  ros::Publisher pub3_;
  ros::Publisher pub4_;
  ros::Subscriber sub_;
  tf::TransformListener listener_;


public:
  PclNode(ros::NodeHandle *nh)
  {
    pub1_ = nh_.advertise<pcl_msgs::ModelCoefficients>("plc_coefficients", 1);
    pub2_ = nh_.advertise<pcl_msgs::PointIndices>("plc_inliers", 1);
    pub3_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("plc_outliers", 1);
    pub4_ = nh_.advertise<geometry_msgs::Point>("plc_centroid", 1);
    sub_ = nh_.subscribe("/panda_camera/depth/points", 1, &PclNode::cloudCb, this);
  }
  // void run()
  // {
  //   ros::spin();
  // }

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*input, cloud);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud.makeShared());
    seg.segment(*inliers, coefficients);

    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(coefficients, ros_coefficients);
    pub1_.publish(ros_coefficients);

    pcl_msgs::PointIndices ros_inliers;
    pcl_conversions::fromPCL(*inliers, ros_inliers);
    pub2_.publish(ros_inliers);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ> cloud_f;
    extract.setInputCloud(cloud.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(cloud_f);
    // pub3_.publish(cloud_f);

    try
    {
      tf::StampedTransform transform;
      
      geometry_msgs::Transform geoMsg;
      listener_.lookupTransform("world", "panda_camera_optical_link", ros::Time(0), transform);
      pcl::PointCloud<pcl::PointXYZ> cloud_y;
      geoMsg.translation.x = transform.getOrigin().getX();
      geoMsg.translation.y = transform.getOrigin().getY();
      geoMsg.translation.z = transform.getOrigin().getZ();
      geoMsg.rotation.x = transform.getRotation().getX();
      geoMsg.rotation.y = transform.getRotation().getY();
      geoMsg.rotation.z = transform.getRotation().getZ();
      geoMsg.rotation.w = transform.getRotation().getW();
      // geoMsg.header.frame_id = "world";

      pcl_ros::transformPointCloud(cloud_f, cloud_y, geoMsg);
      cloud_y.header.frame_id = "world";
      pub3_.publish(cloud_y);
    }
    catch(tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }
  }
};

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl");
  ros::NodeHandle nh;

  // tf::TransformListener listener_;
  // // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe ("/panda_camera/depth/points", 1, cloud_cb);

  // // Create a ROS publisher for the output model coefficients
  // pub1 = nh.advertise<pcl_msgs::ModelCoefficients> ("plc_coefficients", 1);

  // // Create a ROS publisher for the output model inliers
  // pub2 = nh.advertise<pcl_msgs::PointIndices> ("plc_inliers", 1);

  // // Create a ROS publisher for the output model inliers
  // pub3 = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("plc_outliers", 1);
  
  // // Create a ROS publisher for the centroids
  // pub4 = nh.advertise<geometry_msgs::Point> ("plc_centroid", 1);
  
  ros::NodeHandle& ref = nh;
  PclNode obj(&nh);
  // Spin
  ros::spin ();
}