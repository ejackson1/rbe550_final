#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Author: Addison Sears-Collins
// Website: https://automaticaddison.com
// Description: A basic image subscriber for ROS in C++
// Date: June 27, 2020


cv::Point2f search_centroid_in_area(std::vector<cv::Point2f> centroid_vector, cv::Rect area) {
  float sum_x = 0.0;
  float sum_y = 0.0;
  int number_of_centroids_in_area = 0;
 
  for( int i = 0; i<centroid_vector.size(); i++) {
    if(centroid_vector[i].inside(area)) {
      sum_x += centroid_vector[i].x;
      sum_y += centroid_vector[i].y;
      number_of_centroids_in_area++;
    }
  }
  cv::Point2f extracted_point(sum_x/number_of_centroids_in_area, sum_y/number_of_centroids_in_area);
  return extracted_point;
}

std::vector<cv::Point2f> processCanny(cv::Mat canny_output)
{
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
  // get the moments
  std::vector<cv::Moments> mu(contours.size());
  for( int i = 0; i<contours.size(); i++ )
  { mu[i] = cv::moments( contours[i], false ); }
  
  // get the centroid of contours.
  std::vector<cv::Point2f> mc(contours.size());
  for( int i = 0; i<contours.size(); i++) {
    float mc_x = mu[i].m10/mu[i].m00;
    float mc_y = mu[i].m01/mu[i].m00;
    mc[i] = cv::Point2f(mc_x, mc_y);
  }
  // draw contours
  cv::Mat drawing(canny_output.size(), CV_8UC3, cv::Scalar(255,255,255));
  
  for( int i = 0; i<contours.size(); i++ )
  {
  cv::Scalar color = cv::Scalar(167,151,0); // B G R values
  cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
  cv::circle( drawing, mc[i], 4, color, -1, 8, 0 );
  }
  
  // cv::namedWindow( "Contours", cv::WINDOW_AUTOSIZE );
  cv::imshow( "Extracted centroids", drawing );
  cv::waitKey(1000);
  return mc;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
 
  // Pointer used for the conversion from a ROS message to 
  // an OpenCV-compatible image
  cv_bridge::CvImagePtr cv_ptr;
   
  try
  { 
   
    // Convert the ROS message 
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
     
    // Store the values of the OpenCV-compatible image
    // into the current_frame variable
    cv::Mat img_gray;// = cv_ptr->image;
    cv::cvtColor(cv_ptr->image, img_gray, cv::COLOR_BGR2GRAY);
     
    cv::Mat canny_output; 
    cv::Canny(img_gray,canny_output,10,350);
    // Display the current frame
    cv::imshow("view", canny_output); 
    
    // Display frame for 30 milliseconds
    cv::waitKey(1000);

    // extract boundries & calc centroids 
    std::vector<cv::Point2f> centroids = processCanny(canny_output);

    //get box location in 2d image
    cv::Rect box_search_area(0, 0, 1000, 1000);
    cv::Point2f box_centroid = search_centroid_in_area(centroids, box_search_area);
    printf("2D centroid: %f,%f\n", box_centroid.x, box_centroid.y);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


 
int main(int argc, char **argv)
{
  // The name of the node
  ros::init(argc, argv, "frame_listener");
   
  // Default handler for nodes in ROS
  ros::NodeHandle nh;
   
  // Used to publish and subscribe to images
  image_transport::ImageTransport it(nh);
   
  // Subscribe to the /camera topic
  image_transport::Subscriber sub = it.subscribe("/panda_camera/rgb/image_raw", 1, imageCallback);

  // Make sure we keep reading new video frames by calling the imageCallback function
  ros::spin();
   
  // Close down OpenCV
  cv::destroyWindow("view");
}