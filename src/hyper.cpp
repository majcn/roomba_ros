#include <ros/ros.h>

#include "pcl_ros/point_cloud.h"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/point_types.h>

#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  pcl::PointCloud<pcl::PointXYZ> cloud_output;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  pass.setInputCloud (cloud.makeShared());
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (cloud_filtered);

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE); 
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100); 
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered.makeShared());

  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients);
 
  if (inliers_plane->indices.size () == 0) return;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered.makeShared());
  extract.setIndices (inliers_plane);
  extract.setNegative (false);
  extract.filter (cloud_output);

  // Publish the data
  
  pub.publish (cloud_output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("output", 1);

  // Spin
  ros::spin ();
}

