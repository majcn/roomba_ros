#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZRGB PointT;


ros::Publisher pub;
int planeMaxIterations = 100;
double planeNormalDistanceWeight = 0.1;
double planeDistanceThreshold = 0.03;

int maxIterations = 10000;
double normalDistanceWeight = 0.1;
double distanceThreshold = 0.05;
double minRadiusLimits = 0;
double maxRadiusLimits = 0.1;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // All the objects needed
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  pcl::fromROSMsg (*input, *cloud);

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (planeNormalDistanceWeight);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (planeMaxIterations);
  seg.setDistanceThreshold (planeDistanceThreshold);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (normalDistanceWeight);
  seg.setMaxIterations (maxIterations);
  seg.setDistanceThreshold (distanceThreshold); //0.05
  seg.setRadiusLimits (minRadiusLimits, maxRadiusLimits);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);

  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);


  pub.publish(cloud_cylinder);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "psywerx");
  ros::NodeHandle nh("~");

  nh.getParam("maxIterations", maxIterations);
  nh.getParam("normalDistanceWeight", normalDistanceWeight);
  nh.getParam("distanceThreshold", distanceThreshold);
  nh.getParam("minRadiusLimits", minRadiusLimits);
  nh.getParam("maxRadiusLimits", maxRadiusLimits);
  nh.getParam("planeDistanceThreshold", planeDistanceThreshold);
  nh.getParam("planeMaxIterations", planeMaxIterations);
  nh.getParam("planeNormalDistanceWeight", planeNormalDistanceWeight);

  std::cerr << "maxIterations: " << maxIterations << std::endl;
  std::cerr << "normalDistanceWeight: " << normalDistanceWeight << std::endl;
  std::cerr << "distanceThreshold: " << distanceThreshold << std::endl;
  std::cerr << "minRadiusLimits: " << minRadiusLimits << std::endl;
  std::cerr << "maxRadiusLimits: " << maxRadiusLimits << std::endl;
  std::cerr << "planeDistanceThreshold: " << planeDistanceThreshold << std::endl;
  std::cerr << "planeMaxIterations: " << planeMaxIterations << std::endl;
  std::cerr << "planeNormalDistanceWeight: " << planeNormalDistanceWeight << std::endl;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PointCloud<PointT> > ("output", 1);

  // Spin
  ros::spin ();
}
