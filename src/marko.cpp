#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "rgb2hsv.h"

typedef pcl::PointXYZRGB PointT;

ros::Publisher pub;
#ifdef DEBUG
    ros::Publisher pubDebug;
#endif

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {
    // Return
    sensor_msgs::PointCloud2::Ptr res (new sensor_msgs::PointCloud2 ()); 

    // All the objects needed
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    pcl::PassThrough<PointT> pass;
    pcl::SACSegmentation<PointT> segm;
    pcl::ExtractIndices<PointT> extract;
    pcl::EuclideanClusterExtraction<PointT> ec;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());

    // Datasets
    sensor_msgs::PointCloud2::Ptr small_input (new sensor_msgs::PointCloud2 ());
    pcl::PointCloud<PointT>::Ptr small_cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    std::vector<pcl::PointIndices> cluster_indices;

    //sfiltriramo
    sor.setInputCloud (input);
    //TODO: ce bo ze treba in si upamo, lahko kle spreminjamo kolk se filtrira
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*small_input);

    // Read in the cloud data
    pcl::fromROSMsg (*small_input, *small_cloud);
    
    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (small_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.8);
    pass.filter (*cloud_filtered);
    
    ///////////////EKSTRAKCIJA RAVNINE//////////////////////////////////////////
    segm.setOptimizeCoefficients (true);
    segm.setModelType (pcl::SACMODEL_PLANE);
    segm.setMethodType (pcl::SAC_RANSAC);
    segm.setMaxIterations (30);
    segm.setDistanceThreshold (0.03);

    //Segment the largest planar component from the remaining cloud
    segm.setInputCloud (cloud_filtered);
    segm.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0) {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
    }
    
    //Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    
    //Write the planar inliers
    extract.filter (*cloud_plane);
    
    //Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    
    #ifdef DEBUG
        pubDebug.publish (cloud_filtered2);
    #endif
    
    //Creating the KdTree object for the search method of the extraction
    tree->setInputCloud (cloud_filtered2);
    //TODO:tuki moramo nastavit koliko so tocke lahko se narazen da se smatrajo 
    //za isti cluster in pa velikost clusterja, to spreminjamo v skladu s stopnjo filtriranja
    ec.setClusterTolerance (0.03); // 3cm
    ec.setMinClusterSize (1000);
    ec.setMaxClusterSize (3000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered2);
    ec.extract (cluster_indices);
    
    pcl::PointCloud<PointT>::Ptr cloud_cluster_local (new pcl::PointCloud<PointT>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        struct rgb_color rgb;
        rgb.r = 0;    
        rgb.g = 0;
        rgb.b = 0;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
            PointT p = cloud_filtered2->points[*pit]; 
            cloud_cluster_local->points.push_back (p);
            
            uint32_t prgb = *reinterpret_cast<int*>(&p.rgb);
            rgb.r += (prgb >> 16) & 0x0000ff;
            rgb.g += (prgb >> 8) & 0x0000ff;
            rgb.b += (prgb) & 0x0000ff; 
        }
        rgb.r /= cloud_cluster_local->points.size();
        rgb.g /= cloud_cluster_local->points.size();
        rgb.b /= cloud_cluster_local->points.size();
        
        struct hsv_color hsv;
        hsv = rgb_to_hsv(rgb);
        std::cout << rgb.r << " " << rgb.g << " " << rgb.b << std::endl;
        std::cout << hsv.hue << " " << hsv.sat << " " << hsv.val << std::endl << std::endl;      

        break;
    }

    pcl::toROSMsg (*cloud_cluster_local, *res);
    res->header = input->header;
    pub.publish (res);
}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "psywerx");
    ros::NodeHandle nh ("~");
    
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
    
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise <pcl::PointCloud<PointT> > ("output", 1);
    #ifdef DEBUG
        pubDebug = nh.advertise <pcl::PointCloud<PointT> > ("outputDebug", 1);
    #endif
    
    // Spin
    ros::spin ();
}
