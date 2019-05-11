#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <gtest/gtest.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_traits.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <fstream>
#include <locale>
#include <stdexcept>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/random.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::registration;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;

typedef PointNormal PointT;
typedef Histogram<153> FeatureT;
int PLANEIter = 200;

float nameExt=7;
PointCloud<PointNormal>::Ptr filter_scene(PointCloud<PointT>::Ptr scene);
void align_scene(PointCloud<PointT>::Ptr scene);
void downscale(PointCloud<PointT>::Ptr scene);

void posesCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& rosInput)
{   
	ROS_INFO("In Callback");  
    PointCloud<PointT>::Ptr scene(new PointCloud<PointT>);
    ROS_INFO("Convert section");
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*rosInput,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*scene);
    //viewer->addPointCloud<pcl::PointXYZ> (scene, "cloud");
    //viewer->spin();
    ROS_INFO("SAVING IMAGE");

    pcl::io::savePCDFileASCII ("scene"+ std::to_string(nameExt) +".pcd", *scene);
    PointCloud<PointT>::Ptr scene1(new PointCloud<PointT>);
    
    
    align_scene(scene);
    downscale(scene);
    scene1 = filter_scene(scene);
    // Show
    ROS_INFO("Show1 section");
    {
        PCLVisualizer v("Scene Before global alignment");
        v.addPointCloud<PointT>(scene1, PointCloudColorHandlerCustom<PointT>(scene1, 255, 0, 0),"scene");
        v.spin();
    }
    nameExt=nameExt+1;
    
}


int main(int argc, char**argv) {
    ros::init(argc, argv, "Vo3D");
    ros::NodeHandle nh;
    ROS_INFO("About to enter %s" , "Callback");
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, posesCallback);
    //pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);
    ros::spin();
    //return 0;

    
        
    return 0;
}
PointCloud<PointNormal>::Ptr filter_scene(PointCloud<PointT>::Ptr scene) {
    PointT newpoint;
    PointCloud<PointT>::Ptr sceneFiltered(new PointCloud<PointT>);
    for(pcl::PointCloud<PointT>::iterator i = scene->begin(); i<scene->end(); i++){
        newpoint.x = i->x;
        newpoint.y = i->y;
        newpoint.z = i->z;
        if(newpoint.x<1 && newpoint.y<1 && newpoint.z<1) {
            sceneFiltered->push_back(newpoint);
        }
    }
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (PLANEIter);
    seg.setDistanceThreshold (0.013);


    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (sceneFiltered);
    seg.segment (*inliers, *coefficients);
    std::cout << "Inliers: " << inliers->indices.size () << std::endl;
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }


    // Extract the inliers
    extract.setInputCloud(sceneFiltered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*sceneFiltered);
    return sceneFiltered;
}

void align_scene(PointCloud<PointT>::Ptr scene) {
    Eigen::Affine3f moveToTable = Eigen::Affine3f::Identity();

    moveToTable.translation() << -0.04, 0.48, -0.46;

    moveToTable.rotate (Eigen::AngleAxisf (0.994838, Eigen::Vector3f::UnitX()));
    moveToTable.rotate (Eigen::AngleAxisf (3.14159, Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud (*scene, *scene, moveToTable);
}

void downscale(PointCloud<PointT>::Ptr cloud) {
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.006f, 0.006f, 0.006f);
    sor.filter (*cloud);
}
