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

    //pcl::io::savePCDFileASCII ("scene.pcd", scene);
    // Show
    ROS_INFO("Show1 section");
    {
        PCLVisualizer v("Scene Before global alignment");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.spin();
    }
    
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
