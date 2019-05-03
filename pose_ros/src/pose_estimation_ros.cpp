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

//#include <pcl/features/shot.h>


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */


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
//typedef pcl::SHOT ShotFeature;

//pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

//ros::Publisher pub;

float radiusSearch=15;//30;
float bestRadius=0;
float bestRMSE=0;
float bestRadiusInliers=0;
float bestInliers=0;

void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq);

PointCloud<PointT>::Ptr global_allignment(PointCloud<PointT>::Ptr objectObj, PointCloud<PointT>::Ptr sceneObj);

PointCloud<PointT>::Ptr local_allignment(PointCloud<PointT>::Ptr objectObj, PointCloud<PointT>::Ptr sceneObj);

float get_ThetaY(Matrix4f transform);

void posesCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& rosInput)
{   
	ROS_INFO("In Callback");  
	//pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud<PointT>::Ptr scene(new PointCloud<PointT>);
    //pcl::PointCloud<pcl::PointXYZ> cloud1;
    //pcl::fromROSMsg (*cloud, cloud1); 
    //pcl::fromPCLPointCloud2(*cloud, cloud1);
    ROS_INFO("Convert section");
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*rosInput,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*scene);
    //viewer->addPointCloud<pcl::PointXYZ> (scene, "cloud");
    //viewer->spin();

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*scene, *scene, indices);

    // Load
    ROS_INFO("Load section");
    PointCloud<PointT>::Ptr object(new PointCloud<PointT>);
  
    loadPCDFile("/home/student/Desktop/squarePCDV3.pcd", *object);
    
    /*
    // Show
    ROS_INFO("Show1 section");
    {
        PCLVisualizer v("Scene Before global alignment");
        v.addPointCloud<PointT>(scene1, PointCloudColorHandlerCustom<PointT>(scene1, 255, 0, 0),"scene");
        v.spin();
    }
    */

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform (0,0) = transform (0,0) * 100;
    transform (1,1) = transform (1,1) * 100;
    transform (2,2) = transform (2,2) * 100;
    pcl::transformPointCloud (*scene, *scene, transform);

    Eigen::Matrix4f transformObj = Eigen::Matrix4f::Identity();
    transformObj (0,0) = transformObj (0,0) * 1.5;
    transformObj (1,1) = transformObj (1,1) * 1.5;
    transformObj (2,2) = transformObj (2,2) * 1.5;
    pcl::transformPointCloud (*object, *object, transformObj);
    
    PointCloud<PointT>::Ptr sceneFiltered(new PointCloud<PointT>);
    PointCloud<PointT>::Ptr objectFiltered(new PointCloud<PointT>);

    PointT newpoint;
    for(pcl::PointCloud<PointT>::iterator i = scene->begin(); i<scene->end(); i++){
    	newpoint.x = i->x;
    	newpoint.y = i->y;
    	newpoint.z = i->z;
        if(newpoint.z<100) {
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
  	seg.setMaxIterations (2500);
  	seg.setDistanceThreshold (0.9);

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

    pcl::VoxelGrid<PointT> sor;
  	sor.setInputCloud (sceneFiltered);
  	sor.setLeafSize (0.4f, 0.4f, 0.4f);
  	sor.filter (*sceneFiltered);


	pcl::VoxelGrid<PointT> objFilt;
  	objFilt.setInputCloud (object);
  	objFilt.setLeafSize (0.4f, 0.4f, 0.4f);
  	objFilt.filter (*objectFiltered);


    PointCloud<PointNormal>::Ptr object_aligned_global(new PointCloud<PointNormal>);
	PointCloud<PointNormal>::Ptr object_aligned_local(new PointCloud<PointNormal>);

	object_aligned_global=global_allignment(objectFiltered, sceneFiltered);
    
    object_aligned_local=local_allignment(object_aligned_global,sceneFiltered);
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

PointCloud<PointT>::Ptr global_allignment(PointCloud<PointT>::Ptr object, PointCloud<PointT>::Ptr scene) {

  	cout << "SEARCH RADIUS: " << radiusSearch << endl;
  	cout << "bestRadius: " << bestRadius << " RMSE: " << bestRMSE<< endl;
  	cout << "bestRadiusInliers: " << bestRadiusInliers << " inliers: " << bestInliers<< endl;
  	/*
    ROS_INFO("Show1.2 section");
    {
        PCLVisualizer v("Scene Before global alignment after down");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.spin();
    }


    ROS_INFO("Show2 section");
    // Show
    {
        PCLVisualizer v("Before global alignment");
        v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.spin();
    }
    std::cout << "PointCloud after filtering: " << object->width * object->height 
       << " data points (" << pcl::getFieldsList (*object) << ").";
	std::cout << "PointCloud after filtering: " << scene->width * scene->height 
       << " data points (" << pcl::getFieldsList (*scene) << ").";

    */
    // Compute surface normals
    ROS_INFO("Compute surface normals section");
    {
        ScopeTime t("Surface normals");
        NormalEstimation<PointT,PointT> ne;
        //ne.setKSearch(100);
        ne.setRadiusSearch(radiusSearch);
        ROS_INFO("Before Object");
        ne.setInputCloud(object);
        ne.compute(*object);
        ROS_INFO("Before scene");

        ne.setInputCloud(scene);
        ROS_INFO("Before compute scene");
        ne.compute(*scene);
    }
    //std::cout << object_normal->points <<std::endl;
    
    // Compute shape features
    ROS_INFO("Compute shape features section");
    PointCloud<FeatureT>::Ptr object_features(new PointCloud<FeatureT>);
    PointCloud<FeatureT>::Ptr scene_features(new PointCloud<FeatureT>);
    {
        ScopeTime t("Shape features");
        
        //pcl::SHOTEstimation<PointT,PointT,ShotFeature> spin;
        SpinImageEstimation<PointT,PointT,FeatureT> spin;
        spin.setRadiusSearch(radiusSearch);
        ROS_INFO("Compute shape features section - Before object");
        spin.setInputCloud(object);
        spin.setInputNormals(object);
        spin.compute(*object_features);
        
        ROS_INFO("Compute shape features section - Before scene");
        spin.setInputCloud(scene);
        spin.setInputNormals(scene);
        spin.compute(*scene_features);
    }
    
    // Find feature matches
    ROS_INFO("find features section");
    //std::cout << "Number of features: "<<object_features->size() << endl;
    Correspondences corr(object_features->size());
    {
        ScopeTime t("Feature matches");
        for(size_t i = 0; i < object_features->size(); ++i) {
        	//std::cout << i << std::endl;
        	//std::cout << "Object: " << object_features->points[i] << endl;
        	//std::cout << "Scene: " << scene_features->points[i] << endl;
            corr[i].index_query = i;
            nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
        }
    }
    /*
    ROS_INFO("show mathces section");
    // Show matches
    {
        PCLVisualizer v("Matches");
        v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.addCorrespondences<PointT>(object, scene, corr, 1);
        v.spin();
    }
    */
    
    ROS_INFO("k-d section");
    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);
    
    // Set RANSAC parameters
    const size_t iter = 3000;
    const float thressq = 1 * 1;
    
    ROS_INFO("start RANSAC section");
    // Start RANSAC
    Matrix4f pose = Matrix4f::Identity();
    Matrix4f poseRotate = Matrix4f::Identity();
    PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>);
    PointCloud<PointNormal>::Ptr object_aligned_afine(new PointCloud<PointNormal>);
    float penalty = FLT_MAX;
    {
        ScopeTime t("RANSAC");
        cout << "Starting RANSAC..." << endl;
        UniformGenerator<int> gen(0, corr.size() - 1);
        for(size_t i = 0; i < iter; ++i) {
            //if((i + 1) % 100 == 0)
                //cout << "\t" << i+1 << endl;
            // Sample 3 random correspondences
            vector<int> idxobj(3);
            vector<int> idxscn(3);
            for(int j = 0; j < 3; ++j) {
                const int idx = gen.run();
                idxobj[j] = corr[idx].index_query;
                idxscn[j] = corr[idx].index_match;
            }
            
            // Estimate transformation
            Matrix4f T;
            Matrix4f TAfine;
            TransformationEstimationSVD<PointNormal,PointNormal> est;
            est.estimateRigidTransformation(*object, idxobj, *scene, idxscn, T);
            
            
            Eigen::Affine3f TRot;
            TRot.matrix() = T;
            TRot.rotate (Eigen::AngleAxisf (3.14-get_ThetaY(T), Eigen::Vector3f::UnitZ()));
            TAfine=TRot.matrix();


            // Apply pose
            transformPointCloud(*object, *object_aligned_afine, TAfine);
            transformPointCloud(*object, *object_aligned, T);
            
            



            // Validate
            vector<vector<int> > idx;
            vector<vector<float> > distsq;
            tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
            
            // Compute inliers and RMSE
            size_t inliers = 0;
            float rmse = 0;
            for(size_t j = 0; j < distsq.size(); ++j)
                if(distsq[j][0] <= thressq)
                    ++inliers, rmse += distsq[j][0];
            rmse = sqrtf(rmse / inliers);
            
            // Evaluate a penalty function
            const float outlier_rate = 1.0f - float(inliers) / object->size();
            //const float penaltyi = rmse;
            const float penaltyi = outlier_rate;
            
            // Update result
            if(penaltyi < penalty) {
                //cout << "\t--> Got a new model with " << inliers << " inliers!" << endl;
                penalty = penaltyi;
                pose = T;
                poseRotate =TAfine;
            }
        }
        
        transformPointCloud(*object, *object_aligned, pose);
        transformPointCloud(*object, *object_aligned_afine, poseRotate);
        
        // Compute inliers and RMSE
        vector<vector<int> > idx;
        vector<vector<float> > distsq;
        tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
        size_t inliers = 0;
        float rmse = 0;
        for(size_t i = 0; i < distsq.size(); ++i)
            if(distsq[i][0] <= thressq)
                ++inliers, rmse += distsq[i][0];
        rmse = sqrtf(rmse / inliers);
    
        // Print pose
        cout << "Got the following pose:" << endl << pose << endl;
        cout << "Inliers: " << inliers << "/" << object->size() << endl;
        cout << "RMSE: " << rmse << endl;
        if(rmse > bestRMSE)
	    {
	    	bestRMSE = rmse;
	    	bestRadius=radiusSearch;
	    }
	    if(inliers > bestInliers)
	    {
	    	bestInliers = inliers;
	    	bestRadiusInliers=radiusSearch;
	    }
    } // End timing

    radiusSearch-=0.5;
    // Show result

    
    {
        PCLVisualizer v("After global alignment");
        v.addPointCloud<PointT>(object_aligned, PointCloudColorHandlerCustom<PointT>(object_aligned, 0, 255, 0), "object_aligned");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.addPointCloud<PointT>(object_aligned_afine, PointCloudColorHandlerCustom<PointT>(scene, 0, 0, 255),"Rotated");
        v.spin();
    }
    
    return object_aligned;

}

PointCloud<PointT>::Ptr local_allignment(PointCloud<PointT>::Ptr objectObj, PointCloud<PointT>::Ptr sceneObj) {
    // Load
    PointCloud<PointNormal>::Ptr object(new PointCloud<PointNormal>);
    PointCloud<PointNormal>::Ptr scene(new PointCloud<PointNormal>);
    
    object=objectObj;
    scene=sceneObj;
    
    /*
    // Show
    {
        PCLVisualizer v("Before local alignment");
        v.addPointCloud<PointNormal>(object, PointCloudColorHandlerCustom<PointNormal>(object, 0, 255, 0), "object");
        v.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
        v.spin();
    }
    */
    
    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);
    
    // Set ICP parameters
    const size_t iter =  50;
    const float thressq = 0.1 * 0.1;
    
    // Start ICP
    Matrix4f pose = Matrix4f::Identity();
    PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>(*object));
    {
        ScopeTime t("ICP");
        cout << "Starting ICP..." << endl;
        for(size_t i = 0; i < iter; ++i) {
            // 1) Find closest points
            vector<vector<int> > idx;
            vector<vector<float> > distsq;
            tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
            
            // Threshold and create indices for object/scene and compute RMSE
            vector<int> idxobj;
            vector<int> idxscn;
            for(size_t j = 0; j < idx.size(); ++j) {
                if(distsq[j][0] <= thressq) {
                    idxobj.push_back(j);
                    idxscn.push_back(idx[j][0]);
                }
            }
            
            // 2) Estimate transformation
            Matrix4f T;
            TransformationEstimationSVD<PointNormal,PointNormal> est;
            est.estimateRigidTransformation(*object_aligned, idxobj, *scene, idxscn, T);
            
            // 3) Apply pose
            transformPointCloud(*object_aligned, *object_aligned, T);
            
            // 4) Update result
            pose = T * pose;
        }
        
        // Compute inliers and RMSE
        vector<vector<int> > idx;
        vector<vector<float> > distsq;
        tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
        size_t inliers = 0;
        float rmse = 0;
        for(size_t i = 0; i < distsq.size(); ++i)
            if(distsq[i][0] <= thressq)
                ++inliers, rmse += distsq[i][0];
        rmse = sqrtf(rmse / inliers);
    
        // Print pose
        cout << "Got the following pose:" << endl << pose << endl;
        cout << "Inliers: " << inliers << "/" << object->size() << endl;
        cout << "RMSE: " << rmse << endl;

    } // End timing

	    
    // Show result
    {
        PCLVisualizer v("After local alignment");
        v.addPointCloud<PointNormal>(object_aligned, PointCloudColorHandlerCustom<PointNormal>(object_aligned, 0, 255, 0), "object_aligned");
        v.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
        v.spin();
    }
    
    return object_aligned;
    
}

inline float dist_sq(const FeatureT& query, const FeatureT& target) {
    float result = 0.0;
    for(int i = 0; i < FeatureT::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }
    
    return result;
}

void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq) {
    idx = 0;
    distsq = dist_sq(query, target[0]);
    for(size_t i = 1; i < target.size(); ++i) {
        const float disti = dist_sq(query, target[i]);
        if(disti < distsq) {
            idx = i;
            distsq = disti;
        }
    }
}

float get_ThetaY(Matrix4f transform) {
	float thetaX=0;
	float thetaY=0;
	float thetaZ=0;
	if(transform(0,2)<1) {
		if( transform(0,2)>-1) {
			thetaY = asin( transform(0,2) );
			thetaX = atan2(-transform(1,2) , transform(2,2) );
			thetaZ = atan2(-transform(0,1) , transform(0,0) );
		}
		else {
			thetaY =-3.14/2;
			thetaX =-atan2 ( transform(1,0) , transform(1,1)) ;
			thetaZ = 0;
		}
	}
	else {
		thetaY = 3.14/2;
		thetaX = atan2 (transform(1,0) , transform(1,1) );
		thetaZ = 0;
	}
	return thetaZ;
}

