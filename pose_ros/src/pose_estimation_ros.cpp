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
#include "pose_ros/Object_pose.h"

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

int RANSACIter = 4000;
int PLANEIter = 200;
float thressqGlobal = 0.01;
//float radiusSearch=0.05;//30;
//int kSearch = 3;
float bestK=0;
float bestKInliers=0;

float bestRadius=0;
float bestRadiusInliers=0;

float bestRMSE=0;
float bestInliers=0;

float bestRMSEForIn=0;
float bestInliersForIn=0;




void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq);
Matrix4f global_allignment(PointCloud<PointT>::Ptr objectObj, PointCloud<PointT>::Ptr sceneObj, int kSearch, float radiusSearch);
Matrix4f local_allignment(PointCloud<PointT>::Ptr objectObj, PointCloud<PointT>::Ptr sceneObj);
PointCloud<PointNormal>::Ptr filter_scene(PointCloud<PointT>::Ptr scene);
void align_object_grab(PointCloud<PointT>::Ptr object, PointCloud<PointT>::Ptr grab, bool triangle);
void align_scene(PointCloud<PointT>::Ptr scene);
void downscale(PointCloud<PointT>::Ptr scene);



vector<float> get_Theta(Matrix4f transform);


vector<string> objects;


int objectIterator=0;

Eigen::Vector4f centroid;

Matrix4f poseGrab;

ros::Publisher chatter_pub;

void posesCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& rosInput)
{   
	ROS_INFO("In Callback");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<pose_ros::Object_pose>("object_pose", 2);
    PointCloud<PointT>::Ptr scene(new PointCloud<PointT>);
    ROS_INFO("Convert section");
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*rosInput,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*scene);
    std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*scene, *scene, indices);
    

    // Load
    std::cout<<"Load section"<< endl;
    PointCloud<PointT>::Ptr object(new PointCloud<PointT>);
    PointCloud<PointT>::Ptr grab(new PointCloud<PointT>);
  
    loadPCDFile(objects[objectIterator], *object);
    loadPCDFile("/home/student/Desktop/grab.pcd", *grab);

    align_scene(scene);

    if(objectIterator==1) {
        align_object_grab(object,grab,true);
    }
    
    else {
        align_object_grab(object,grab,false);
    }


    scene = filter_scene(scene);

    downscale(scene);

    downscale(object);

    downscale(grab);

    

    PointCloud<PointNormal>::Ptr object_aligned_global(new PointCloud<PointNormal>);
    PointCloud<PointNormal>::Ptr object_aligned_local(new PointCloud<PointNormal>);

    Matrix4f poseGlobal, poseLocal, totalPose;

    poseGlobal=global_allignment(object, scene,16, 0.05);
    

    pcl::transformPointCloud (*object, *object_aligned_global, poseGlobal);
    pcl::transformPointCloud (*grab, *grab, poseGlobal);

    poseLocal=local_allignment(object_aligned_global,scene);

    pcl::transformPointCloud (*grab, *grab, poseLocal);
 
    poseGrab=local_allignment(grab, scene);
    pcl::transformPointCloud (*grab, *grab, poseGrab);
    pcl::compute3DCentroid (*grab, centroid);

    totalPose = poseGlobal+poseLocal+poseGrab;
    std::vector<float> angles = get_Theta(totalPose);

    pose_ros::Object_pose msg;
    msg.ID = objectIterator;
    msg.x = centroid[0];
    msg.y = centroid[1];
    msg.z = centroid[2];
    msg.roll=angles[0];
    msg.pitch=angles[1];
    msg.yaw=angles[2];

    chatter_pub.publish(msg);

    if(objectIterator==1) {
        objectIterator=0;
    }
    else {
        objectIterator++;
    }
}


int main(int argc, char**argv) {

    objects.push_back("/home/student/Desktop/squarePCDV3.pcd");
    objects.push_back("/home/student/Desktop/trianglePCD.pcd"); 
    ros::init(argc, argv, "Vo3D");
    ros::NodeHandle nh;
    ROS_INFO("About to enter %s" , "Callback");
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, posesCallback);
    
    ros::Rate loop_rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
    
    return 0;
}

Matrix4f global_allignment(PointCloud<PointT>::Ptr object, PointCloud<PointT>::Ptr scene, int kSearch, float radiusSearch) {
    // Compute surface normals
    {
        NormalEstimation<PointT,PointT> ne;
        ne.setKSearch(kSearch);
        ne.setInputCloud(object);
        ne.compute(*object);
        ne.setInputCloud(scene);
        ne.compute(*scene);
    }
    
    // Compute shape features
    PointCloud<FeatureT>::Ptr object_features(new PointCloud<FeatureT>);
    PointCloud<FeatureT>::Ptr scene_features(new PointCloud<FeatureT>);
    {
        SpinImageEstimation<PointT,PointT,FeatureT> spin;
        spin.setRadiusSearch(radiusSearch);

        spin.setInputCloud(object);
        spin.setInputNormals(object);
        spin.compute(*object_features);
        
        spin.setInputCloud(scene);
        spin.setInputNormals(scene);
        spin.compute(*scene_features);
    }
    
    Correspondences corr(object_features->size());
    {
        for(size_t i = 0; i < object_features->size(); ++i) {
            corr[i].index_query = i;
            nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
        }
    }
    
    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);
    
    // Set RANSAC parameters
    const size_t iter = RANSACIter;
    const float thressq = thressqGlobal * thressqGlobal;
    
    // Start RANSAC
    Matrix4f pose = Matrix4f::Identity();
    Matrix4f poseRotate = Matrix4f::Identity();
    PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>);

    float penalty = FLT_MAX;
    {
        UniformGenerator<int> gen(0, corr.size() - 1);
        for(size_t i = 0; i < iter; ++i) {
            vector<int> idxobj(3);
            vector<int> idxscn(3);
            for(int j = 0; j < 3; ++j) {
                const int idx = gen.run();
                idxobj[j] = corr[idx].index_query;
                idxscn[j] = corr[idx].index_match;
            }
            
            // Estimate transformation
            Matrix4f T;
            TransformationEstimationSVD<PointNormal,PointNormal> est;
            est.estimateRigidTransformation(*object, idxobj, *scene, idxscn, T);
        

            // Apply pose
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
            vector<float> angles = get_Theta(T);
            // Update result
            if(penaltyi < penalty && abs(angles[0])<0.174533&&abs(angles[2])<0.174533) {
                penalty = penaltyi;
                pose = T;
            }
        }
        
        transformPointCloud(*object, *object_aligned, pose);

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
        
       // close the opened 
        if(rmse > bestRMSE)
        {
            bestRMSE = rmse;
            bestRadius=radiusSearch;
            bestInliers = inliers;
            bestK =kSearch;
        }
        if(inliers > bestInliersForIn)
        {
            bestInliersForIn = inliers;
            bestRadiusInliers=radiusSearch;
            bestRMSEForIn=rmse;
            bestKInliers = kSearch;
        }
    } // End timing

    
    return pose;
    //return object_aligned;

}

Matrix4f local_allignment(PointCloud<PointT>::Ptr objectObj, PointCloud<PointT>::Ptr sceneObj) {
    // Load
    PointCloud<PointNormal>::Ptr object(new PointCloud<PointNormal>);
    PointCloud<PointNormal>::Ptr scene(new PointCloud<PointNormal>);
    
    object=objectObj;
    scene=sceneObj;

    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);
    
    // Set ICP parameters
    const size_t iter =  500;
    const float thressq = thressqGlobal * thressqGlobal;
    
    // Start ICP
    Matrix4f pose = Matrix4f::Identity();
    PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>(*object));
    {
        //ScopeTime t("ICP");
        //cout << "Starting ICP..." << endl;
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
    } // End timing
    
    //return object_aligned;
    return pose;
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

vector<float> get_Theta(Matrix4f transform) {
    float thetaX=0;
    float thetaY=0;
    float thetaZ=0;
    vector<float> angles;
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
    angles.push_back(thetaX);
    angles.push_back(thetaY);
    angles.push_back(thetaZ);
    return angles;
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

void align_object_grab(PointCloud<PointT>::Ptr object, PointCloud<PointT>::Ptr grab, bool triangle) {
    
    Eigen::Vector4f centroidMove; 
    pcl::compute3DCentroid (*object, centroidMove);

    Eigen::Matrix4f transformGrab = Eigen::Matrix4f::Identity();
    transformGrab (0,3) = 1.5;
    transformGrab (1,3) = 4;
    transformGrab (2,3) = -1.5;
    pcl::transformPointCloud (*grab, *grab, transformGrab);
    
    Eigen::Matrix4f MoveToCamera = Eigen::Matrix4f::Identity();
    MoveToCamera (0,3) = -centroidMove[0];
    MoveToCamera (1,3) = -centroidMove[1]+2.4;
    MoveToCamera (2,3) = -centroidMove[2];
    pcl::transformPointCloud (*grab, *grab, MoveToCamera);
    pcl::transformPointCloud (*object, *object, MoveToCamera);


    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform (0,0) = transform (0,0) / 100;
    transform (1,1) = transform (1,1) / 100;
    transform (2,2) = transform (2,2) / 100;
    pcl::transformPointCloud (*object, *object, transform);
    pcl::transformPointCloud (*grab, *grab, transform);

    Eigen::Matrix4f transformObj = Eigen::Matrix4f::Identity();
    transformObj (0,0) = transformObj (0,0) * 1.5;
    transformObj (1,1) = transformObj (1,1) * 1.5;
    transformObj (2,2) = transformObj (2,2) * 1.5;
    pcl::transformPointCloud (*object, *object, transformObj);
    pcl::transformPointCloud (*grab, *grab, transformObj);
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

