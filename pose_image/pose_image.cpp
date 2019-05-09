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

//TODO comment in ScopeTime
typedef PointNormal PointT;
typedef Histogram<153> FeatureT;


int RANSACIter = 2500;
int PLANEIter = 1000;
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

PointCloud<PointT>::Ptr global_allignment(PointCloud<PointT>::Ptr objectObj, PointCloud<PointT>::Ptr sceneObj, int kSearch, float radiusSearch);

PointCloud<PointT>::Ptr local_allignment(PointCloud<PointT>::Ptr objectObj, PointCloud<PointT>::Ptr sceneObj);

float get_ThetaY(Matrix4f transform);

int main(int argc, char**argv) {
    std::vector<int> indices;

    // Load
    std::cout<<"Load section"<< endl;
    PointCloud<PointT>::Ptr object(new PointCloud<PointT>);
    PointCloud<PointT>::Ptr scene(new PointCloud<PointT>);
  
    loadPCDFile(argv[1], *object);
    loadPCDFile(argv[2], *scene);

    pcl::removeNaNFromPointCloud(*scene, *scene, indices);
    //loadPCDFile("/home/student/Desktop/squarePCDV3.pcd", *object);
    
    /*
    // Show
    std::cout<<"Show1 section");
    {
        PCLVisualizer v("Scene Before global alignment");
        v.addPointCloud<PointT>(scene1, PointCloudColorHandlerCustom<PointT>(scene1, 255, 0, 0),"scene");
        v.spin();
    }
    */

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform (0,0) = transform (0,0) / 100;
    transform (1,1) = transform (1,1) / 100;
    transform (2,2) = transform (2,2) / 100;
    pcl::transformPointCloud (*object, *object, transform);

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
        if(newpoint.z<1) {
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

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (sceneFiltered);
    sor.setLeafSize (0.004f, 0.004f, 0.004f);
    sor.filter (*sceneFiltered);


    pcl::VoxelGrid<PointT> objFilt;
    objFilt.setInputCloud (object);
    objFilt.setLeafSize (0.004f, 0.004f, 0.004f);
    objFilt.filter (*objectFiltered);

    int iterration=1;
    for(int kSearch = 4; kSearch<=20; kSearch=kSearch+4) {
        for(float radiusSearch=0.005; radiusSearch<=0.1;radiusSearch=radiusSearch+0.005) {
                cout << "iterration " << iterration << " out of 100" << endl; 
                //cout << "SEARCH RADIUS: " << radiusSearch << endl;
                //cout << "SEARCH K: " << kSearch << endl;
                cout << "FOR RMSE -> bestK: " << bestK << " best radius: " << bestRadius << " RMSE: " << bestRMSE<< " inliers: " << bestInliers<< endl;
                cout << "FOR INLIERS -> bestK: " << bestKInliers << " best radius: " << bestRadiusInliers << " RMSE: " << bestRMSEForIn<< " inliers: " << bestInliersForIn<< endl;

            PointCloud<PointNormal>::Ptr object_aligned_global(new PointCloud<PointNormal>);
            PointCloud<PointNormal>::Ptr object_aligned_local(new PointCloud<PointNormal>);

            object_aligned_global=global_allignment(objectFiltered, sceneFiltered,kSearch, radiusSearch);
            //object_aligned_local=local_allignment(object_aligned_global,sceneFiltered);
            iterration++;
            }
        }
    ofstream outfile;
    outfile.open("data1.txt", std::ios_base::app);
    outfile <<"Best Radius in RMSE: " << bestRadius << " K: "<< bestK <<" Inliers: " << bestInliers << " rmse: " << bestRMSE << endl;
    outfile <<"Best Radius in Inliers: " << bestRadiusInliers << " K: "<< bestKInliers <<" Inliers: " << bestInliersForIn << " rmse: " << bestRMSEForIn << endl;
    outfile.close();
    return 0;
}

PointCloud<PointT>::Ptr global_allignment(PointCloud<PointT>::Ptr object, PointCloud<PointT>::Ptr scene, int kSearch, float radiusSearch) {

  	//cout << "bestRadius: " << bestRadius << " RMSE: " << bestRMSE<< endl;
  	//cout << "bestRadiusInliers: " << bestRadiusInliers << " inliers: " << bestInliers<< endl;
  	/*
    std::cout<<"Show1.2 section"<< endl;
    {
        PCLVisualizer v("Scene Before global alignment after down");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.spin();
    }


    std::cout<<"Show2 section"<< endl;
    // Show
    {
        PCLVisualizer v("Before global alignment");
        v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.spin();
    }
    */
    // Compute surface normals
    //std::cout<<"Compute surface normals section"<< endl;
    {
        //ScopeTime t("Surface normals");
        NormalEstimation<PointT,PointT> ne;
        ne.setKSearch(kSearch);
        //ne.setRadiusSearch(radiusSearch);
        //std::cout<<"Before Object"<< endl;
        ne.setInputCloud(object);
        ne.compute(*object);
        //std::cout<<"Before scene"<< endl;

        ne.setInputCloud(scene);
        //std::cout<<"Before compute scene"<< endl;
        ne.compute(*scene);
    }
    //std::cout << object_normal->points <<std::endl;
    
    // Compute shape features
    //std::cout<<"Compute shape features section"<< endl;
    PointCloud<FeatureT>::Ptr object_features(new PointCloud<FeatureT>);
    PointCloud<FeatureT>::Ptr scene_features(new PointCloud<FeatureT>);
    {
        //ScopeTime t("Shape features");
        
        //pcl::SHOTEstimation<PointT,PointT,ShotFeature> spin;
        SpinImageEstimation<PointT,PointT,FeatureT> spin;
        spin.setRadiusSearch(radiusSearch);
        //spin.setKSearch(kSearch);


        //std::cout<<"Compute shape features section - Before object"<< endl;
        spin.setInputCloud(object);
        spin.setInputNormals(object);
        spin.compute(*object_features);
        
        //std::cout<<"Compute shape features section - Before scene"<< endl;
        spin.setInputCloud(scene);
        spin.setInputNormals(scene);
        spin.compute(*scene_features);
    }
    
    // Find feature matches
    //std::cout<<"find features section"<< endl;
    //std::cout << "Number of features: "<<object_features->size() << endl;
    Correspondences corr(object_features->size());
    {
        //ScopeTime t("Feature matches");
        for(size_t i = 0; i < object_features->size(); ++i) {
        	//std::cout << i << std::endl;
        	//std::cout << "Object: " << object_features->points[i] << endl;
        	//std::cout << "Scene: " << scene_features->points[i] << endl;
            corr[i].index_query = i;
            nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
        }
    }
    
    /*
    std::cout<<"show mathces section"<< endl;
    // Show matches
    {
        PCLVisualizer v("Matches");
        v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.addCorrespondences<PointT>(object, scene, corr, 1);
        v.spin();
    }
    */
    
    //std::cout<<"k-d section"<< endl;
    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);
    
    // Set RANSAC parameters
    const size_t iter = RANSACIter;
    const float thressq = 0.01 * 0.01;
    
    //std::cout<<"start RANSAC section"<< endl;
    // Start RANSAC
    Matrix4f pose = Matrix4f::Identity();
    Matrix4f poseRotate = Matrix4f::Identity();
    PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>);
    PointCloud<PointNormal>::Ptr object_aligned_afine(new PointCloud<PointNormal>);
    float penalty = FLT_MAX;
    {
        //ScopeTime t("RANSAC");
        //cout << "Starting RANSAC..." << endl;
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
        //cout << "Got the following pose:" << endl << pose << endl;
        //cout << "Inliers: " << inliers << "/" << object->size() << endl;
        //cout << "RMSE: " << rmse << endl;
        ofstream outfile;
        outfile.open("data1.txt", std::ios_base::app);
        outfile <<"Radius: " << radiusSearch << " K: "<< kSearch <<" Inliers: " << inliers << " rmse: " << rmse << endl;
        outfile.close();

        
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

    // Show result

    /*
    {
        PCLVisualizer v("After global alignment");
        v.addPointCloud<PointT>(object_aligned, PointCloudColorHandlerCustom<PointT>(object_aligned, 0, 255, 0), "object_aligned");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        //v.addPointCloud<PointT>(object_aligned_afine, PointCloudColorHandlerCustom<PointT>(scene, 0, 0, 255),"Rotated");
        v.spin();
    }
    */
    
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
    const size_t iter =  500;
    const float thressq = 0.01 * 0.01;
    
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
    
        // Print pose
        //cout << "Got the following pose:" << endl << pose << endl;
        //cout << "Inliers: " << inliers << "/" << object->size() << endl;
        //cout << "RMSE: " << rmse << endl;

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

