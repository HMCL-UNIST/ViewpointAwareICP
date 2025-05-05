
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h> 

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>

#include <chrono>
// #include <thread>
#include "viewpointAwareICP.h"

// using PointType = PointType;
using PointCloudT = pcl::PointCloud<PointType>;


using namespace std;

int main(){
    // This sample PCD pair is from the loop closure scene of LeGO-LOAM using the KITTI 05 sequence.
    std::string target_path = "/home/js/ViewpointAwareICP/pcd/tgt.pcd";
    std::string source_path = "/home/js/ViewpointAwareICP/pcd/src.pcd";

    pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr tgt_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(source_path, *src_cloud) == -1) {
        std::cerr << "Could not read source file: " << source_path << std::endl;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(target_path, *tgt_cloud) == -1) {
        std::cerr << "Could not read source file: " << target_path << std::endl;
    }
    
    Eigen::Matrix4d mat_;
    mat_ << 1.0,0.0,0.0,0.0,
            0.0,1.0,0.0,0.0,
            0.0,0.0,1.0,0.0,
            0.0,0.0,0.0,1.0;
    Eigen::Isometry3d init_guess;
    init_guess.matrix() = mat_;

    ViewpointAwareICP vicp;
    vicp.setInputSource(src_cloud);
    vicp.setInputTarget(tgt_cloud);
    vicp.setInitialGuess(init_guess);
    vicp.setMaxCorrespondenceDistance(50.0);
    vicp.setMaximumIterations(50);
    vicp.setTransformationEpsilon(1e-6);
    vicp.setEuclideanFitnessEpsilon(0.5);

    vicp.isValidSourcePose(true);

    // Since Target point cloud is an accumulation of a sequence of keyframe point cloud from lego loam, it does not have certain viewpoint! 
    vicp.isValidTargetPose(false);

    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    vicp.align(*unused_result);

    Eigen::Isometry3d result;
    result = vicp.getFinalTransformation();

    return 0;
}
