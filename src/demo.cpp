
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
    // This sample PCD pair is from the first loop closure scene of LeGO-LOAM using the KITTI 05 sequence.
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

    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    vicp.align(*unused_result);

    Eigen::Isometry3d result;
    result = vicp.getFinalTransformation();

    //////////////////// Point-to-Point ICP ///////////////////////
    // pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    // icp.setMaxCorrespondenceDistance(50);
    // icp.setMaximumIterations(50);
    // icp.setTransformationEpsilon(1e-6);
    // icp.setEuclideanFitnessEpsilon(1e-6);
    // icp.setRANSACIterations(0);

    // // Align clouds using vanilla ICP
    // icp.setInputSource(src_cloud);
    // icp.setInputTarget(tgt_cloud);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZI>());
    // icp.align(*icp_result); 

    // Eigen::Matrix4f trans = icp.getFinalTransformation();
    // Eigen::Affine3f final_result(trans);

    //////////////////// Generalized ICP ///////////////////////
    // pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    // gicp.setMaxCorrespondenceDistance(50);
    // gicp.setMaximumIterations(50);
    // gicp.setTransformationEpsilon(1e-6);
    // gicp.setEuclideanFitnessEpsilon(1e-6);

    // gicp.setInputSource(src_cloud);
    // gicp.setInputTarget(tgt_cloud);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr gicp_result(new pcl::PointCloud<pcl::PointXYZI>());
    // gicp.align(*gicp_result);

    // if (gicp.hasConverged()) {
    //     std::cout << "GICP converged." << std::endl;
    //     std::cout << "Transformation:" << std::endl << gicp.getFinalTransformation() << std::endl;
    // } else {
    //     std::cout << "GICP did not converge." << std::endl;
    // }

    // trans = gicp.getFinalTransformation();
    // Eigen::Affine3f final_result2(trans);


    //////////////////// Point-to-Plane ICP ///////////////////////
    // pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    // pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    // ne.setSearchMethod(tree);
    // ne.setKSearch(10);

    // pcl::PointCloud<pcl::Normal>::Ptr src_normals(new pcl::PointCloud<pcl::Normal>());
    // ne.setInputCloud(src_cloud);
    // ne.compute(*src_normals);
    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr src_cloud_normals(new pcl::PointCloud<pcl::PointXYZINormal>());
    // pcl::concatenateFields(*src_cloud, *src_normals, *src_cloud_normals);

    // pcl::PointCloud<pcl::Normal>::Ptr tgt_normals(new pcl::PointCloud<pcl::Normal>());
    // ne.setInputCloud(tgt_cloud);
    // ne.compute(*tgt_normals);
    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr tgt_cloud_normals(new pcl::PointCloud<pcl::PointXYZINormal>());
    // pcl::concatenateFields(*tgt_cloud, *tgt_normals, *tgt_cloud_normals);

    // pcl::IterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> icp_ptp;
    // icp_ptp.setTransformationEstimation(
    //     boost::shared_ptr<pcl::registration::TransformationEstimationPointToPlane<pcl::PointXYZINormal, pcl::PointXYZINormal> >(
    //         new pcl::registration::TransformationEstimationPointToPlane<pcl::PointXYZINormal, pcl::PointXYZINormal>()));
    // icp_ptp.setMaxCorrespondenceDistance(50);
    // icp_ptp.setMaximumIterations(50);
    // icp_ptp.setTransformationEpsilon(1e-6);
    // icp_ptp.setEuclideanFitnessEpsilon(1e-6);
    // icp_ptp.setRANSACIterations(0);
    // icp_ptp.setInputSource(src_cloud_normals);
    // icp_ptp.setInputTarget(tgt_cloud_normals);
    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptp_result(new pcl::PointCloud<pcl::PointXYZINormal>());
    // icp_ptp.align(*ptp_result);

    // if (icp_ptp.hasConverged()) {
    //     std::cout << "Point-to-Plane ICP converged." << std::endl;
    //     std::cout << "Point-to-Plane ICP Transformation:" << std::endl << icp_ptp.getFinalTransformation() << std::endl;
    // } else {
    //     std::cout << "Point-to-Plane ICP did not converge." << std::endl;
    // }

    // pcl::PointCloud<pcl::PointXYZI>::Ptr ptp_result_xyz(new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::copyPointCloud(*ptp_result, *ptp_result_xyz);


    return 0;
}
