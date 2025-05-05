#include <open3d/Open3D.h>
#include <open3d/geometry/Qhull.h>
#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/visualization/visualizer/Visualizer.h>
#include <open3d/visualization/utility/DrawGeometry.h>
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


    pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud_initguess(new pcl::PointCloud<pcl::PointXYZI>());
    // *tgt_cloud_initguess = *src_cloud;
    Eigen::Matrix4f tf = init_guess.matrix().cast<float>();
    pcl::transformPointCloud(*src_cloud, *src_cloud_initguess, tf);

    auto source_point = std::make_shared<open3d::geometry::PointCloud>();
    auto target_point = std::make_shared<open3d::geometry::PointCloud>();
    auto result_point = std::make_shared<open3d::geometry::PointCloud>();

    auto source_point_cp = std::make_shared<open3d::geometry::PointCloud>();
    auto source_point_cp2 = std::make_shared<open3d::geometry::PointCloud>();
    auto target_point_cp = std::make_shared<open3d::geometry::PointCloud>();

    for (const auto& pt : src_cloud_initguess->points) {
        source_point->points_.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
        source_point_cp->points_.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
        source_point_cp2->points_.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
    }
    for (const auto& pt : tgt_cloud->points) {
        target_point->points_.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
    }
    for (const auto& pt : unused_result->points) {
        result_point->points_.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
    }
    auto cam = open3d::geometry::TriangleMesh::CreateCoordinateFrame(5., Eigen::Vector3d(0, 0, 0));
    source_point->PaintUniformColor({1,0,0}); 
    target_point->PaintUniformColor({0,1,0}); 
    source_point_cp->PaintUniformColor({1,0,0}); 
    source_point_cp2->PaintUniformColor({1,0,0}); 
    result_point->PaintUniformColor({1,0,0}); 

    target_point_cp = target_point;
    
    open3d::visualization::DrawGeometriesWithCustomAnimation({source_point, target_point, cam}, "VICP init", 1080,720);

    auto target_point_weights = std::make_shared<open3d::geometry::PointCloud>();
    pcl::PointCloud<PointType>::Ptr filtered_target_cloud;
    vicp.get_target_cloud(filtered_target_cloud);  

    for (const auto& pt : filtered_target_cloud->points) {
        target_point_weights->points_.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
    }

    std::vector<double> target_scores;
    target_scores = vicp.get_target_visibility_score();

    std::vector<Eigen::Vector3d> color_vec_;

    for (int idx=0; idx<target_scores.size(); idx++){
        auto score = target_scores[idx];
        double ratio = 2*score;
        // double ratio = score;
        // Closer to Red, Lower value.
        double r = ((1-ratio) < 0) ? 0.0 : (1-ratio);
        double b = ((ratio - 1) < 0) ? 0.0 : (ratio-1);
        double g = 1-b-r;
        color_vec_.push_back(Eigen::Vector3d(r, g, b));
    }

    target_point_weights->colors_ = color_vec_;


    auto source_position = open3d::geometry::TriangleMesh::CreateCoordinateFrame(5., result.matrix().block<3,1>(0,3));

    source_point->PaintUniformColor({1,1,1}); 
    // open3d::visualization::DrawGeometriesWithCustomAnimation({target_point_weights, source_point, source_position}, "Vis Weight", 1080,720);
    source_point->PaintUniformColor({1,0,0}); 

    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow("1st Stage",1280,720,50,50,true);
    vis.AddGeometry({source_point});
    vis.AddGeometry({target_point});
    // vis.AddGeometry({correspondence_line_set});
    vis.Run();
    std::vector<Eigen::Matrix4f> first_stage_transformations, second_stage_transformations;
    first_stage_transformations = vicp.get_first_stage_transformations();
    second_stage_transformations = vicp.get_second_stage_transformations();

    for(int i=0; i<first_stage_transformations.size(); i++){
        source_point->Transform(first_stage_transformations[i].cast<double>());
        vis.UpdateGeometry({source_point});
        vis.PollEvents();
        vis.UpdateRender();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    vis.DestroyVisualizerWindow();

    open3d::visualization::Visualizer vis2;
    vis2.CreateVisualizerWindow("2nd Stage",1280,720,50,50,true);
    vis2.AddGeometry({source_point});
    vis2.AddGeometry({target_point});
    vis2.Run();

    for(int i=0; i<second_stage_transformations.size(); i++){
        source_point->Transform(second_stage_transformations[i].cast<double>());
        vis2.UpdateGeometry({source_point});
        vis2.PollEvents();
        vis2.UpdateRender();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    vis2.DestroyVisualizerWindow();

    open3d::visualization::DrawGeometriesWithCustomAnimation({result_point, target_point}, "VICP result", 1080,720);

    open3d::visualization::DrawGeometriesWithCustomAnimation({target_point_weights, source_position}, "Visibility Score of Target Point Cloud relative to result source pose", 1080,720);


    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setMaxCorrespondenceDistance(50);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds using vanilla ICP
    icp.setInputSource(src_cloud_initguess);
    icp.setInputTarget(tgt_cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZI>());
    icp.align(*icp_result); 

    Eigen::Matrix4f trans = icp.getFinalTransformation();
    Eigen::Affine3f final_result(trans);
    source_point_cp->Transform(final_result.matrix().cast<double>());

    open3d::visualization::Visualizer vis3;
    vis3.CreateVisualizerWindow("Vanila ICP Result",1280,720,50,50,true);
    vis3.AddGeometry({source_point_cp});
    vis3.AddGeometry({target_point_cp});
    vis3.Run();

    while (vis3.PollEvents() && vis2.PollEvents()) {
        vis3.UpdateRender();
        vis2.UpdateRender();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    vis3.DestroyVisualizerWindow();

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setMaxCorrespondenceDistance(50);
    gicp.setMaximumIterations(50);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);

    gicp.setInputSource(src_cloud_initguess);
    gicp.setInputTarget(tgt_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr gicp_result(new pcl::PointCloud<pcl::PointXYZI>());
    gicp.align(*gicp_result);

    if (gicp.hasConverged()) {
        std::cout << "GICP converged." << std::endl;
        std::cout << "Transformation:" << std::endl << gicp.getFinalTransformation() << std::endl;
    } else {
        std::cout << "GICP did not converge." << std::endl;
    }

    trans = gicp.getFinalTransformation();
    Eigen::Affine3f final_result2(trans);
    source_point_cp2->Transform(final_result2.matrix().cast<double>());

    open3d::visualization::Visualizer vis4;
    vis4.CreateVisualizerWindow("Generalized ICP Result",1280,720,50,50,true);
    vis4.AddGeometry({source_point_cp2});
    vis4.AddGeometry({target_point_cp});
    vis4.Run();
    vis4.DestroyVisualizerWindow();


    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    ne.setSearchMethod(tree);
    ne.setKSearch(10);

    pcl::PointCloud<pcl::Normal>::Ptr src_normals(new pcl::PointCloud<pcl::Normal>());
    ne.setInputCloud(src_cloud_initguess);
    ne.compute(*src_normals);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr src_cloud_normals(new pcl::PointCloud<pcl::PointXYZINormal>());
    pcl::concatenateFields(*src_cloud_initguess, *src_normals, *src_cloud_normals);

    pcl::PointCloud<pcl::Normal>::Ptr tgt_normals(new pcl::PointCloud<pcl::Normal>());
    ne.setInputCloud(tgt_cloud);
    ne.compute(*tgt_normals);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr tgt_cloud_normals(new pcl::PointCloud<pcl::PointXYZINormal>());
    pcl::concatenateFields(*tgt_cloud, *tgt_normals, *tgt_cloud_normals);

    pcl::IterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> icp_ptp;
    icp_ptp.setTransformationEstimation(
        boost::shared_ptr<pcl::registration::TransformationEstimationPointToPlane<pcl::PointXYZINormal, pcl::PointXYZINormal> >(
            new pcl::registration::TransformationEstimationPointToPlane<pcl::PointXYZINormal, pcl::PointXYZINormal>()));
    icp_ptp.setMaxCorrespondenceDistance(50);
    icp_ptp.setMaximumIterations(50);
    icp_ptp.setTransformationEpsilon(1e-6);
    icp_ptp.setEuclideanFitnessEpsilon(1e-6);
    icp_ptp.setRANSACIterations(0);
    icp_ptp.setInputSource(src_cloud_normals);
    icp_ptp.setInputTarget(tgt_cloud_normals);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptp_result(new pcl::PointCloud<pcl::PointXYZINormal>());
    icp_ptp.align(*ptp_result);

    if (icp_ptp.hasConverged()) {
        std::cout << "Point-to-Plane ICP converged." << std::endl;
        std::cout << "Point-to-Plane ICP Transformation:" << std::endl << icp_ptp.getFinalTransformation() << std::endl;
    } else {
        std::cout << "Point-to-Plane ICP did not converge." << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptp_result_xyz(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*ptp_result, *ptp_result_xyz);
    auto o3d_ptp = std::make_shared<open3d::geometry::PointCloud>();
    for (const auto &pt : ptp_result_xyz->points) {
        o3d_ptp->points_.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
    }
    o3d_ptp->PaintUniformColor({1, 0, 0});
    open3d::visualization::DrawGeometriesWithCustomAnimation({o3d_ptp, target_point}, "Point-to-Plane ICP Result", 1080,720);
    
    return 0;
}
