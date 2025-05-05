#include <iostream>       
#include <typeinfo>      
#include <ctime>
#include <fstream>
#include <cmath>
#include <tgmath.h>
#include <algorithm>
#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/statistical_outlier_removal.h>

// #include <pcl_conversions/pcl_conversions.h>
#include "v_score.h"
// #include "icp_closedform.h"
// #include "progressbar.h"

class ViewpointAwareICP{
private:
    using PointType = pcl::PointXYZINormal;

    pcl::PointCloud<PointType>::Ptr targetCloud_;
    pcl::PointCloud<PointType>::Ptr sourceCloud_;
    

    Eigen::Isometry3d final_transformation_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d initialGuess_;
    Eigen::Isometry3d inv_initialGuess_;
    

    bool initSrc_ = false;
    bool initTgt_ = false;
    bool initGuess_ = false;

    bool is_valid_src_pose = false;
    bool is_valid_tgt_pose = false;

    bool converged_ = false;

    double corres_dist_ = 10.0;
    int max_iter_ = 200;
    double eps_transform_ = 1e-6;
    double eps_euclidean_fitness_ = 1e-6;
    // void estimateNormalVec();
    bool second_stage = false;
    double fitness_score_ = std::numeric_limits<double>::max();
    std::vector<Eigen::Matrix4f> first_transformation_deltas_, second_transformation_deltas_;
    std::vector<double> src_scores, tgt_scores;

    V_SCORE score_s, score_t;
    
public:
    ViewpointAwareICP();

    void setInputTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr);
    void get_target_cloud(pcl::PointCloud<PointType>::Ptr&);

    void setInputSource(pcl::PointCloud<pcl::PointXYZI>::Ptr);
    void setInitialGuess(Eigen::Isometry3d);
    // pcl::PointCloud<PointType>::Ptr computeNormals(pcl::PointCloud<PointType>::Ptr cloud);
    pcl::PointCloud<PointType>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    pcl::PointCloud<PointType>::Ptr voxelDownsample(const pcl::PointCloud<PointType>::Ptr& cloud, float leaf_size);
    pcl::PointCloud<PointType>::Ptr denoisePointCloud(const pcl::PointCloud<PointType>::Ptr& cloud, int mean_k = 3, double stddev_mul_thresh = 0.8);
    double getFitnessScoreNupdateWeightsSecondStage(pcl::CorrespondencesPtr corres);
    double getFitnessScoreNupdateWeightsFirstStage(pcl::CorrespondencesPtr corres);
    double getFitnessScoreNupdateUniformWeights(pcl::CorrespondencesPtr corres);


    void setMaxCorrespondenceDistance(double corres_dist) {corres_dist_ = corres_dist;}
    void setMaximumIterations(int max_iter) {max_iter_ = max_iter;}
    void setTransformationEpsilon(double eps_transform) {eps_transform_ = eps_transform;}
    void setEuclideanFitnessEpsilon(double eps_euclidean_fitness) {eps_euclidean_fitness_ = eps_euclidean_fitness;}
    void align(pcl::PointCloud<PointType>& result_pc);
    void transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, Eigen::Matrix4d transformIn);

    void isValidSourcePose(bool is_valid) {is_valid_src_pose = is_valid;}
    void isValidTargetPose(bool is_valid) {is_valid_tgt_pose = is_valid;}


    bool hasConverged() {return converged_;}
    double getFitnessScore() {return fitness_score_;}
    Eigen::Isometry3d getFinalTransformation() {return final_transformation_;}
    std::vector<Eigen::Matrix4f> get_first_stage_transformations() const { return first_transformation_deltas_; }
    std::vector<Eigen::Matrix4f> get_second_stage_transformations() const { return second_transformation_deltas_; }
    std::vector<double> get_target_visibility_score() const { return tgt_scores; }
};