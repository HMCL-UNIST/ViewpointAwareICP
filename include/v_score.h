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
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

// #include "progressbar.h"

static bool DEBUG_FILE = false;
using PointType = pcl::PointXYZINormal;

class V_SCORE{
    private:

        Eigen::Vector3d camera_position_;

        pcl::PointCloud<PointType>::Ptr original_cloud_;
        pcl::PointCloud<PointType>::Ptr input_cloud_;
        pcl::PointCloud<PointType>::Ptr projected_cloud_;
        pcl::PointCloud<pcl::Normal>::Ptr normals_;

        pcl::search::KdTree<PointType>::Ptr projected_kdtree_;

        std::vector<double> original_dist_vec_;
        std::vector<double> vis_score_vec_;
        std::vector<double> normal_score_vec_;
        std::vector<double> mix_score_vec_;
        std::vector<Eigen::Vector3d> color_vec_;
        std::vector<double> planar_score_vec_;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver_;

        std::ofstream normal_score_file_,vis_score_file_,mix_score_file_;

        bool debug_file_ = DEBUG_FILE;

        void initializeVariables(Eigen::Vector3d camera_position, pcl::PointCloud<PointType>::Ptr pointcloud);

        void denoisePointCloud();

        void estimateNormals();

        void setVectorSize();

        void projectPointCloud();

        void projectPointCloud(pcl::PointCloud<PointType>::Ptr in_cloud, Eigen::Vector3d camera_position, pcl::search::KdTree<PointType>::Ptr in_tree, std::vector<double> &dist_vec);

        void projectPointCloud(pcl::PointCloud<PointType>::Ptr &in_cloud, Eigen::Vector3d camera_position);
        /// \brief  Calculate  Score. 
        ///         Visibility, Planar, Noraml
        ///
        /// \todo   How to combine 3 different score in elegant way,
        void calculateScore();

        /// \brief  Calculate Normal Score. To consider its geometry attribute.
        ///         Use Line Of Sight Vector and Normal Vector of the point,
        ///         Now, just use angle between two vectors
        ///
        /// \ref   Jean-Emmanuel deschaud, "IMLS-SLAM, scan-to-model mathjcing based on 3D data"
        ///
        /// \param  idx : index of the point
        ///
        /// \todo   Make some transition function from angle to score (Activate function?)
        ///         Any other approach to arrange lower score to ground point? (product / cross with normal vector?
        inline double calculateNormalScore(int idx);

        /// \brief  Calculate Visibility Score. 
        ///         To consider whehter the point is occluded, spatial attribute(?)
        ///         
        /// \ref    Pierre Biasutti, "Visibility estimation in point clouds with variable density"
        ///
        /// \param  idx : index of the point
        inline double calculateVisibilityScore(int idx);

        /// \brief  Calculate Planar Score. 
        ///         To consider its geometry attribute, to lower the influence of noisy points (ex, leaves)
        ///         
        /// \ref    Jerome Demantke, "Dimensionality Based Scale Selection in 3D Lidar Point Clouds"
        ///
        /// \param  idx : index of the point
        ///
        /// \todo   Fine tuning the KnnSearchParam
        inline double calculatePlanarScore(int idx);

        void colorizeResult();

        void setFile(std::string path);
        
        void closeFile();

        inline double sigmoid(double x, double a=0){
            return 1/(1+exp(-(x+a)));
        }

        inline double activate_function(double x, double criteria = 0){
            return x < criteria ? x : criteria+pow(x-criteria,2);
        }

    public:
        // V_SCORE(Eigen::Vector3d camera_position, pcl::PointCloud<PointType>::Ptr pointcloud);
        V_SCORE(Eigen::Vector3d camera_position, pcl::PointCloud<PointType>::Ptr pointcloud);
        V_SCORE(){}

        void initialize(Eigen::Vector3d camera_position, pcl::PointCloud<PointType>::Ptr pointcloud);

        void visualizeResult();

        inline double getScore(int idx);

        std::vector<double> getScores();

        pcl::PointCloud<PointType>::Ptr getCloud();

        std::vector<int> getTopIndex(int num);

        // pcl::PointCloud<PointType>::Ptr getTopScoredCloud(int num);
        pcl::PointCloud<PointType>::Ptr getTopScoredCloud(int num,  std::vector<double> &scores, pcl::PointCloud<pcl::Normal>::Ptr);

        pcl::PointCloud<PointType>::Ptr getTopScoredCloud(int num, pcl::PointCloud<PointType>::Ptr pointcloud, std::vector<double> weight);

        pcl::PointCloud<PointType>::Ptr getTopScoredCloud(int num, std::vector<double> &scores);
        
        pcl::PointCloud<PointType>::Ptr getTopScoredCloud(double ratio, std::vector<double> &scores);

        pcl::PointCloud<PointType>::Ptr getTopScoredCloudByScore(double score);

        std::vector<double> getCrossScores(std::vector<Eigen::Vector3d> &src_pt, std::vector<Eigen::Vector3d> &tgt_pt, Eigen::Vector3d src_pose, Eigen::Vector3d tgt_pose);

        void visualizeWeight(const pcl::PointCloud<PointType>::Ptr pointcloud, const std::vector<double> weight);

        void visualizeWeight(const pcl::PointCloud<PointType>::Ptr pointcloud, const std::vector<double> weight,const pcl::PointCloud<PointType>::Ptr pointcloud2);
        
        std::vector<double> getUncertainVisiblityScores(const pcl::PointCloud<PointType>::Ptr source_cloud, const pcl::PointCloud<PointType>::Ptr target_cloud, Eigen::Vector3d position={0,0,0}, const double std_score=0.4, const double std_perc=0.1);

};