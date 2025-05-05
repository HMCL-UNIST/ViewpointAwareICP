#include "viewpointAwareICP.h"

ViewpointAwareICP::ViewpointAwareICP(){
    targetCloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    sourceCloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
}

pcl::PointCloud<PointType>::Ptr ViewpointAwareICP::computeNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    ne.setSearchMethod(tree);
    ne.setKSearch(30);
    ne.compute(*normals);

    pcl::PointCloud<PointType>::Ptr cloud_with_normals(new pcl::PointCloud<PointType>());
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    
    return cloud_with_normals;
}

pcl::PointCloud<PointType>::Ptr ViewpointAwareICP::voxelDownsample(
    const pcl::PointCloud<PointType>::Ptr& cloud,
    float leaf_size)
{
    pcl::VoxelGrid<PointType> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);

    pcl::PointCloud<PointType>::Ptr downsampled(new pcl::PointCloud<PointType>());
    voxel.filter(*downsampled);
    return downsampled;
}

pcl::PointCloud<PointType>::Ptr ViewpointAwareICP::denoisePointCloud(
    const pcl::PointCloud<PointType>::Ptr& cloud,
    int mean_k,
    double stddev_mul_thresh)
{
    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_mul_thresh);

    auto filtered = boost::make_shared<pcl::PointCloud<PointType>>();
    sor.filter(*filtered);
    return filtered;
}

void ViewpointAwareICP::setInputTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    if (!cloud || cloud->empty()) {
        std::cerr << "Input target cloud is empty!" << std::endl;
        return;
    }
    targetCloud_ = computeNormals(cloud);

    targetCloud_ = voxelDownsample(targetCloud_, 0.2f);

    targetCloud_ = denoisePointCloud(targetCloud_, 3, 0.8);

    initTgt_ = true;
}

void ViewpointAwareICP::get_target_cloud(pcl::PointCloud<PointType>::Ptr& cloud) {
    cloud = targetCloud_;
}

void ViewpointAwareICP::setInputSource(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    if (!cloud || cloud->empty()) {
        std::cerr << "Input source cloud is empty!" << std::endl;
        return;
    }
    sourceCloud_ = computeNormals(cloud);

    sourceCloud_ = voxelDownsample(sourceCloud_, 0.2f);

    sourceCloud_ = denoisePointCloud(sourceCloud_, 3, 0.8);

    initSrc_ = true;
}


void ViewpointAwareICP::setInitialGuess(Eigen::Isometry3d initialGuess) {
    initialGuess_ = initialGuess;
    inv_initialGuess_ = initialGuess_.inverse();
    initGuess_ = true;
}

void ViewpointAwareICP::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, Eigen::Matrix4d transform_matrix){
    for (auto& pt : cloudIn->points) {
        Eigen::Vector4d point(pt.x, pt.y, pt.z, 1.0);
        Eigen::Vector4d transformed_point = transform_matrix * point;
        
        pt.x = transformed_point.x();
        pt.y = transformed_point.y();
        pt.z = transformed_point.z();
    }
}

double ViewpointAwareICP::getFitnessScoreNupdateWeightsSecondStage(pcl::CorrespondencesPtr corres){
    double sum_distance = 0.0;

    for (size_t i = 0; i < corres->size(); ++i)
    {
        float d = corres->at(i).distance;
        float w = 0.0f;

        if (is_valid_src_pose && is_valid_tgt_pose) {
            float src_w = static_cast<float>(src_scores[corres->at(i).index_query]);
            float tgt_w = static_cast<float>(tgt_scores[corres->at(i).index_match]);
            w = std::min(src_w, tgt_w);
        }
        else if (is_valid_src_pose && !is_valid_tgt_pose) {
            w = static_cast<float>(tgt_scores[corres->at(i).index_match]);
        }
        else if (!is_valid_src_pose && is_valid_tgt_pose) {
            w = static_cast<float>(src_scores[corres->at(i).index_query]);
        }

        corres->at(i).weight = w;

        sum_distance += std::sqrt(d);
    }

    return sum_distance / static_cast<double>(corres->size());
}

double ViewpointAwareICP::getFitnessScoreNupdateWeightsFirstStage(pcl::CorrespondencesPtr corres){
    double sum_distance = 0.0;

    for (size_t i = 0; i < corres->size(); ++i)
    {
        float d = corres->at(i).distance;
        float w = 0.0f;

        if (is_valid_src_pose && is_valid_tgt_pose) {
            float src_w = static_cast<float>(src_scores[corres->at(i).index_query]);
            float tgt_w = static_cast<float>(tgt_scores[corres->at(i).index_match]);
            w = std::max(src_w, tgt_w);
        }
        else if (is_valid_src_pose && !is_valid_tgt_pose) {
            w = static_cast<float>(src_scores[corres->at(i).index_query]);
            // w = 1.0f;
        }
        else if (!is_valid_src_pose && is_valid_tgt_pose) {
            w = static_cast<float>(tgt_scores[corres->at(i).index_match]);
        }

        corres->at(i).weight = w;

        sum_distance += std::sqrt(d);
    }

    return sum_distance / static_cast<double>(corres->size());
}

double ViewpointAwareICP::getFitnessScoreNupdateUniformWeights(pcl::CorrespondencesPtr corres){
    double sum_distance = 0.0;

    for (size_t i = 0; i < corres->size(); ++i)
    {
        float d = corres->at(i).distance;

        corres->at(i).weight = 1.f;

        sum_distance += std::sqrt(d);
    }

    return sum_distance / static_cast<double>(corres->size());
}

void ViewpointAwareICP::align(pcl::PointCloud<PointType>& result_pc) {
    if (!initSrc_ || !initTgt_ || !initGuess_) {
        std::cerr << "Source or Target not initialized!" << std::endl;
        return;
    }

    if (!is_valid_src_pose && !is_valid_tgt_pose) {
        std::cerr << "There is no valid pose information!!" << std::endl;
        return;
    }

    pcl::PointCloud<PointType>::Ptr temp(new pcl::PointCloud<PointType>());
    pcl::transformPointCloudWithNormals(*sourceCloud_, *temp, initialGuess_.matrix());
    sourceCloud_ = temp;
    // std::vector<double> src_scores, tgt_scores;
    src_scores.clear();
    tgt_scores.clear();

    if(is_valid_src_pose){
        score_s.initialize(initialGuess_.matrix().block<3,1>(3,0), sourceCloud_);
        *sourceCloud_ = *score_s.getTopScoredCloud(0.7, src_scores);
    }
    if(is_valid_tgt_pose){
        score_t.initialize(Eigen::Vector3d::Zero(), targetCloud_);
        *targetCloud_ = *score_t.getTopScoredCloud(0.7, tgt_scores); 
    }

    pcl::search::KdTree<PointType>::Ptr target_kdtree_(new pcl::search::KdTree<PointType>());
    target_kdtree_->setInputCloud(targetCloud_);

    Eigen::Isometry3d last_result = Eigen::Isometry3d::Identity();
    std::vector<Eigen::Vector3d> src_points, tgt_points, normals;
    std::vector<std::pair<int,int>> corres_pairs;
    std::vector<double> valid_scores;

	pcl::registration::CorrespondenceEstimation<PointType, PointType> corr_est;
	corr_est.setInputTarget(targetCloud_);
    pcl::PointCloud<PointType>::Ptr current_source(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*sourceCloud_, *current_source);

    // Weighted ICP Iteration
	Eigen::Matrix4f T_last = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_total = Eigen::Matrix4f::Identity();
    pcl::registration::TransformationEstimationPointToPlaneLLSWeighted<PointType, PointType>::Ptr transformation_estimation(
        new pcl::registration::TransformationEstimationPointToPlaneLLSWeighted<PointType, PointType>());

    first_transformation_deltas_.clear();
    second_transformation_deltas_.clear();

    int iter = 0;
    while (iter < max_iter_){
        corr_est.setInputSource(current_source);
        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
        corr_est.determineCorrespondences(*correspondences, corres_dist_);

        if (correspondences->empty())
        {
            std::cout << "No correspondences found at iteration " << iter << std::endl;
            break;
        }

        if(!second_stage) fitness_score_ = getFitnessScoreNupdateWeightsFirstStage(correspondences);
        // if(!second_stage) fitness_score_ = getFitnessScoreNupdateUniformWeights(correspondences);
        else fitness_score_ = getFitnessScoreNupdateWeightsSecondStage(correspondences);


        Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();
        transformation_estimation->estimateRigidTransformation(*current_source, *targetCloud_, *correspondences, delta);

        if(!second_stage) first_transformation_deltas_.push_back(delta);
        else second_transformation_deltas_.push_back(delta);
        T_total = delta * T_total;

        pcl::PointCloud<PointType>::Ptr temp(new pcl::PointCloud<PointType>());
        pcl::transformPointCloudWithNormals(*current_source, *temp, delta);
        current_source = temp;

        Eigen::Matrix4f diff = delta - T_last;
		T_last = delta;
        double trans_diff_norm = diff.norm();

        std::cout << "Iteration " << iter+1 
                  << ", transformation diff norm: " << trans_diff_norm 
                  << ", fitness score: " << fitness_score_ << std::endl;

        if (second_stage && trans_diff_norm < 0.00001){
            converged_ = true;
            break;
        }
        if (trans_diff_norm < 0.001){

            second_stage = true;
            std::cout << "second" << std::endl;
        }
        if(second_stage){
            if(is_valid_src_pose){
                auto tgt_uv_score = score_t.getUncertainVisiblityScores(targetCloud_,current_source, (initialGuess_.matrix()* T_total.cast<double>()).block<3,1>(0,3));
                tgt_scores = tgt_uv_score;
            }
            if(is_valid_tgt_pose){
                auto src_uv_score = score_s.getUncertainVisiblityScores(current_source,targetCloud_, Eigen::Vector3d::Zero());
                src_scores = src_uv_score;
            }
        }

        iter++;
    }

    final_transformation_.matrix() = initialGuess_.matrix() * T_total.cast<double>();

    result_pc = *current_source; 
}
