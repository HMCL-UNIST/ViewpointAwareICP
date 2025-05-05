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
    ne.setKSearch(10);
    ne.compute(*normals);

    pcl::PointCloud<PointType>::Ptr cloud_with_normals(new pcl::PointCloud<PointType>());
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    
    return cloud_with_normals;
}

void ViewpointAwareICP::setInputTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    if (!cloud || cloud->empty()) {
        std::cerr << "Input target cloud is empty!" << std::endl;
        return;
    }
    targetCloud_ = computeNormals(cloud);
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
    initSrc_ = true;
}


void ViewpointAwareICP::setInitialGuess(Eigen::Isometry3d initialGuess) {
    initialGuess_ = initialGuess;
    inv_initialGuess_ = initialGuess_.inverse();
    initGuess_ = true;
}

void ViewpointAwareICP::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, Eigen::Matrix4d transform_matrix){
    for (auto& pt : cloudIn->points) {
        Eigen::Vector4d point(pt.x, pt.y, pt.z, 1.0);  // Homogeneous 좌표로 변환
        Eigen::Vector4d transformed_point = transform_matrix * point;  // 변환 적용
        
        pt.x = transformed_point.x();
        pt.y = transformed_point.y();
        pt.z = transformed_point.z();
    }
}

double ViewpointAwareICP::getFitnessScoreNupdateWeights(pcl::CorrespondencesPtr corres, std::vector<double> weights){
    double sum_distance = 0.0;

    for (size_t i = 0; i < corres->size(); ++i)
    {
        float d = corres->at(i).distance;
        corres->at(i).weight = static_cast<float>(weights[corres->at(i).index_match]);;
        sum_distance += std::sqrt(d);
    }

    return sum_distance / static_cast<double>(corres->size());
}

double ViewpointAwareICP::getFitnessScoreNupdateUniformWeights(pcl::CorrespondencesPtr corres){
    double sum_distance = 0.0;

    for (size_t i = 0; i < corres->size(); ++i)
    {
        float d = corres->at(i).distance;
        corres->at(i).weight = 1.0f;
        sum_distance += std::sqrt(d);
    }

    return sum_distance / static_cast<double>(corres->size());
}


void ViewpointAwareICP::align(pcl::PointCloud<PointType>& result_pc) {
    if (!initSrc_ || !initTgt_ || !initGuess_) {
        std::cerr << "Source or Target not initialized!" << std::endl;
        return;
    }

    pcl::PointCloud<PointType>::Ptr temp(new pcl::PointCloud<PointType>());
    pcl::transformPointCloudWithNormals(*sourceCloud_, *temp, initialGuess_.matrix());
    sourceCloud_ = temp;

    V_SCORE score_s(initialGuess_.matrix().block<3,1>(3,0), sourceCloud_);
    V_SCORE score_t(Eigen::Vector3d::Zero(), targetCloud_);

    // std::vector<double> src_scores, tgt_scores;
    src_scores.clear();
    tgt_scores.clear();

    *sourceCloud_ = *score_s.getTopScoredCloud(0.6, src_scores);
    *targetCloud_ = *score_t.getTopScoredCloud(0.6, tgt_scores); 

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
        // std::vector<float> weights(targetCloud_->points.size(), 1.0f);

        if(!second_stage) fitness_score_ = getFitnessScoreNupdateUniformWeights(correspondences);
        else fitness_score_ = getFitnessScoreNupdateWeights(correspondences, tgt_scores);
        // fitness_score_ = getFitnessScoreNupdateUniformWeights(correspondences);
        // // fitness_score_ = getFitnessScoreNupdateWeights(correspondences, tgt_scores);

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
        // if (trans_diff_norm < eps_transform_ && fitness_score_ < eps_euclidean_fitness_)
            break;
        }
        // if (trans_diff_norm < eps_transform_*2 && fitness_score_ < eps_euclidean_fitness_*2){
        if (trans_diff_norm < 0.001){
            // break;     
            // auto uv_score = score_s.getUncertainVisiblityScores(sourceCloud_, targetCloud_);
            // auto tgt_uv_score = score_s.getUncertainVisiblityScores(targetCloud_,sourceCloud_);

            second_stage = true;
            std::cout << "second" << std::endl;
        }
        if(second_stage){
            auto tgt_uv_score = score_t.getUncertainVisiblityScores(targetCloud_,sourceCloud_, (initialGuess_.matrix()* T_total.cast<double>()).block<3,1>(0,3));
            tgt_scores = tgt_uv_score;
        }

        iter++;
    }

    transformPointCloud(sourceCloud_, initialGuess_.matrix());
    final_transformation_.matrix() = initialGuess_.matrix() * T_total.cast<double>();
    // final_transformation_ = final_transformation_ * initialGuess_;
    result_pc = *sourceCloud_; 
}
