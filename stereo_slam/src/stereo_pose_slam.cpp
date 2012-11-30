#include <ros/ros.h>

stereo_slam::StereoPoseSlam::StereoPoseSlam() :
  next_vertex_id_(0)
{

  g2o::BlockSolver_6_3::LinearSolverType* linearSolver = 
    new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
  graph.setSolver(linearSolver);
}


void stereo_slam::StereoPoseSlam::update(
    const cv::Mat& image_left, const cv::Mat& image_right,
    const Eigen::Isometry3d& pose_estimate, const ros::Time& stamp)
{
  if (key_frame_selection_policy_->isNewKeyFrame(
        image_left, image_right, pose_estimate, stamp))
  {
    ROS_INFO("Adding new key frame...");
    Features3D features = feature_extractor_->extract(
        image_left, image_right, camera_model_);
    ROS_INFO("Extracted %zu features.", features.key_points.size());

    PoseNode* node = new PoseNode();
    node->setFeatures(features);
    node->setEstimate(pose_estimate);
    node->setId(next_vertex_id_);
    if (next_vertex_id_ == 0)
    {
      node->setFixed(true);
    }
    ++next_vertex_id_;
    graph.addVertex(node);

    std::vector<PoseNode*> match_candidates =
      computeMatchCandidates(node);
    ROS_INFO("Found %zu match candidates.", match_candidates.size());
    bool edge_added = false;
    for (size_t i = 0; i < match_candidates.size(); ++i)
    {
      const PoseNode* candidate = match_canditates[i];
      Eigen::Isometry3d transform;
      int num_inliers;
      bool transform_found = 
        findTransform(node->features(), 
            candidate->features(),
            transform, num_inliers);
      if (transform_found)
      {
        ROS_INFO("Found transform to candidate %i with %i inliers. "
                 "Adding edge.", i, num_inliers);
        // create new edge
        boost::shared_ptr<g2o::EdgeSE3> edge(new g2o::EdgeSE3());
        edge->setVertex(0, node);
        edge->setVertex(1, candidate);
        edge->setMeasurement(transform);
        graph.addEdge(edge);
        edge_added = true;
      }
      else
      {
        ROS_INFO("No transform found to candidate %i.", i);
      }
    }

    if (edge_added)
    {
      ROS_INFO("Optimizing global pose graph...");
      graph.initializeOptimization();
      graph.optimize(10);
      ROS_INFO("Optimizing done.");
      updateMapTf();

    }
  } 
  else
  {
    ROS_INFO("Not a key frame, skipping update.");
  }
}

