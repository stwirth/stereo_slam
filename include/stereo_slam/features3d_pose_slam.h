#ifndef _FEATURES3D_POSE_SLAM_H_
#define _FEATURES3D_POSE_SLAM_H_

#include <g2o/core/sparse_optimizer.h>
#include <Eigen/Geometry>

#include "stereo_slam/types.h"

namespace stereo_slam
{

class PoseNode;
class Features3D;
class MatchCandidatePolicy;
class FeatureExtractor;

/**
 * Example usage:
 * @code 
 *   if (key_frame_selection_policy_->isKeyFrame(...)
 *   {
 *     Features3D features = feature_extractor_->extract(left_image, right_image, 
 *       cam_model);
 *     slam_->update(features, pose_estimate, stamp);
 *   }
 */
class Features3DPoseSlam
{

public:

  /**
   * Constructs a new instance for stereo pose slam, initializes
   * some members.
   */
  Features3DPoseSlam();

  /**
   * Main update method. 
   */
  void update(const Features3D& current_features, 
      const Eigen::Isometry3d& pose_estimate, const TimeStamp& stamp);

protected:

  /**
   * Looks for match candidates for given node and returns them in
   * a vector.
   */
  std::vector<PoseNode*> findMatchCandidates(const PoseNode* node);

  /**
   * Finds the transform between two nodes by matching their features.
   * @param node1 source node
   * @param node2 destination node
   * @param[out] transform transform from source to destination node
   * @param[out] num_inliers number of inliers for found transform
   * @return true if transform was found, false otherwise
   */
  bool findTransform(const PoseNode* node1, const PoseNode* node2,
      Eigen::Isometry3d* transform, int* num_inliers);

  bool haveOverlap(const PoseNode* node1, const PoseNode* node2) const;

private:

  boost::shared_ptr<MatchCandidatePolicy> match_candidate_policy_;
  g2o::SparseOptimizer graph_optimizer_;
  int next_vertex_id_;

};

}

#endif

