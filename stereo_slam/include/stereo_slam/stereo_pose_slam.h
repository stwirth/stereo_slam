#ifndef _STEREO_POSE_SLAM_H_
#define _STEREO_POSE_SLAM_H_

namespace stereo_slam
{

class StereoPoseSlam
{

public:

  void update(const cv::Mat& image_left, const cv::Mat& image_right, 
      const Eigen::Isometry3d& pose_estimate, const ros::Time& stamp);

private:

  image_geometry::StereoCameraModel camera_model_;
  boost::shared_ptr<KeyFrameSelectionPolicy> key_frame_selection_policy_;
  g2o::SparseOptimizer graph_;
  int next_vertex_id_;

};

}

#endif
