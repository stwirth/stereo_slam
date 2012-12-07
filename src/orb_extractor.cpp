#include "stereo_slam/orb_extractor.h"
#include "stereo_slam/stereo_matcher.h"
#include "stereo_slam/features3d.h"
#include "stereo_slam/types.h"

// default settings
static const int NUM_FEATURES = 1000;
static const float MAX_DIST_2 = 10.0 * 10.0;

stereo_slam::ORBExtractor::ORBExtractor() :
  orb_(NUM_FEATURES)
{}

stereo_slam::Features3D stereo_slam::ORBExtractor::extract(
    const cv::Mat& image_left,
    const cv::Mat& image_right,
    const StereoCameraModel& camera_model) 
{
  // detect key points for both images
  std::vector<cv::KeyPoint> key_points_left, key_points_right;
  cv::Mat descriptors;
  orb_(image_left, cv::noArray(), key_points_left, descriptors);
  orb_(image_right, cv::noArray(), key_points_right, cv::noArray());

  // perform stereo matching
  StereoMatcher matcher;
  std::vector<cv::DMatch> matches;
  matcher.match(image_left, image_right,
      key_points_left, key_points_right, MAX_DIST_2, matches);

  // extract descriptors, key points, and 3D for matched points
  Features3D features;
  features.descriptors.create(matches.size(), descriptors.cols,
      descriptors.type());
  features.key_points.resize(matches.size());
  features.points.resize(matches.size());
  for (size_t i = 0; i < matches.size(); ++i)
  {
    const cv::KeyPoint& kp_left = key_points_left[matches[i].trainIdx];
    const cv::KeyPoint& kp_right = key_points_right[matches[i].queryIdx];
    features.key_points[i] = kp_left;
    descriptors.row(matches[i].trainIdx).copyTo(
        features.descriptors.row(i));
    float disparity = kp_left.pt.x - kp_right.pt.x;
    cv::Point2d left_uv(kp_left.pt.x, kp_left.pt.y);
    camera_model.projectDisparityTo3d(
        left_uv, disparity, features.points[i]);
  }

  return features;
}

