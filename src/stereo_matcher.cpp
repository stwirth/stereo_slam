#include "stereo_slam/image_proc.h"
#include "stereo_slam/stereo_matcher.h"

stereo_slam::StereoMatcher::Params::Params() :
  max_y_diff(2.0),
  min_disparity(1.0),
  max_disparity(10000.0),
  max_angle_diff(5.0),
  max_size_diff(10.0)
{}

stereo_slam::StereoMatcher::StereoMatcher()
{}

stereo_slam::StereoMatcher::StereoMatcher(const Params& params) :
  params_(params)
{}

bool stereo_slam::StereoMatcher::isMatch(
    const cv::Mat& image_left, const cv::Mat& image_right,
    const cv::KeyPoint& kp_left, const cv::KeyPoint& kp_right,
    float max_dist_2, float* distance) const
{
  // same octave
  if (kp_left.octave != kp_right.octave)
    return false;

  // epipolar constraint
  float y_diff = fabs(kp_left.pt.y - kp_right.pt.y);
  if (y_diff > params_.max_y_diff)
    return false;

  // disparity constraint
  float disparity = kp_left.pt.x - kp_right.pt.x;
  if (disparity < params_.min_disparity || disparity > params_.max_disparity)
    return false;

  // key point angle
  if (kp_left.angle > 0 && kp_right.angle > 0)
  {
    float angle_diff = fabs(kp_left.angle - kp_right.angle);
    if (angle_diff > params_.max_angle_diff)
      return false;
  }

  // key point size
  float size_diff = fabs(kp_left.size - kp_right.size);
  if (size_diff > params_.max_size_diff)
    return false;

  // compute matching using a light descriptor for the remaining
  using stereo_slam::image_proc::lightMatch;
  return lightMatch(image_left, image_right, 
      kp_left.pt, kp_right.pt, max_dist_2, distance);
}

void stereo_slam::StereoMatcher::match(
  const cv::Mat& image_left, const cv::Mat& image_right,
  const std::vector<cv::KeyPoint>& key_points_left,
  const std::vector<cv::KeyPoint>& key_points_right,
  float squared_max_dist,
  std::vector<cv::DMatch>& matches) const
{
  matches.clear();
  // brute force
  for (size_t i = 0; i < key_points_left.size(); ++i)
  {
    for (size_t j = 0; j < key_points_right.size(); ++j)
    {
      float distance;
      if (isMatch(image_left, image_right, key_points_left[i],
            key_points_right[j], squared_max_dist, &distance))
      {
        cv::DMatch match(j, i, -1, distance);
        matches.push_back(match);
      }
    }
  }
}

