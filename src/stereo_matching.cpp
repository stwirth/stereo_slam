#include "stereo_slam/image_proc.h"
#include "stereo_slam/stereo_matching.h"

static const float MAX_Y_DIFF = 2.0;
static const float MIN_DISPARITY = 1.0;
static const float MAX_DISPARITY = 10000.0;
static const float MAX_ANGLE_DIFF = 5.0;
static const float MAX_SIZE_DIFF = 10.0;

bool _isMatch(const cv::Mat& image_left, const cv::Mat& image_right,
    const cv::KeyPoint& kp_left, const cv::KeyPoint& kp_right,
    float max_dist_2, float* distance)
{
  // same octave
  if (kp_left.octave != kp_right.octave)
    return false;

  // epipolar constraint
  float y_diff = fabs(kp_left.pt.y - kp_right.pt.y);
  if (y_diff > MAX_Y_DIFF)
    return false;

  // disparity constraint
  float disparity = kp_left.pt.x - kp_right.pt.x;
  if (disparity < MIN_DISPARITY || disparity > MAX_DISPARITY)
    return false;

  // key point angle
  if (kp_left.angle > 0 && kp_right.angle > 0)
  {
    float angle_diff = fabs(kp_left.angle - kp_right.angle);
    if (angle_diff > MAX_ANGLE_DIFF)
      return false;
  }

  // key point size
  float size_diff = fabs(kp_left.size - kp_right.size);
  if (size_diff > MAX_SIZE_DIFF)
    return false;

  // compute matching using a light descriptor for the remaining
  using stereo_slam::image_proc::lightMatch;
  return lightMatch(image_left, image_right, 
      kp_left.pt, kp_right.pt, max_dist_2, distance);
}

void stereo_slam::stereo_matching::match(
  const cv::Mat& image_left, const cv::Mat& image_right,
  const std::vector<cv::KeyPoint>& key_points_left,
  const std::vector<cv::KeyPoint>& key_points_right,
  float squared_max_dist,
  std::vector<cv::DMatch>& matches)
{
  matches.clear();
  // brute force
  for (size_t i = 0; i < key_points_left.size(); ++i)
  {
    for (size_t j = 0; j < key_points_right.size(); ++j)
    {
      float distance;
      if (_isMatch(image_left, image_right, key_points_left[i],
            key_points_right[j], squared_max_dist, &distance))
      {
        cv::DMatch match(j, i, -1, distance);
        matches.push_back(match);
      }
    }
  }
}

