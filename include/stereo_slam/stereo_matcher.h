#ifndef _STEREO_MATCHER_H_
#define _STEREO_MATCHER_H_

#include <vector>

namespace cv
{
  class Mat;
  class KeyPoint;
  class DMatch;
}

namespace stereo_slam
{

class StereoMatcher
{

public:

  struct Params
  {
    /**
     * Sets default params (see cpp for values)
     */
    Params();

    int max_octave_diff;
    float max_y_diff;
    float min_disparity;
    float max_disparity;
    float max_angle_diff;
    float max_size_diff;
  };

  /**
   * Creates a matcher with default params.
   */
  StereoMatcher();

  /**
   * Create matcher with given params.
   */
  StereoMatcher(const Params& params);

  void match(const cv::Mat& image_left, const cv::Mat& image_right,
    const std::vector<cv::KeyPoint>& key_points_left,
    const std::vector<cv::KeyPoint>& key_points_right,
    float squared_max_dist,
    std::vector<cv::DMatch>& matches) const;

  bool isMatch(
      const cv::Mat& image_left, const cv::Mat& image_right,
      const cv::KeyPoint& kp_left, const cv::KeyPoint& kp_right,
      float max_dist_2, float* distance) const;

private:

  Params params_;

};

}

#endif

