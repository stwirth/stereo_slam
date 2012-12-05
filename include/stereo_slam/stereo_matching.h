#ifndef _STEREO_MATCHING_H_
#define _STEREO_MATCHING_H_

#include <vector>

namespace cv
{
  class Mat;
  class KeyPoint;
  class DMatch;
}

namespace stereo_slam
{

namespace stereo_matching
{

void match(const cv::Mat& image_left, const cv::Mat& image_right,
    const std::vector<cv::KeyPoint>& key_points_left,
    const std::vector<cv::KeyPoint>& key_points_right,
    float squared_max_dist,
    std::vector<cv::DMatch>& matches);

}

}


#endif
