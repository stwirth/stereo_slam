#ifndef _FEATURES_3D_
#define _FEATURES_3D_

#include <opencv2/features2d/features2d.hpp>

namespace stereo_slam
{

struct Features3D
{
  std::vector<cv::KeyPoint> key_points;
  cv::Mat descriptors;
  std::vector<cv::Point3f> points;
};

}

#endif
