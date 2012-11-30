#ifndef _FEATURES_3D_
#define _FEATURES_3D_

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
