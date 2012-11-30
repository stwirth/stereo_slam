#ifndef _FEATURES3D_EXTRACTOR_
#define _FEATURES3D_EXTRACTOR_

namespace stereo_slam
{

class Features3DExtractor
{
public:

  Features3D extract(
      const cv::Mat& image_left,
      const cv::Mat& image_right,
      const image_geometry::StereoCameraModel& camera_model);
};

}

#endif

