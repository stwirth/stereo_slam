#ifndef _ORB_EXTRACTOR_
#define _ORB_EXTRACTOR_

#include <opencv2/features2d/features2d.hpp>

#include "stereo_slam/types.h"

namespace stereo_slam
{

class Features3D;

/**
 * @brief Extractor for ORB features, uses OpenCV implementation.
 *
 * Detects key points in left and right images,
 * matches using epipolar constraints and a simple
 * descriptor, then adds descriptors for all matches
 * using the left image.
 */
class ORBExtractor
{
public:

  /**
   * Creates the implementation instance
   */
  ORBExtractor();

  virtual Features3D extract(
      const cv::Mat& image_left,
      const cv::Mat& image_right,
      const StereoCameraModel& camera_model);

private:

  cv::ORB orb_;
  
};

}

#endif

