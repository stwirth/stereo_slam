#ifndef _FEATURES3D_EXTRACTOR_
#define _FEATURES3D_EXTRACTOR_

#include "stereo_slam/types.h"

namespace stereo_slam
{

/**
 * Interface for 3D feature extractors.
 */
class Features3DExtractor
{
public:

  virtual ~Features3DExtractor() {}

  /**
   * Extraction interface
   */
  virtual Features3D extract(
      const cv::Mat& image_left,
      const cv::Mat& image_right,
      const StereoCameraModel& camera_model) = 0;
};

}

#endif

