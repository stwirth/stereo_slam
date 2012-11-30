#ifndef _POSE_NODE_H_
#define _POSE_NODE_H_

#include "stereo_slam/features3d.h"

namespace stereo_slam
{

class PoseNode : public g2o::VertexSE3
{

public:

  /**
   * Creates an empty pose node without any information.
   * Use setEstimate and setFeatures to store data.
   */
  PoseNode();

  inline setFeatures(const Features3D& features)
  {
    features_ = features;
  }

  inline const Features3D& features() const
  {
    return features_;
  }


private:
  Features3D features_;
};

}

#endif
