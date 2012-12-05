#ifndef _STEREO_SLAM_TYPES_H_
#define _STEREO_SLAM_TYPES_H_

#include <ros/time.h>
#include <image_geometry/stereo_camera_model.h>

namespace stereo_slam
{

typedef ros::Time TimeStamp;
typedef image_geometry::StereoCameraModel StereoCameraModel;

}

#endif
