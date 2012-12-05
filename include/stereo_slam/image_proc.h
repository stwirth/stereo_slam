#ifndef _IMAGE_PROC_H_
#define _IMAGE_PROC_H_

#include <opencv2/features2d/features2d.hpp>

namespace stereo_slam
{
namespace image_proc
{

/**
 * @brief get pixel value using bilinear interpolation
 * @return interpolated pixel value
 */
inline unsigned char getValue(const cv::Mat& image, float x, float y)
{
  int px = static_cast<int>(x);
  int py = static_cast<int>(y);

  unsigned char v1 = image.at<unsigned char>(py, px);
  unsigned char v2 = image.at<unsigned char>(py, px + 1);
  unsigned char v3 = image.at<unsigned char>(py + 1, px);
  unsigned char v4 = image.at<unsigned char>(py + 1, px + 1);

  float fx = x - px;
  float fx1 = 1.0f - fx;
  float fy = y - py;
  float fy1 = 1.0f - fy;

  int w1 = fx1 * fy1 * 256.0f;
  int w2 = fx  * fy1 * 256.0f;
  int w3 = fx1 * fy  * 256.0f;
  int w4 = fx  * fy  * 256.0f;

  return (v1 * w1 + v2 * w2 + v3 * w3 + v4 * w4) >> 8;
}

/**
 * @brief very lightweight descriptor matching, returning as fast as possible
 */
bool lightMatch(const cv::Mat& image_left, const cv::Mat& image_right,
    const cv::Point2f& pt_left, const cv::Point2f& pt_right,
    float max_dist_2, float* distance)
{
  // sample points for the descriptor
  static const int DESCRIPTOR_SIZE = 7;
  static const float DESCRIPTOR_X[] = {0, -2,  2, -4,  4,  8, -8};
  static const float DESCRIPTOR_Y[] = {0,  0,  0,  0,  0,  0,  0};

  *distance = 0.0;
  for (int i = 0; i < DESCRIPTOR_SIZE; ++i)
  {
    unsigned char left_pixel_value = 
      getValue(image_left, pt_left.x + DESCRIPTOR_X[i],
                pt_left.y + DESCRIPTOR_Y[i]);
    unsigned char right_pixel_value = 
      getValue(image_right, pt_right.x + DESCRIPTOR_X[i],
                pt_right.y + DESCRIPTOR_Y[i]);
    float dist = left_pixel_value - right_pixel_value;
    *distance += dist * dist;
    if (*distance > max_dist_2)
      return false;
  }
  // if we did not return yet, we have a match and
  // its distance is stored in 'distance'.
  return true;
}

}
}

#endif

