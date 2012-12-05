#include <gtest/gtest.h>
#include <boost/progress.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "stereo_slam/stereo_matching.h"

TEST(StereoMatchingTests, matchTests)
{
  cv::initModule_nonfree();

  cv::Ptr<cv::Feature2D> sift = Algorithm::create<Feature2D>(
      "Feature2D.SIFT");

  cv::Mat descriptors;
  std::vector<cv::KeyPoint> key_points;
  (*sift)(image, cv::noArray(), key_points, descriptors);

  using stereo_slam::stereo_matching::match;

  cv::KeyPointDetector::Ptr
  match(image_left, image_right, key_points_left, key_points_right,
        squared_max_dist, matches);
  {
    boost::progress_timer t;
    std::cout << "Time: ";
  }

}

