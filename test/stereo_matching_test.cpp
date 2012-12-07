#include <gtest/gtest.h>
#include <boost/progress.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "stereo_slam/stereo_matcher.h"

TEST(StereoMatchingTests, matchTests)
{
  stereo_slam::StereoMatcher matcher;

  cv::initModule_nonfree();
  cv::Ptr<cv::Feature2D> sift = Algorithm::create<Feature2D>(
      "Feature2D.SIFT");
  std::vector<cv::KeyPoint> key_points_left, key_points_right;
  cv::Mat descriptors_left, descriptors_right;
  (*sift)(image_left, cv::noArray(), key_points_left, descriptors_left);
  (*sift)(image_right, cv::noArray(), key_points_right, descriptors_right);

  // do heavy load matching as "ground truth"
  std::vector<cv::DMatch> good_matches;
  feature_matching::threshold_matching(image_left, image_right,
      key_points_left, key_points_right,
      descriptors_left, descriptors_right, 0.5, good_matches);

  std::cout << good_matches.size() << " good matches.";

  matcher.match(image_left, image_right, key_points_left, key_points_right,
        squared_max_dist, matches);
  {
    boost::progress_timer t;
    std::cout << "Time: ";
  }

}

