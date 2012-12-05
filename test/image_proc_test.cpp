#include <gtest/gtest.h>
#include <boost/progress.hpp>

#include "stereo_slam/image_proc.h"

TEST(ImageProcTests, getValueTests)
{
  using stereo_slam::image_proc::getValue;

  cv::Mat image(2, 2, CV_8UC1);
  image.at<unsigned char>(0, 0) = 0;
  image.at<unsigned char>(1, 0) = 0;
  image.at<unsigned char>(0, 1) = 0;
  image.at<unsigned char>(1, 1) = 255;

  EXPECT_EQ(getValue(image, 0.0f, 0.0f), 0);
  EXPECT_EQ(getValue(image, 1.0f, 0.0f), 0);
  EXPECT_EQ(getValue(image, 0.0f, 1.0f), 0);
  EXPECT_EQ(getValue(image, 1.0f, 1.0f), 255);

  EXPECT_EQ(getValue(image, 0.5f, 1.0f), 255/2);
  EXPECT_EQ(getValue(image, 1.0f, 0.5f), 255/2);
  EXPECT_EQ(getValue(image, 0.5f, 0.5f), 255/4);

  {
    boost::progress_timer t;
    unsigned char val;
    for (int i = 0; i < 1e6; ++i)
    {
      float x = 1.0 * rand() / RAND_MAX;
      float y = 1.0 * rand() / RAND_MAX;
      val = getValue(image, x, y);
    }
    std::cout << "Time for 1e6 calls: ";
  }

}

