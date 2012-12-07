#include <iomanip>

#include <opencv2/features2d/features2d.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include "stereo_slam/match_stats.h"

template<typename T>
void printStats(const std::string& title, const std::vector<T>& values)
{
  using namespace boost::accumulators;
  accumulator_set<double, stats<tag::min, tag::max, tag::mean, tag::variance> > acc;
  for (size_t i = 0; i < values.size(); ++i) acc(values[i]);

  using namespace std;
  cout << title << endl;
  cout << " MIN     MAX     MEAN    STDDEV" << endl;

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  cout.setf(ios::right);
  cout << setw(8) << setprecision(3) << boost::accumulators::min(acc)
       << setw(8) << setprecision(3) << boost::accumulators::max(acc)
       << setw(8) << setprecision(3) << mean(acc)
       << setw(8) << setprecision(3) << sqrt(variance(acc)) << endl;
}

float angleDiff(float a1, float a2)
{
  float diff = fabs(a1 - a2);
  return std::min(360 - diff, diff);
}

void stereo_slam::match_stats::printMatchStats(
    const std::vector<cv::KeyPoint>& query_key_points,
    const std::vector<cv::KeyPoint>& train_key_points,
    const std::vector<cv::DMatch> matches)
{
  std::vector<int> octave_diffs;
  std::vector<float> y_diffs;
  std::vector<float> disparities;
  std::vector<float> angle_diffs;
  std::vector<float> size_diffs;

  for (size_t i = 0; i < matches.size(); ++i)
  {
    const cv::KeyPoint& kp_query = 
      query_key_points[matches[i].queryIdx];
    const cv::KeyPoint& kp_train = 
      train_key_points[matches[i].trainIdx];
    octave_diffs.push_back(abs(kp_query.octave - kp_train.octave));
    y_diffs.push_back(fabs(kp_query.pt.y - kp_train.pt.y));
    disparities.push_back(kp_query.pt.x - kp_train.pt.x);
    angle_diffs.push_back(angleDiff(kp_query.angle, kp_train.angle));
    size_diffs.push_back(fabs(kp_query.size - kp_train.size));
  }

  std::cout << std::endl;
  printStats("Octaves", octave_diffs);
  std::cout << std::endl;
  printStats("Y-Diffs", y_diffs);
  std::cout << std::endl;
  printStats("Disparities", disparities);
  std::cout << std::endl;
  printStats("Angle Diffs", angle_diffs);
  std::cout << std::endl;
  printStats("Size Diffs", size_diffs);
}


