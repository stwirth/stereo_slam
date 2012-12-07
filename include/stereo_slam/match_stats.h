#ifndef _MATCH_STATS_
#define _MATCH_STATS_

#include <vector>

namespace cv
{
  class KeyPoint;
  class DMatch;
}

namespace stereo_slam
{


namespace match_stats
{

void printMatchStats(
    const std::vector<cv::KeyPoint>& query_key_points,
    const std::vector<cv::KeyPoint>& train_key_points,
    const std::vector<cv::DMatch> matches);


}

}

#endif
