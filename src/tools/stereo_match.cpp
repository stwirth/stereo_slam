#include <iomanip>
#include <boost/program_options.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "stereo_slam/stereo_matcher.h"
#include "stereo_slam/match_stats.h"

namespace po = boost::program_options;

int main(int argc, char** argv)
{
  // declare the supported options
  std::string left_image_file;
  std::string right_image_file;
  std::string detector_name;
  double squared_max_dist;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("image_left", 
     po::value<std::string>(&left_image_file)->required(), 
     "left rectified image")
    ("image_right", 
     po::value<std::string>(&right_image_file)->required(), 
     "right rectified image")
    ("feature_detector,F", 
     po::value<std::string>(&detector_name)->default_value("ORB"), 
     "feature detector to use")
    ("max_dist,D", 
     po::value<double>(&squared_max_dist)->default_value(100.0), 
     "maximum distance for light descriptors")
    ("display", "display matching output (blocks while window is open)")
  ;
  po::positional_options_description positional_options;
  positional_options.add("image_left", 1);
  positional_options.add("image_right", 1);

  // parse command line
  po::variables_map vm;
  try
  {
    po::store(po::command_line_parser(argc, argv).options(desc)
        .positional(positional_options).run(), vm);
    po::notify(vm);
  } catch (const po::error& error)
  {
    std::cerr << "Error parsing program options: " << std::endl;
    std::cerr << "  " << error.what() << std::endl;
    std::cerr << desc << std::endl;
    return EXIT_FAILURE;
  }

  squared_max_dist *= squared_max_dist;

  // load images
  cv::Mat image_left = cv::imread(left_image_file, 0);
  cv::Mat image_right = cv::imread(right_image_file, 0);
  if (image_left.empty() || image_right.empty())
  {
    std::cerr << "Error loading images." << std::endl;
    return EXIT_FAILURE;
  }

  // extract features
  cv::initModule_nonfree();
  cv::Ptr<cv::FeatureDetector> detector = 
    cv::FeatureDetector::create(detector_name);
  std::vector<cv::KeyPoint> key_points_left, key_points_right;
  detector->detect(image_left, key_points_left);
  detector->detect(image_right, key_points_right);

  std::cout << key_points_left.size() 
    << " features in left image" << std::endl;
  std::cout << key_points_right.size() 
    << " features in right image" << std::endl;

  // perform matching
  std::vector<cv::DMatch> matches;
  stereo_slam::StereoMatcher stereo_matcher;
  stereo_matcher.match(image_left, image_right,
      key_points_left, key_points_right, 
      squared_max_dist, matches);

  std::cout << matches.size() << " matches." << std::endl;
  stereo_slam::match_stats::printMatchStats(
      key_points_left, key_points_right, matches);

  if (vm.count("display"))
  {
    cv::Mat canvas;
    cv::drawMatches(image_left, key_points_left, 
                    image_right, key_points_right, 
                    matches, canvas);
    cv::namedWindow("Matches", 0);
    cv::imshow("Matches", canvas);
    cv::waitKey(0);
  }

  return EXIT_SUCCESS;
}

