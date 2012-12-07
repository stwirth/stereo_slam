#include <iomanip>
#include <boost/program_options.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "stereo_slam/match_stats.h"

namespace po = boost::program_options;

int main(int argc, char** argv)
{
  // declare the supported options
  std::string left_image_file;
  std::string right_image_file;
  std::string detector_name;
  std::string extractor_name;
  std::string matcher_name;
  double matching_threshold;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("image_left", 
     po::value<std::string>(&left_image_file)->required(), 
     "left rectified image")
    ("image_right", 
     po::value<std::string>(&right_image_file)->required(), 
     "right rectified image")
    ("matching_threshold,T", 
     po::value<double>(&matching_threshold)->default_value(0.8), 
     "matching threshold")
    ("feature_detector,F", 
     po::value<std::string>(&detector_name)->default_value("SIFT"), 
     "feature detector to use")
    ("descriptor_extractor,D", 
     po::value<std::string>(&extractor_name)->default_value("SIFT"), 
     "descriptor extractor to use")
    ("matcher,M", 
     po::value<std::string>(&matcher_name)->default_value("BruteForce"), 
     "descriptor matcher to use")
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
  cv::Ptr<cv::DescriptorExtractor> extractor = 
    cv::DescriptorExtractor::create(extractor_name);
  cv::Mat descriptors_left, descriptors_right;
  extractor->compute(image_left, key_points_left, descriptors_left);
  extractor->compute(image_right, key_points_right, descriptors_right);

  std::cout << key_points_left.size() 
    << " features in left image" << std::endl;
  std::cout << key_points_right.size() 
    << " features in right image" << std::endl;

  // perform matching
  cv::Ptr<cv::DescriptorMatcher> matcher = 
    cv::DescriptorMatcher::create(matcher_name);
  std::vector<std::vector<cv::DMatch> > knn_matches;
  matcher->knnMatch(descriptors_left, descriptors_right, 
      knn_matches, 2);

  // select good matches using threshold
  std::vector<cv::DMatch> good_matches;
  for (size_t i = 0; i < knn_matches.size(); ++i)
  {
    if (knn_matches[i].size() < 2) continue;
    float dist_ratio = 
      knn_matches[i][0].distance / knn_matches[i][1].distance;
    if (dist_ratio < matching_threshold)
    {
      good_matches.push_back(knn_matches[i][0]);
    }
  }
  std::cout << good_matches.size() << " matches." << std::endl;
  std::cout << "Statistics of good matches: " << std::endl;
  stereo_slam::match_stats::printMatchStats(
      key_points_left, key_points_right, good_matches);
  if (vm.count("display"))
  {
    cv::Mat canvas;
    cv::drawMatches(image_left, key_points_left, 
                    image_right, key_points_right, 
                    good_matches, canvas);
    cv::namedWindow("Matches", 0);
    cv::imshow("Matches", canvas);
    cv::waitKey(0);
  }

  return EXIT_SUCCESS;
}

