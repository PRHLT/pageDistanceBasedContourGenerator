#ifndef ALGORITHM_CALCULATE_SEARCH_AREA_HPP_78GJFFW2930W
#define ALGORITHM_CALCULATE_SEARCH_AREA_HPP_78GJFFW2930W

#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/helpers/exception.h>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include "image.hpp"
#include <math.h>
#include <opencv2/core/core.hpp>

//namespace std {using namespace __gnu_cxx;}
using namespace std;

namespace prhlt
{
class Algorithm_Calculate_Search_Area
{
public:
  Algorithm_Calculate_Search_Area(cv::Mat image_mat, vector<vector<cv::Point> > ex_regions, vector<vector<vector<cv::Point> > > ex_baselines);
  vector<vector<cv::Rect> > run();

private:
  cv::Mat input_mat; 
  Image global_input_image;
  vector<vector<cv::Rect > > search_areas;
  vector<vector<cv::Point > > regions;
  vector<vector<vector<cv::Point > > > baselines;
  log4cxx::LoggerPtr logger;
  int max_allowed_distance;
  int current_line_distance;
  void calculate_all_search_areas();
  void calculate_search_area(int region_id, int line_id);
  float distance_between_point_and_segment(cv::Point ex_point, cv::Point start_segment, cv::Point end_segment);
  cv::Point closest_point_on_segment_to_point(cv::Point ex_point, cv::Point start_segment, cv::Point end_segment);
  vector<vector<cv::Point > > calculate_lines_between_baselines(int ex_region_id, int ex_current_line, int ex_other_line);
  void show_rectangle(cv::Rect ex_rect);
  void show_lines(vector<vector<cv::Point> > ex_lines);
  void show_points(vector<cv::Point2d> ex_points);
};
} // namespace prhlt

#endif
