#ifndef POINTS_FILE_HPP_99GTZXXCS5
#define POINTS_FILE_HPP_99GTZXXCS5

#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/helpers/exception.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/unordered_map.hpp>
#include <boost/multi_array.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <stdlib.h>


//namespace std {using namespace __gnu_cxx;}
using namespace std;

namespace prhlt{
    class Points_File{
        public:
            Points_File(string ex_file_name);
            ~Points_File();
            vector < vector< cv::Point2f > > get_points();
            int get_num_points();
            vector< vector< cv::Point2f > > get_points_as_polygons(int points_per_polygon);
        private:
            string file_name;
            vector < vector<cv::Point2f> > points;
            log4cxx::LoggerPtr logger;
            
            void load_file();
            cv::Point2f string_2_point(vector<string> string_values);
            int extract_num_points(vector<string> string_values);
            bool is_comment_line(string line);
            bool is_num_figures_line(vector<string> string_values);
            bool is_num_points_in_figure_line(vector<string> string_values);
    		int  extract_num_figures_in_file(vector<string> string_values);
            int  extract_num_points_in_figure(vector<string> string_values);
            bool is_point_line(vector<string> string_values);
            
    };
}
#endif /* end of include guard*/
