#ifndef ALGORITHM_EXTRACT_POLYGONS_HPP_45SDFXCB23
#define ALGORITHM_EXTRACT_POLYGONS_HPP_45SDFXCB23

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


//namespace std {using namespace __gnu_cxx;}
using namespace std;

namespace prhlt {

    typedef vector< vector<cv::Point2f> > point_vector_list;
    class Algorithm_Extract_Polygons{
        public:
            Algorithm_Extract_Polygons(cv::Mat &ex_image);
            ~Algorithm_Extract_Polygons();
            cv::Mat run();
            void add_polygon( vector< cv::Point2f> ex_polygon);
            void add_polygons( vector< vector< cv::Point2f> > ex_polygon);
            void add_polygons( vector <vector< vector <cv::Point2f> > > ex_polygons);
        private:
            log4cxx::LoggerPtr logger;
            cv::Mat input_image;
            cv::Mat output_image;
            point_vector_list polygon_list;
            void calculate_output_image();
            bool is_point_in_polygon(cv::Point2f ex_point);
            void paint_output_pixel_white(int row , int col);
	};
}

#endif /* end of include guard*/
