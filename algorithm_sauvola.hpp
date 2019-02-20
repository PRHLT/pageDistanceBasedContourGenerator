#ifndef ALGORITHM_SAUVOLA_HPP_41GTBVKV4
#define ALGORITHM_SAUVOLA_HPP_41GTBVKV4

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
//#include "grey_level_histogram.hpp"


//namespace std {using namespace __gnu_cxx;}
using namespace std;

namespace prhlt {
    class Algorithm_SAUVOLA{
        public:
            Algorithm_SAUVOLA(cv::Mat &ex_image);
            ~Algorithm_SAUVOLA();
            cv::Mat run(float ex_standard_deviation_constant, float ex_dynamic_range_constant, int ex_window_height, int ex_window_width);
        private:
            float standard_deviation_constant;
            float dynamic_range_constant;
            int window_height;
            int window_width;
            log4cxx::LoggerPtr logger;
            cv::Mat image;
            cv::Mat binarized_image;
            cv::Mat integral_image;
            cv::Mat integral_image_squared;
            void update_pixel(int x, int y);
            void initialize_integral_images();
            double average_grey_of_region(int from_x, int from_y);
            double standard_deviation_grey_of_region(double mean, int from_x, int from_y); 
	};
}

#endif /* end of include guard*/
