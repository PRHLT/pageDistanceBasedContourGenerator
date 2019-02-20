#ifndef GREY_LEVEL_HISTOGRAM_HPP_45FTSEBNG7
#define GREY_LEVEL_HISTOGRAM_HPP_45FTSEBNG7

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
    typedef boost::unordered_map<int,float> sparse_histogram;
    
    class Grey_Level_Histogram{
        public:
            Grey_Level_Histogram();
            Grey_Level_Histogram(cv::Mat &ex_image);
            ~Grey_Level_Histogram();
            void set_image(cv::Mat &ex_image);
            float run();
            sparse_histogram histogram;
        private:
            cv::Mat image;
            log4cxx::LoggerPtr logger;
            float histogram_sum;
            float calculate_grey_level_histogram();
    };
}

#endif /* end of include guard: GREY_LEVEL_HISTOGRAM_HPP_45FTSEBNG7*/
