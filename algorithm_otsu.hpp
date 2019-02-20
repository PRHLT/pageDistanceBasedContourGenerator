#ifndef ALGORITHM_OTSU_HPP_10GTZXCBS4
#define ALGORITHM_OTSU_HPP_10GTZXCBS4

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
#include "grey_level_histogram.hpp"


//namespace std {using namespace __gnu_cxx;}
using namespace std;

namespace prhlt {
    class Algorithm_OTSU{
        public:
            Algorithm_OTSU(cv::Mat &ex_image);
            int run();
        private:
            Grey_Level_Histogram grey_level_histogram;
            log4cxx::LoggerPtr logger;
            int threshold;
            int calculate_otsu_threshold();	
    };
}

#endif /* end of include guard: ALGORITHM_OTSU_HPP_10GTZXCBS4*/
