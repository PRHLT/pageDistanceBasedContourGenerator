#ifndef ALGORITHM_RLSA_HPP_9F3UIS3QOP
#define ALGORITHM_RLSA_HPP_9F3UIS3QOP

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
    class Algorithm_RLSA{
        public:
            Algorithm_RLSA(cv::Mat &ex_image);
            ~Algorithm_RLSA();
            cv::Mat run(const int vertical_mean_letter_length, const int horizontal_mean_letter_length, const int threshold);
        private:
            cv::Mat image;
            log4cxx::LoggerPtr logger;
            int threshold;
            void perform_vertical_rlsa(const int meanLetterLength);
            void perform_horizontal_rlsa(const int meanLetterLength);
    };
}

#endif /* end of include guard: ALGORITHM_RLSA_HPP_9F3UIS3QOP*/
