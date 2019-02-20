#ifndef ALGORITHM_DBOSCH_HPP_XB3U83S3CR1
#define ALGORITHM_DBOSCH_HPP_XB3U83S3CR1
#include <stdlib.h>
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
#include <Eigen/Core>
#include <Eigen/Eigen>
#include "algorithm_distance_map.hpp"

//namespace std {using namespace __gnu_cxx;}
using namespace std;

namespace prhlt {
    class Algorithm_DBOSCH: public Algorithm_Distance_Map{
        public:
            Algorithm_DBOSCH(cv::Mat &ex_image);
            void run(int ex_curvature_ratio, int ex_threshold,float ex_direct_constant, float ex_diagonal_constant);
            void run(int ex_curvature_ratio, cv::Mat frontier_points_mat,float ex_direct_constant, float ex_diagonal_constant);
        protected:
            float direct_distance_constant;
            float diagonal_distance_constant;
            virtual float calculate_neighbour_value(int r1,int c1, int r2, int c2);
	};
}

#endif /* end of include guard*/
