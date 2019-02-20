#ifndef ALGORITHM_DTOCS_HPP_GF3U3S3CO1
#define ALGORITHM_DTOCS_HPP_GF3U3S3CO1
#include "algorithm_distance_map.hpp"


namespace prhlt {
    class Algorithm_DTOCS: public Algorithm_Distance_Map{
        public:
            Algorithm_DTOCS(cv::Mat &ex_image);
        protected:
            virtual float calculate_neighbour_value(int r1,int c1, int r2, int c2);
            int calculate_grey_gradient(int r1,int c1, int r2, int c2);
    };
}

#endif /* end of include guard*/
