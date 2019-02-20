#include "algorithm_dtocs.hpp"

namespace prhlt {
    
    Algorithm_DTOCS::Algorithm_DTOCS(cv::Mat &ex_image):Algorithm_Distance_Map(ex_image){
    }
    
    float  Algorithm_DTOCS::calculate_neighbour_value(int r1,int c1, int r2, int c2){
        LOG4CXX_DEBUG(this->logger,"<<DTOCS: calculate neighbour value>>");
        return 1+(float)calculate_grey_gradient(r1,c1,r2,c2)+this->distance_matrix(r2,c2);
    }
    
    int Algorithm_DTOCS::calculate_grey_gradient(int r1,int c1, int r2, int c2){
        LOG4CXX_DEBUG(this->logger,"<<DTOCS: calculate grey gradient>>");
        return this->curvature_ratio*abs((int)this->image.at<uchar>(r1,c1) - (int)this->image.at<uchar>(r2,c2));
    }
}
