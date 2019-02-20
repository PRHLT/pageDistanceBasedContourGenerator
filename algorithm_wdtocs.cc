#include "algorithm_wdtocs.hpp"

namespace prhlt {
    using namespace log4cxx;using namespace log4cxx::helpers;
    using namespace boost;
    using namespace Eigen;
    
    Algorithm_WDTOCS::Algorithm_WDTOCS(cv::Mat &ex_image):Algorithm_Distance_Map(ex_image){
    
    }

    void Algorithm_WDTOCS::run(int ex_curvature_ratio, int ex_threshold,float ex_direct_constant, float ex_diagonal_constant){
        this->direct_distance_constant = ex_direct_constant;
        this->diagonal_distance_constant = ex_diagonal_constant;
        Algorithm_Distance_Map::run(ex_curvature_ratio,ex_threshold);
    }

    void Algorithm_WDTOCS::run(int ex_curvature_ratio, cv::Mat frontier_points_mat,float ex_direct_constant, float ex_diagonal_constant){
        this->direct_distance_constant = ex_direct_constant;
        this->diagonal_distance_constant = ex_diagonal_constant;
        Algorithm_Distance_Map::run(ex_curvature_ratio,frontier_points_mat);
    }

    float Algorithm_WDTOCS::calculate_neighbour_value(int r1,int c1, int r2, int c2){
        LOG4CXX_DEBUG(this->logger,"<<WDTOCS: calculate grey gradient>>");
        float gradient = 0.0;
        if(r1 != r2 and c1 != c2)
            gradient = calculate_diagonal_path_grey_gradient(r1,c1,r2,c2);
        else
            gradient = calculate_straight_path_grey_gradient(r1,c1,r2,c2);
    
        return gradient+this->distance_matrix(r2,c2);
    }

    float Algorithm_WDTOCS::calculate_straight_path_grey_gradient(int r1,int c1, int r2, int c2){
        return ((float)this->curvature_ratio)*sqrt( pow( (float)((int)this->image.at<uchar>(r1,c1) - (int)this->image.at<uchar>(r2,c2)),2)+this->direct_distance_constant);
    }

    float Algorithm_WDTOCS::calculate_diagonal_path_grey_gradient(int r1,int c1, int r2, int c2){
        return ((float)this->curvature_ratio)*sqrt( pow((float)((int)this->image.at<uchar>(r1,c1) - (int)this->image.at<uchar>(r2,c2)),2)+this->diagonal_distance_constant);
    }

}
