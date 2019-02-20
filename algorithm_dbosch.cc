#include "algorithm_dbosch.hpp"

namespace prhlt {
    using namespace log4cxx;using namespace log4cxx::helpers;
    using namespace boost;
    using namespace Eigen;
    
    Algorithm_DBOSCH::Algorithm_DBOSCH(cv::Mat &ex_image):Algorithm_Distance_Map(ex_image){
    
    }

    void Algorithm_DBOSCH::run(int ex_curvature_ratio, int ex_threshold,float ex_direct_constant, float ex_diagonal_constant){
        this->direct_distance_constant = ex_direct_constant;
        this->diagonal_distance_constant = ex_diagonal_constant;
        Algorithm_Distance_Map::run(ex_curvature_ratio,ex_threshold);
    }

    void Algorithm_DBOSCH::run(int ex_curvature_ratio, cv::Mat frontier_points_mat,float ex_direct_constant, float ex_diagonal_constant){
        this->direct_distance_constant = ex_direct_constant;
        this->diagonal_distance_constant = ex_diagonal_constant;
        Algorithm_Distance_Map::run(ex_curvature_ratio,frontier_points_mat);
    }

    float Algorithm_DBOSCH::calculate_neighbour_value(int r1,int c1, int r2, int c2){
        LOG4CXX_DEBUG(this->logger,"<<WDTOCS: calculate grey gradient>>");
        float gradient = 0.0;
        if(r1 != r2 and c1 != c2)
            gradient = 1.414213; 
        else
            gradient = 1.0; 

        return gradient+distance_matrix(r2,c2)+(float)((int)this->image.at<uchar>(r1,c1)/255.0);
        //return gradient+distance_matrix(r2,c2);
    }

}
