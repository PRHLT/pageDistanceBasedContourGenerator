#ifndef ALGORITHM_Distance_Map_HPP_VG3U4S7CO1
#define ALGORITHM_Distance_Map_HPP_VG3U4S7CO1
#include <stdlib.h>
#include <fstream>
#include <iomanip>
#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/helpers/exception.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/unordered_map.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/multi_array.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

//namespace std {using namespace __gnu_cxx;}
using namespace std;

namespace prhlt {
    class Algorithm_Distance_Map{
        public:
            Algorithm_Distance_Map(cv::Mat &ex_image);
            ~Algorithm_Distance_Map();
            void run(int ex_curvature_ratio, int ex_threshold);
            void run(int ex_curvature_ratio, cv::Mat ex_frontier_mat);
            cv::Mat get_distance_matrix_as_greyscale_image();
            Eigen::MatrixXd get_distance_matrix();
            void save_distance_matrix_to_file(string ex_filename);
            void save_image_matrix_to_file(string ex_filename);
        protected:
          float scale; 
          cv::Mat image;
          log4cxx::LoggerPtr logger;
          int curvature_ratio;
          Eigen::MatrixXd distance_matrix;
          void initialize_border();
          bool distance_matrix_changed_flag;
          void activate_distance_matrix_changed_flag();
          void reset_distance_matrix_changed_flag();
          void calculate_distance_map();
          void initialize_distance_matrix(int threshold);
          void initialize_distance_matrix(cv::Mat ex_frontier_mat);
          void forward_raster_sequence();
          void update_value_forward(int r, int c);
          void backward_raster_sequence();
          void update_value_backward(int r, int c);
          float get_minimum_for_update(int r1, int c1, int r2, int c2);
          virtual float calculate_neighbour_value(int r1, int c1, int r2, int c2);
          bool solution_converged();
    };
}

#endif /* end of include guard*/
