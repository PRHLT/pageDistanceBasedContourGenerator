#ifndef ALGORITHM_DP_PATH_FINDER_HPP_2T9GR5EG
#define ALGORITHM_DP_PATH_FINDER_HPP_2T9GR5EG

#include <stdlib.h>
#include <fstream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/helpers/exception.h>
#include "image.hpp"
#include <opencv2/core/core.hpp>


//namespace std {using namespace __gnu_cxx;}
using namespace std;

namespace prhlt {

    class Algorithm_DP_Path_Finder
    {
        public:
            Algorithm_DP_Path_Finder(cv::Mat ex_image_mat, const Eigen::MatrixXd& ex_cost_matrix);
            Algorithm_DP_Path_Finder(cv::Mat ex_image_mat, const Eigen::MatrixXd& ex_cost_matrix, cv::Rect ex_search_area );
            Algorithm_DP_Path_Finder(cv::Mat ex_image_mat, const Eigen::MatrixXd& ex_cost_matrix,int orig_x, int orig_y , int size_x, int size_y);
            ~Algorithm_DP_Path_Finder();
            void run(double ex_alpha, double ex_beta);
            void set_search_limits(vector < vector <cv::Point> > ex_upper_limits, vector < vector <cv::Point> > ex_lower_limits);
            void set_lower_search_limits(vector < vector <cv::Point> > ex_lower_limits);
            void set_upper_search_limits(vector < vector <cv::Point> > ex_lower_limits);
            void set_search_limits(vector <cv::Point> ex_upper_limits, vector <cv::Point> ex_lower_limits);
            void set_lower_search_limits(vector <cv::Point> ex_lower_limits);
            void set_upper_search_limits(vector <cv::Point> ex_lower_limits);
            vector<cv::Point2d> recover_path(int x, int y);
            vector< vector<cv::Point2d> > recover_all_paths();
            vector<cv::Point2d> recover_best_path();
            vector<cv::Point2d> get_best_path_collision_points();
            void save_path_matrix_to_file(string file_name);
        private:
            //GENERAL
            void forward();
            void backward();
            void reset_change_counter();
            bool solution_not_converged();
            void show_search_area();

            //INITIALIZATION
            void initialize_bound_matrix();
            //PATH MANAGEMENT FUNCTIONS
            void update_column(int c);
            void update_cell(int x , int y);
            bool update_cell_from(int from_x, int from_y, int to_x , int to_y);
            void review_column(int c);
            bool review_cell(int x , int y);
            void backtrack_column(int c);
            bool backtrack_cell(int x , int y);
            bool update_bound_matrix(int x, int y, double cost);
            void update_path_matrix(int x, int y, int from_x, int from_y);
            void display_path(int x, int y);
            void display_best_path();
            void save_best_path();
            bool is_collision_point(vector<cv::Point2d> points, int index );
            cv::Point2d localize_point(cv::Point2d point);
            cv::Point2d localize_point(int x , int y);


            //vector<cv::Point2d> recover_all_paths();
            //COST FUNCTIONS
            double base_movement_cost(int from_x, int from_y, int to_x, int to_y);
            double contextual_average(int x, int y, int context_size);
            double future_contextual_average(int x, int y,const int x_context_size, const int y_context_size);
			void calculate_valid_search_area();
            bool precalc_is_valid_point_as_per_search_limits(int x, int y);
            bool is_valid_point_as_per_search_limits(int x, int y);
            int point_position_in_respect_to_line(vector<cv:: Point> ex_line, int x, int y);
            bool restriction_segment_applicable_to_point(vector<cv:: Point> ex_line, int x, int y);
            //DATA
		    int axis_x;
		    int axis_y;
    		int cells_changed;
		    double alpha;
		    double beta;
		    double average_euclidian_distance_cost;
		    double average_grey_distance_cost;
		    Image image;
		    double roof_distance;
        vector < vector <cv::Point> > upper_search_limits;
        vector < vector <cv::Point> > lower_search_limits;
            Eigen::MatrixXd cost_matrix;
            Eigen::MatrixXd bound_matrix;
            Eigen::MatrixXi limits_matrix;
            Eigen::MatrixXi path_matrix_x;
            Eigen::MatrixXi path_matrix_y;
            log4cxx::LoggerPtr logger;
    };

}


#endif /* end of include guard: ALGORITHM_PATH_FINDER_HPP_2T9GR5EG */
