#ifndef POLYLINE_EXTRACTOR_HPP_9F3UIS3QOP
#define POLYLINE_EXTRACTOR_HPP_9F3UIS3QOP

#include "algorithm_dbosch.hpp"
#include "algorithm_sauvola.hpp"
#include "algorithm_otsu.hpp"
#include "algorithm_rlsa.hpp"
#include "algorithm_dp_path_finder.hpp"
#include "algorithm_calculate_search_area.hpp"
#include "line_region_list.hpp"
#include "image.hpp"
#include <boost/thread.hpp>
#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/helpers/exception.h>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>

//namespace std {using namespace __gnu_cxx;}
using namespace std;

namespace prhlt
{
    class Polyline_Extractor
    {
        public:
            Polyline_Extractor(string input_image_file_name, string extract_image_file_name);
            Polyline_Extractor(cv::Mat input_mat, cv::Mat extract_mat);
            ~Polyline_Extractor();
            void run(string line_region_file_name);
            void run(vector<int> region_limits, vector<vector<vector<cv::Point> > > limits);
            void run(vector<vector<cv::Point> > ex_regions, vector<vector<vector<cv::Point> > > ex_baselines, int ex_num_workers = 1, float ex_approx_dist = -1, bool ex_enclosing_rect = false, int up_dist = 100, int low_dist = 50, int horizontal_padding=0);
            void run(vector<int> region_limits, cv::Mat ex_limits_mat);
            void set_frontier_calculation_parameters(float ex_standard_deviation_contant, float ex_dynamic_range_constant, int ex_window_size);
            void set_distance_map_parameters(int ex_curv_ratio, float ex_delta, float ex_beta);
            void save_line_images_to_file(string base_file_name);
            void save_line_images_with_alpha_to_file(string base_file_name);
            void save_labeled_image_to_file(string file_name);
            void save_global_polylines_image(string file_name);
            void save_global_polylines_image(cv::Mat tmp, string file_name);
            void save_polylines_tool_format(string file_name);
            void load_polylines(vector<vector<cv::Point2f> > ex_polylines);
            vector<vector<vector<cv::Point> > > get_line_contours();
            void show_polylines();

        private:
            int num_workers;
            vector<thread *> threads;
            bool enclosing_rect;
            int up_dist;
            int low_dist;
            int horizontal_padding; 
            float approx_dist_error;
            float standard_deviation_constant;
            float dynamic_range_constant;
            int window_size;
            int curvature_ratio;
            float delta;
            float beta;
            log4cxx::LoggerPtr logger;
            string input_image_file_name;
            string extract_image_file_name;
            string input_region_file_name;
            bool images_calculated;
            Image global_input_image;
            Image global_extract_image;
            Image global_output_image;
            unsigned int *label_image;
            cv::Mat input_mat;
            cv::Mat extract_mat;
            cv::Mat sauvola_mat;
            cv::Mat limits_mat;
            Eigen::MatrixXd distance_mat;
            vector<vector<vector<cv::Point > > > search_restrictions;
            vector<vector<cv::Point2d > > polylines;
            vector<vector<cv::Point2d > > collision_points;
            vector<vector<vector<cv::Point2d > > > area_polylines;
            vector<vector<vector<cv::Point2d > > > area_collision_points;
            vector<vector<vector<cv::Point> > > line_contours;

            vector<vector<cv::Rect> > search_areas;
            vector<vector<int> > search_regions;
            vector<vector<int> > line_limits;
            vector<cv::Mat> line_images;
            vector<cv::Mat> mask_images;
            vector<cv::Point> vector_2d_to_simp(const vector<cv::Point2d> &aux_vector);
            vector<vector<cv::Point2d> > review_loaded_polylines(vector<vector<cv::Point2f> > ex_polylines);
            vector<cv::Point2d> expand_points(cv::Point2f from, cv::Point2f to);
            void show_rectangle(cv::Rect ex_rect);
            void show_lines(vector<vector<cv::Point> > ex_lines);
            void load_input_image(string ex_input_image_file_name);
            void load_extract_image(string ex_extract_image_file_name);
            void set_default_constant_values();
            void calculate_frontiers();
            void calculate_search_areas(vector<vector<cv::Point> > ex_regions, vector<vector<vector<cv::Point> > > ex_baselines);
            void calculate_search_areas2(vector<vector<cv::Point> > ex_regions, vector<vector<vector<cv::Point> > > ex_baselines);
            vector<cv::Point> calculate_contour_from_baseline(const vector<cv::Point > &baseline);
            void calculate_line_contours();
            vector<cv::Point2d> get_clipped_contour(int reg_id, int line_id, string direction);
            void calculate_range_line_frontier_with_search_area(int i, int from, int to);
            void calculate_line_contour(int i, int j);
            void calculate_distance_matrix();
            void calculate_line_frontiers();
            vector<vector<cv::Point> > expand_search_restriction(int reg_index, int res_index, int x_offset = 0, int y_offset = 0);
            void calculate_line_frontiers_with_search_areas();
            void parallel_calculate_line_frontiers_with_search_areas();
            void calculate_single_line_frontier_with_search_area(int i, int j);
            void run();
            void generate_line_images();
            void extract_line(int start_frontier_index, int end_frontier_index);
            void extract_line_full_page(int start_frontier_index, int end_frontier_index);
            void copy_region_to_last_image(int column, int start_row, int end_row, int high_point);
            void label_region(int column, int start_row, int end_row, int line_number);
            void mask_region(int column, int start_row, int end_row, int high_point, int mask_value);
            vector<vector<int> > sequence_frontier(const vector<cv::Point2d> &frontier, int frontier_index, string mode);
            void cicular_sequenced_frontier_correction(vector<vector<int> > &sequence, int frontier_index, string mode, int radius);
            int get_highest_point_in_path(const vector<vector<int> > &sequenced_path);
            int get_highest_point_in_path(const vector<cv::Point2d> &path);
            int get_lowest_point_in_path(const vector<vector<int> > &sequenced_path);
            int get_lowest_point_in_path(const vector<cv::Point2d> &path);
            bool is_too_far_above(cv::Rect current_rect, cv::Rect compared_to);
            bool is_too_far_below(cv::Rect current_rect, cv::Rect compared_to);
            bool it_overlaps(cv::Rect current_rect, cv::Rect compared_to);
            vector<vector<cv::Point> > expand_my_search_restriction(int reg_index, int res_index, int x_offset, int y_offset);
            vector<vector<cv::Point> > expand_other_search_restriction(int reg_index, int res_index, int x_offset, int y_offset);
            vector<vector<cv::Point> > expand_above_search_restrictions(int reg_index, int res_index, int x_offset, int y_offset); 
            vector<vector<cv::Point> > expand_below_search_restrictions(int reg_index, int res_index, int x_offset, int y_offset);   
            int relative_possition_of_point_to_baseline(cv::Point ex_point, const vector<cv::Point> & ex_baseline); 
    }; 
}; // namespace prhlt

#endif /* end of include guard: ALGORITHM_RLSA_HPP_9F3UIS3QOP*/
