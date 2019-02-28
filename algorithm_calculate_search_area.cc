#include "algorithm_calculate_search_area.hpp"

namespace prhlt
{
using namespace log4cxx;
using namespace log4cxx::helpers;
using namespace boost;

Algorithm_Calculate_Search_Area::Algorithm_Calculate_Search_Area(cv::Mat image_mat, vector<vector<cv::Point > > ex_regions, vector<vector<vector<cv::Point > > > ex_baselines)
{
    this->logger = Logger::getLogger("Calculate Search Area");
    LOG4CXX_INFO(logger, "<<Instantiating Calculate Search Area>>");
    this->baselines = ex_baselines;
    this->regions = ex_regions;
    this->input_mat = image_mat;
    this->global_input_image.load_from_matrix(image_mat);
    this->max_allowed_distance = 50;
    this->current_line_distance = this->max_allowed_distance;
    LOG4CXX_INFO(logger, "<<DONE Instantiation>>");
}

vector<vector<cv::Rect > > Algorithm_Calculate_Search_Area::run()
{
    calculate_all_search_areas();
    return this->search_areas;
}

void Algorithm_Calculate_Search_Area::calculate_all_search_areas()
{
    this->search_areas.resize(this->regions.size());
    for (int region_id = 0; region_id < this->baselines.size(); region_id++)
    {
        for (int line_id = 0; line_id < this->baselines[region_id].size(); line_id++){
            calculate_search_area(region_id, line_id);
        }
    }
}

void Algorithm_Calculate_Search_Area::calculate_search_area(int ex_region_id, int ex_line_id){
    vector<vector<cv::Point> > res_lines;
    this->current_line_distance = this->max_allowed_distance;

    for (int other_line_id = 0; other_line_id < this->baselines[ex_region_id].size(); other_line_id++)
    {
        if(other_line_id != ex_line_id){
            vector<vector<cv::Point> > tmp_lines = calculate_lines_between_baselines(ex_region_id,ex_line_id, other_line_id);
            LOG4CXX_INFO(logger, "<<temp lines>> " << tmp_lines.size() << " from " << other_line_id);
            res_lines.reserve(res_lines.size() + tmp_lines.size());
            res_lines.insert(res_lines.end(), tmp_lines.begin(), tmp_lines.end());
        }
    }
    //show_lines(res_lines);
    LOG4CXX_INFO(logger, "<<temp lines>> " << res_lines.size());

    cv::RotatedRect tmp_rotated_rect = cv::minAreaRect(this->baselines[ex_region_id][ex_line_id]);
    cv::Rect tmp_rect = cv::boundingRect(this->baselines[ex_region_id][ex_line_id]);
    //show_rectangle(tmp_rect);

    cv::Rect high_clip(tmp_rect.x, tmp_rect.y - this->max_allowed_distance, tmp_rect.width, this->max_allowed_distance);

    vector<cv::Point> final_up_points;
    vector<cv::Point2d> final_up2d_points;
    for (int i = 0; i < res_lines.size(); i++)
    {
        cv::Point p1(res_lines[i][0].x, res_lines[i][0].y);
        cv::Point p2(res_lines[i][1].x, res_lines[i][1].y);
        LOG4CXX_INFO(logger, "<<CLIP>> " << high_clip.x << " - " << high_clip.y << " : " << high_clip.width << " - " << high_clip.height);
        LOG4CXX_INFO(logger, "<<BEFORE>> " << p2.x << " - " << p2.y);
        bool clip_res = cv::clipLine(high_clip, p1, p2);
        LOG4CXX_INFO(logger, "<<AFTER>> " << p2.x << " - " << p2.y << " : " << clip_res);
        if (clip_res)
        {
            final_up_points.push_back(p1);
            final_up_points.push_back(p2);
            final_up2d_points.push_back(cv::Point2d(p2.y, p2.x));
        }
        }
       // show_points(final_up2d_points);
        cv::Rect final_up_rect = cv::boundingRect(final_up_points);
        //show_rectangle(final_up_rect);
        this->search_areas[ex_region_id].push_back(final_up_rect);

        cv::Rect low_clip(tmp_rect.x, tmp_rect.y, tmp_rect.width, this->max_allowed_distance);
        //show_rectangle(low_clip);
        vector<cv::Point> final_points;
        vector<cv::Point2d> final_down2d_points;
        for (int i = 0; i < res_lines.size(); i++)
        {
            vector<cv::Point> sorted_baseline = res_lines[i];
            bool clip_res = cv::clipLine(low_clip, sorted_baseline[0], sorted_baseline[1]);
            if (clip_res)
            {
                final_points.push_back(sorted_baseline[0]);
                final_points.push_back(sorted_baseline[1]);
                final_down2d_points.push_back(cv::Point2d(sorted_baseline[1].y, sorted_baseline[1].x));
                final_down2d_points.push_back(cv::Point2d(sorted_baseline[0].y, sorted_baseline[0].x));
            }
        }
       // show_points(final_down2d_points);
        cv::Rect final_rect = cv::boundingRect(final_points);
        
        this->search_areas[ex_region_id].push_back(final_rect);
}

vector<vector<cv::Point> > Algorithm_Calculate_Search_Area::calculate_lines_between_baselines(int ex_region_id, int ex_current_line, int ex_other_line)
{
    vector<vector<cv::Point> > res_lines;


    for (int current_line_point = 0; current_line_point < this->baselines[ex_region_id][ex_current_line].size(); current_line_point++){
        for (int region_point = 0; region_point < this->regions[ex_region_id].size(); region_point++){
            vector<cv::Point> tmp_vector;
            tmp_vector.push_back(this->baselines[ex_region_id][ex_current_line][current_line_point]);
            if (region_point < this->regions[ex_region_id].size()-1)
            {
                tmp_vector.push_back(closest_point_on_segment_to_point(this->baselines[ex_region_id][ex_current_line][current_line_point], this->regions[ex_region_id][region_point], this->regions[ex_region_id][region_point + 1]));
            }else{
                tmp_vector.push_back(closest_point_on_segment_to_point(this->baselines[ex_region_id][ex_current_line][current_line_point], this->regions[ex_region_id][region_point], this->regions[ex_region_id][0]));
            }
            res_lines.push_back(tmp_vector);
        }
        for (int other_line_point = 0; other_line_point < this->baselines[ex_region_id][ex_other_line].size() - 1; other_line_point++)
            {
                vector<cv::Point> tmp_vector;
                tmp_vector.push_back(this->baselines[ex_region_id][ex_current_line][current_line_point]);
                tmp_vector.push_back(closest_point_on_segment_to_point(this->baselines[ex_region_id][ex_current_line][current_line_point], this->baselines[ex_region_id][ex_other_line][other_line_point], this->baselines[ex_region_id][ex_other_line][other_line_point + 1]));
                res_lines.push_back(tmp_vector);
            }
    }
    return res_lines;
}

void Algorithm_Calculate_Search_Area::show_points(vector<cv::Point2d> ex_points)
{
    cv::Mat tmp_mat = this->input_mat.clone();
    this->global_input_image.load_from_matrix(tmp_mat);
    this->global_input_image.encircle_points(ex_points, 5, cv::Scalar(0, 0, 255));
    this->global_input_image.display_with_scale("Rectangle", 0.5, 500000, true);
}

void Algorithm_Calculate_Search_Area::show_lines(vector<vector<cv::Point> > ex_lines)
{
    cv::Mat tmp_mat = this->input_mat.clone();
    this->global_input_image.load_from_matrix(tmp_mat);

    for(int i = 0; i < ex_lines.size(); i++){
        this->global_input_image.draw_line(cv::Point2d(ex_lines[i][0].x, ex_lines[i][0].y), cv::Point2d(ex_lines[i][1].x, ex_lines[i][1].y), cv::Scalar(0,255,0));
    }
    this->global_input_image.display_with_scale("Rectangle", 0.5, 500000, true);
}

void Algorithm_Calculate_Search_Area::show_rectangle(cv::Rect ex_rect)
{
    cv::Mat tmp_mat = this->input_mat.clone();
    this->global_input_image.load_from_matrix(tmp_mat);
    this->global_input_image.draw_rectangle(ex_rect, cv::Scalar(0, 0, 255));
    this->global_input_image.display_with_scale("Rectangle", 0.5, 500000, true);
}

float Algorithm_Calculate_Search_Area::distance_between_point_and_segment(cv::Point ex_point, cv::Point start_segment, cv::Point end_segment)
{
    float res;
        float a = ex_point.x - start_segment.x;
        float b = ex_point.y - start_segment.y;
        float c = end_segment.x - start_segment.x;
        float d = end_segment.y - start_segment.y;

        float dot = a * c + b * d;
        float len_sq = c * c + d * d;
        float param = -1;
        if (len_sq != 0) //in case of 0 length line
            param = dot / len_sq;

        float xx, yy;

        if (param < 0){
            xx = start_segment.x;
            yy = start_segment.y;
        }
        else 
            if (param > 1){
                xx = end_segment.x;
                yy = end_segment.y;
            }
        else{
            xx = start_segment.x + param * c;
            yy = start_segment.y + param * d;
        }

        float dx = ex_point.x - xx;
        float dy = ex_point.y - yy;
        res= sqrt(dx * dx + dy * dy);
        return res; 
}

cv::Point Algorithm_Calculate_Search_Area::closest_point_on_segment_to_point(cv::Point ex_point, cv::Point start_segment, cv::Point end_segment)
{
    float a = ex_point.x - start_segment.x;
    float b = ex_point.y - start_segment.y;
    float c = end_segment.x - start_segment.x;
    float d = end_segment.y - start_segment.y;

    float dot = a * c + b * d;
    float len_sq = c * c + d * d;
    float param = -1;
    if (len_sq != 0) //in case of 0 length line
        param = dot / len_sq;

    float xx, yy;

    if (param < 0)
    {
        xx = start_segment.x;
        yy = start_segment.y;
    }
    else if (param > 1)
    {
        xx = end_segment.x;
        yy = end_segment.y;
    }
    else
    {
        xx = start_segment.x + param * c;
        yy = start_segment.y + param * d;
    }

    return cv::Point(xx,yy);
}

} // namespace prhlt
