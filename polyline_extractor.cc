#include "polyline_extractor.hpp"

namespace prhlt
{
using namespace log4cxx;
using namespace log4cxx::helpers;
using namespace boost;
using namespace Eigen;

Polyline_Extractor::Polyline_Extractor(string ex_input_image_file_name, string ex_extract_image_file_name)
{
  this->logger = Logger::getLogger("Polyline_Extractor");
  LOG4CXX_INFO(logger, "<<Instantiating POLYLINE EXTRACTOR>>");
  load_input_image(ex_input_image_file_name);
  //LOG4CXX_INFO(logger, "<<Loaded Input Image>>");
  this->approx_dist_error = -1;
  this->up_dist = 100;
  this->low_dist = 25;
  this->enclosing_rect = false;
  this->horizontal_padding=0;
  if (ex_extract_image_file_name == "")
    ex_extract_image_file_name = ex_input_image_file_name;
  load_extract_image(ex_extract_image_file_name);
  //LOG4CXX_INFO(logger, "<<Loaded Extract Image>>");

  this->limits_mat = cv::Mat(this->input_mat.rows, this->input_mat.cols, CV_8U, cv::Scalar(0, 0, 0));
  this->images_calculated = false;
  set_default_constant_values();
  LOG4CXX_INFO(logger, "<<DONE>>");
}

Polyline_Extractor::Polyline_Extractor(cv::Mat ex_input_mat, cv::Mat ex_extract_mat)
{
  this->logger = Logger::getLogger("Polyline_Extractor");
  LOG4CXX_INFO(logger, "<<Instantiating POLYLINE EXTRACTOR>>");
  this->approx_dist_error = -1;
  this->up_dist = 100;
  this->low_dist = 25;
  this->enclosing_rect = false;
  this->horizontal_padding=0;
  if (ex_input_mat.channels() > 1)
  {
    this->input_mat = cv::Mat(ex_input_mat.rows, ex_input_mat.cols, CV_8U, cv::Scalar(0, 0, 0));
    cvtColor(ex_input_mat, this->input_mat, CV_RGB2GRAY);
  }
  else
    this->input_mat = ex_input_mat;
  this->global_input_image.load_from_matrix(ex_extract_mat);
  this->extract_mat = ex_extract_mat;
  this->limits_mat = cv::Mat(this->input_mat.rows, this->input_mat.cols, CV_8U, cv::Scalar(0, 0, 0));
  this->images_calculated = false;
  set_default_constant_values();
  LOG4CXX_INFO(logger, "<<DONE>>");
}
/*  cv::Mat mask = cv::Mat::zeros(this->image.rows,this->image.cols, CV_8UC1);
        drawContours(mask,contour_vector,j,cv::Scalar(255), CV_FILLED);
        cv::Mat transparent( this->image.rows,this->image.cols, CV_8UC4);
        cv::Mat srcImg[] = {this->image,mask};
        int from_to[] = { 0,0, 1,1, 2,2, 3,3 };
        cv::mixChannels( srcImg, 2, &transparent, 1, from_to, 4 );
        cv::Mat cropped_image;
        cv::Mat(transparent,this->bounding_rectangles[i][j]).copyTo(cropped_image);
        this->line_images[this->line_images.size()-1].push_back(cropped_image);
        cropped_image.release();
        mask.release();
        transparent.release();
            */
void Polyline_Extractor::load_input_image(string ex_input_image_file_name)
{
  this->input_image_file_name = ex_input_image_file_name;
  //LOG4CXX_INFO(logger, "<<CALLING IMAGE CLASS TO LOAD>>");
  this->global_input_image.load_from_file(this->input_image_file_name);
  //LOG4CXX_INFO(logger, "<<GETTING MATRIX>>");
  cv::Mat tmp = this->global_input_image.get_matrix();
  //    this->orig_mat = tmp;

  if (tmp.channels() > 1)
  {
    //LOG4CXX_INFO(logger, "<<Channels>>");
    this->input_mat = cv::Mat(tmp.rows, tmp.cols, CV_8U, cv::Scalar(0, 0, 0));
    cvtColor(tmp, this->input_mat, CV_RGB2GRAY);
  }
  else
    this->input_mat = tmp;
  this->label_image = (unsigned int *)calloc(this->input_mat.cols * this->input_mat.rows, sizeof(int));
}

/* void Polyline_Extractor::generate_alpha_extract_image(string ex_extract_image_file_name){
  }*/

void Polyline_Extractor::load_extract_image(string ex_extract_image_file_name)
{
  this->extract_image_file_name = ex_extract_image_file_name;
  this->global_extract_image.load_from_file(this->extract_image_file_name);
  cv::Mat tmp = this->global_extract_image.get_matrix();

  //  if(tmp.channels()>1){
  //      this->extract_mat = cv::Mat(tmp.rows,tmp.cols,CV_8U,0);
  //      cvtColor(tmp, this->extract_mat, CV_RGB2GRAY);
  //  }
  //  else
  this->extract_mat = tmp;
}

void Polyline_Extractor::set_default_constant_values()
{
  this->curvature_ratio = 1;
  this->delta = 0.95509;
  this->beta = 1.3693;
  this->standard_deviation_constant = 0.2;
  this->dynamic_range_constant = 128;
  this->window_size = 40;
}

Polyline_Extractor::~Polyline_Extractor()
{
  this->input_mat.release();
  this->extract_mat.release();
  this->sauvola_mat.release();
  if (this->images_calculated)
  {
    for (int i = 0; i < this->line_images.size(); i++)
      this->line_images[i].release();
    for (int i = 0; i < this->polylines.size(); i++)
      this->polylines[i].clear();
    for (int i = 0; i < this->collision_points.size(); i++)
      this->collision_points[i].clear();
  }
  this->polylines.clear();
  this->search_regions.clear();
  this->line_images.clear();
}
void Polyline_Extractor::set_frontier_calculation_parameters(float ex_standard_deviation_constant, float ex_dynamic_range_constant, int ex_window_size)
{
  this->standard_deviation_constant = ex_standard_deviation_constant;
  this->dynamic_range_constant = ex_dynamic_range_constant;
  this->window_size = ex_window_size;
}

void Polyline_Extractor::set_distance_map_parameters(int ex_curv_ratio, float ex_delta, float ex_beta)
{
  this->curvature_ratio = ex_curv_ratio;
  this->delta = ex_delta;
  this->beta = ex_beta;
}

void Polyline_Extractor::run(string ex_line_region_file_name)
{
  LOG4CXX_INFO(logger, "<<RUNNING WITH OUT LIMITS>>");
  this->input_region_file_name = ex_line_region_file_name;
  Line_Region_List line_list(this->input_region_file_name, 0, this->input_mat.rows);
  //LOG4CXX_INFO(logger, "<<Getting regions>>");
  this->search_regions = line_list.get_search_zones();
  //LOG4CXX_INFO(logger, "<<Getting line limits>>");
  this->line_limits = line_list.get_line_limits();
  run();
}
void Polyline_Extractor::run(vector<int> region_limits, vector<vector<vector<cv::Point > > > ex_restrictions)
{

  LOG4CXX_INFO(logger, "<<RUNNING WITH LIMITS>>");
  //LOG4CXX_INFO(logger, "<<Generating regions and limits>>");
  for (int i = 0; i < region_limits.size() - 1; i++)
  {
    vector<int> tmp_region;
    tmp_region.push_back(region_limits[i]);
    tmp_region.push_back(region_limits[i + 1]);
    this->search_regions.push_back(tmp_region);
  }
  vector<int> tmp_region;
  tmp_region.push_back(region_limits[region_limits.size() - 1]);
  tmp_region.push_back(this->input_mat.rows - 1);
  this->search_regions.push_back(tmp_region);

  for (int i = 1; i < region_limits.size() - 1; i++)
  {
    vector<int> tmp_line;
    tmp_line.push_back(region_limits[i] - 40);
    tmp_line.push_back(region_limits[i]);
    this->line_limits.push_back(tmp_line);
  }

  //LOG4CXX_INFO(logger, "<<Saving search restrictions>>");
  this->search_restrictions = ex_restrictions;

  run();
}

void Polyline_Extractor::run(vector<vector<cv::Point> > ex_regions, vector<vector<vector<cv::Point > > > ex_baselines, int ex_num_workers, float ex_dist_error, bool ex_enclosing_rect, int ex_up_dist, int ex_low_dist, int ex_horizontal_padding)
{
  this->num_workers = ex_num_workers;
  this->approx_dist_error = ex_dist_error;
  this->up_dist = ex_up_dist;
  this->low_dist = ex_low_dist;
  this->enclosing_rect = ex_enclosing_rect;
  this->horizontal_padding=ex_horizontal_padding;

  LOG4CXX_INFO(logger, "<<RUNNING WITH SEARCH AREAS>>");
  //cout << "Num Lines " << ex_baselines[0].size() << endl;
  calculate_search_areas(ex_regions, ex_baselines);
  //this->search_restrictions = ex_baselines;
  this->search_restrictions.resize(ex_baselines.size());
  for (int i = 0; i < ex_baselines.size(); i++)
  {
    for (int j = 0; j < ex_baselines[i].size(); j++){
      this->search_restrictions[i].push_back(ex_baselines[i][j]);
      this->search_restrictions[i].push_back(ex_baselines[i][j]);
    }
  }

    calculate_frontiers();
  calculate_distance_matrix();
  LOG4CXX_INFO(logger, "<<CALCULATING LINE FRONTIER>>");
  if (this->num_workers > 1)
  {
    parallel_calculate_line_frontiers_with_search_areas();
  }
  else
  {
    if (this->num_workers <= 0)
    {
      this->num_workers = thread::hardware_concurrency();
      parallel_calculate_line_frontiers_with_search_areas();
    }
    else
    {
      calculate_line_frontiers_with_search_areas();
    }
  }
  LOG4CXX_INFO(logger, "<<CALCULATING CONTOUR>>");
  calculate_line_contours();
}
vector<vector<vector<cv::Point> > > Polyline_Extractor::get_line_contours()
{
  return this->line_contours;
}

void Polyline_Extractor::calculate_search_areas(vector<vector<cv::Point> > ex_regions, vector<vector<vector<cv::Point> > > ex_baselines)
{
  this->search_areas.resize(ex_regions.size());
  int horizontal_padding = this->horizontal_padding; 
  for (int i = 0; i < ex_regions.size(); i++)
  {
    cv::Rect region_rect = cv::boundingRect(ex_regions[i]);
    cv::Rect top_rect_frontier(region_rect.x, region_rect.y, region_rect.width, 0);
    cv::Rect bottom_rect_frontier(region_rect.x, region_rect.y + region_rect.height, region_rect.width, 0);
    cv::Rect last_rect(top_rect_frontier);
    cv::Rect last_calc_rect; 
    if (ex_baselines[i].size() > 0)
    {
      cv::Rect current_rect = cv::boundingRect(ex_baselines[i][0]);
      
      //int first_dist = abs(current_rect.y - last_rect.y) - 1;
      //if(is_too_far_above(current_rect,top_rect_frontier))
     // {
       int  first_dist = this->up_dist; 
      //}

      this->search_areas[i].push_back(cv::Rect(current_rect.x-horizontal_padding, current_rect.y - first_dist, current_rect.width+horizontal_padding, first_dist));
      last_calc_rect = this->search_areas[i][this->search_areas[i].size() - 1];
      last_rect = current_rect;
      //LOG4CXX_INFO(this->logger, "<<FIRST UP >> ");
      //show_rectangle(this->search_areas[i][this->search_areas[i].size() - 1]);
      //last_rect = this->search_areas[i][0];

      for (int j = 1; j < ex_baselines[i].size(); j++)
      {
        current_rect = cv::boundingRect(ex_baselines[i][j]);
        /*
        if(it_overlaps(current_rect,last_rect))
        {
          //LOG4CXX_INFO(this->logger, "<<Last RECT          : " << region_rect.y << " - " << region_rect.x << " : " << region_rect.height << " - " << region_rect.width);
          //LOG4CXX_INFO(this->logger, "<<Current RECT          : " << last_rect.y << " - " << last_rect.x << " : " << last_rect.height << " - " << last_rect.width);
          int max_x = (last_rect.x + last_rect.width) >= (current_rect.x + current_rect.width) ? (last_rect.x + last_rect.width) : (current_rect.x + current_rect.width);
          int max_y = (last_rect.y + last_rect.height) >= (current_rect.y + current_rect.height) ? (last_rect.y + last_rect.height) : (current_rect.y + current_rect.height);
          //if(abs(current_rect.y+current_rect.height-max_y) < this->max_dist)
          //	  max_y=current_rect.y+current_rect.height-this->max_dist;
          int min_x = last_rect.x <= current_rect.x ? last_rect.x : current_rect.x;
          int min_y = last_rect.y <= current_rect.y ? last_rect.y : current_rect.y;
          int current_low_dist = abs(max_y - min_y) > this->low_dist ? this->low_dist : abs(max_y - min_y);
          int current_dist = abs(max_y - min_y) > this->up_dist ? this->up_dist : abs(max_y - min_y);
          //this->search_areas[i].push_back(cv::Rect(cv::Point(min_x, min_y), cv::Point(max_x, max_y)));
          this->search_areas[i].push_back(cv::Rect(last_calc_rect.x, last_calc_rect.y + last_calc_rect.height, last_calc_rect.width, current_low_dist));
          //LOG4CXX_INFO(this->logger, "<<OVERLAP DOWN >> ");
          //show_rectangle(this->search_areas[i][this->search_areas[i].size() - 1]);
          //this->search_areas[i].push_back(cv::Rect(cv::Point(min_x, min_y), cv::Point(max_x, max_y)));
          this->search_areas[i].push_back(cv::Rect(current_rect.x, current_rect.y - current_dist, current_rect.width, current_dist));
          last_calc_rect = this->search_areas[i][this->search_areas[i].size() - 1];
          //LOG4CXX_INFO(this->logger, "<<OVERLAP UP >> ");
          //show_rectangle(this->search_areas[i][this->search_areas[i].size() - 1]);
        }
        else
        {*/
          this->search_areas[i].push_back(cv::Rect(last_rect.x-horizontal_padding, last_rect.y+last_rect.height, last_rect.width+horizontal_padding, last_rect.height+this->low_dist));
          //LOG4CXX_INFO(this->logger, "<<SINGLE DOWN >> ");
          //show_rectangle(this->search_areas[i][this->search_areas[i].size() - 1]);
          this->search_areas[i].push_back(cv::Rect(current_rect.x-horizontal_padding,current_rect.y+current_rect.height-this->up_dist,current_rect.width+horizontal_padding,this->up_dist));
          last_calc_rect = this->search_areas[i][this->search_areas[i].size() - 1];
          //LOG4CXX_INFO(this->logger, "<<SINGLE UP >> ");
          //show_rectangle(this->search_areas[i][this->search_areas[i].size() - 1]);
        //}
    
        last_rect = current_rect;
        //last_rect = this->search_areas[i][j];
      }

      //int last_dist = last_dist = region_rect.y + region_rect.height - 1;

      //if(is_too_far_below(last_rect,bottom_rect_frontier)){
        int last_dist = last_rect.y + this->low_dist;
      //}

      this->search_areas[i].push_back(cv::Rect(last_rect.x-horizontal_padding, last_rect.y, last_rect.width+horizontal_padding, abs((last_rect.y) - (last_dist))));
      //LOG4CXX_INFO(this->logger, "<<LAST UP >> ");
      //show_rectangle(this->search_areas[i][this->search_areas[i].size() - 1]);
      //LOG4CXX_INFO(this->logger, "<<CALC : " << last_rect.y << " - " << (last_dist));
      //		int age;
      //	cin >> age;
    }
  }
}

bool Polyline_Extractor::is_too_far_above(cv::Rect current_rect, cv::Rect compared_to)
{
  if (compared_to.y+compared_to.height-current_rect.y <= -this->up_dist)
    return true;
  return false; 
}
bool Polyline_Extractor::is_too_far_below(cv::Rect current_rect, cv::Rect compared_to){
  if(current_rect.y + current_rect.height - compared_to.y <= -this->low_dist )
    return true;
  return false; 
}
bool Polyline_Extractor::it_overlaps(cv::Rect current_rect, cv::Rect compared_to){
  if (current_rect.x > compared_to.x && current_rect.x < compared_to.x+compared_to.width)
    return true;
  if (current_rect.x + current_rect.width > compared_to.x && current_rect.x + current_rect.width < compared_to.x + compared_to.width)
    return true;
  return false; 
}

void Polyline_Extractor::calculate_search_areas2(vector<vector<cv::Point > > ex_regions, vector<vector<vector<cv::Point> > > ex_baselines)
{
  prhlt::Algorithm_Calculate_Search_Area acsa_instance(this->extract_mat, ex_regions, ex_baselines);
  this->search_areas = acsa_instance.run();
/*
  for (int i = 0; i < this->search_areas.size();i++){
    for (int j = 0; j < this->search_areas[i].size();j++){
      show_rectangle(this->search_areas[i][j]);
    }
  }*/
}

void Polyline_Extractor::calculate_line_frontiers_with_search_areas()
{
  //cv::Mat tmp = this->global_input_image.get_matrix();
  LOG4CXX_INFO(this->logger, "<<NON PARALLEL LINE FRONTIER CALCULATION>>");
  this->area_polylines.resize(this->search_areas.size());
  this->area_collision_points.resize(this->search_areas.size());
  for (int i = 0; i < this->search_areas.size(); i++)
  {
    this->area_polylines[i].resize(this->search_areas[i].size());
    this->area_collision_points[i].resize(this->search_areas[i].size());
    for (int j = 0; j < this->search_areas[i].size(); j++)
    {
      calculate_single_line_frontier_with_search_area(i, j);
    }
  }
}

void Polyline_Extractor::parallel_calculate_line_frontiers_with_search_areas()
{
  this->threads.resize(this->num_workers);
  this->area_polylines.resize(this->search_areas.size());
  this->area_collision_points.resize(this->search_areas.size());
  int work_load, work_remaining, at, from, to;

  for (int i = 0; i < this->search_areas.size(); i++)
  {
    this->area_polylines[i].resize(this->search_areas[i].size());
    this->area_collision_points[i].resize(this->search_areas[i].size());

    work_load = (this->search_areas[i].size()) / this->num_workers;
    work_remaining = (this->search_areas[i].size()) % this->num_workers;
    at = 0;

    //cout << "Total " << this->search_areas[i].size() << " Work load: " << work_load << " work_remaining: " << work_remaining << endl;

    for (int j = 0; j < this->num_workers; ++j)
    {

      if (this->threads[j] != NULL)
        delete this->threads[j];
      if (at < this->search_areas[i].size())
      {
        from = at;
        to = at + max(work_load, 1) - 1;

        if (j == this->num_workers - 1)
          to += work_remaining;
        //cout << "Region: " << i << " computing from: " << from << " to: " << to << endl;
        this->threads[j] = new thread(&prhlt::Polyline_Extractor::calculate_range_line_frontier_with_search_area, this, i, from, to);
        at += max(work_load, 1);
      }
    }
    for (int j = 0; j < this->num_workers; ++j)
    {
      if (this->threads[j] != NULL)
        this->threads[j]->join();
    }
  }
}
void Polyline_Extractor::calculate_range_line_frontier_with_search_area(int i, int from, int to)
{
  for (int j = from; j <= to; ++j)
  {
    calculate_single_line_frontier_with_search_area(i, j);
  }
}

void Polyline_Extractor::calculate_single_line_frontier_with_search_area(int i, int j)
{
  //LOG4CXX_INFO(this->logger, "<<Instantiating >> " << i << " - " << j);
  cv::Mat tmp = this->global_input_image.get_matrix();
  prhlt::Algorithm_DP_Path_Finder path_finder_instance(tmp, this->distance_mat, this->search_areas[i][j]);
  //LOG4CXX_INFO(this->logger, "<<Instantiating Done >> " << i << " - " << j);

  //show_rectangle(this->search_areas[i][j]);
  if (j == 0)
  {
    //path_finder_instance.set_lower_search_limits(this->search_restrictions[i][j]);
    //path_finder_instance.set_lower_search_limits(expand_search_restriction(i, j, 0, -10));
    //LOG4CXX_INFO(this->logger, "<<TOP Adding limits >> " << i << " - " << j);
    path_finder_instance.set_lower_search_limits(expand_my_search_restriction(i, j, 0, -10));
    //LOG4CXX_INFO(this->logger, "<<Adding limits done >> " << i << " - " << j);
  }
  else
  {
    if (j < this->search_areas[i].size() - 1)
    {
      //path_finder_instance.set_search_limits(this->search_restrictions[i][j-1],this->search_restrictions[i][j]);
      //path_finder_instance.set_search_limits(expand_search_restriction(i, j - 1, 0, 0), expand_search_restriction(i, j, 0, -10));
      //LOG4CXX_INFO(this->logger, "<<MID Adding limits >> " << i << " - " << j);
      if(j % 2 == 0){
        path_finder_instance.set_search_limits(expand_other_search_restriction(i, j, 0, 0), expand_my_search_restriction(i, j, 0, -10));
      }
      else{
        path_finder_instance.set_search_limits(expand_my_search_restriction(i, j, 0, 0), expand_other_search_restriction(i, j, 0, 0));
      }
      //LOG4CXX_INFO(this->logger, "<<Adding limits done >> " << i << " - " << j);
    }
    else
    {
      //path_finder_instance.set_upper_search_limits(this->search_restrictions[i][j-1]);
      //LOG4CXX_INFO(this->logger, "<<BOT Adding limits >> " << i << " - " << j);
      path_finder_instance.set_upper_search_limits(expand_search_restriction(i, j - 1, 0, 0));
      //LOG4CXX_INFO(this->logger, "<<Adding limits done >> " << i << " - " << j);
    }
  }
  //LOG4CXX_DEBUG(this->logger, "<<Launching search >>");
  path_finder_instance.run(1.0, 1.0);
  //LOG4CXX_DEBUG(this->logger, "<<Done >>");
  LOG4CXX_INFO(this->logger, "<<RECOVERING BEST PATH >> " << i << " - " << j);
  this->area_polylines[i][j] = path_finder_instance.recover_best_path();
  //LOG4CXX_DEBUG(logger, "Calculating Collision Points ");
  this->area_collision_points[i][j] = path_finder_instance.get_best_path_collision_points();
  //LOG4CXX_DEBUG(logger, "Calculating Collision Points - ENDED " << this->collision_points.size());
  tmp.release();
}

vector<vector<cv::Point> > Polyline_Extractor::expand_my_search_restriction(int reg_index, int res_index, int x_offset, int y_offset)
{
  //LOG4CXX_INFO(this->logger, "<<My limits >> " << reg_index << " - " << res_index);
  //LOG4CXX_INFO(this->logger, "<<LINES >> " << this->search_restrictions[reg_index].size());
  vector<vector<cv::Point> > res;
  for (int i = 0; i < this->search_restrictions[reg_index][res_index].size() - 1; i++)
  {
    vector<cv::Point> tmp;
    tmp.push_back(cv::Point(this->search_restrictions[reg_index][res_index][i].x + x_offset, this->search_restrictions[reg_index][res_index][i].y + y_offset));
    tmp.push_back(cv::Point(this->search_restrictions[reg_index][res_index][i + 1].x + x_offset, this->search_restrictions[reg_index][res_index][i + 1].y + y_offset));
    res.push_back(tmp);
  }
  //LOG4CXX_INFO(this->logger, "<<My limits DONE >> " << reg_index << " - " << res_index);
  //show_lines(res);
  return res;
}

vector<vector<cv::Point> > Polyline_Extractor::expand_other_search_restriction(int reg_index, int res_index, int x_offset, int y_offset)
{
  //LOG4CXX_INFO(this->logger, "<<Other limits >> " << reg_index << " - " << res_index);
  vector<vector<cv::Point> > res;
  for (int i = 0; i < this->search_restrictions[reg_index].size();i++)
  {
    if(i!=res_index && ((res_index % 2 == 0 && res_index+1 != i ) || (res_index % 2 != 0 && res_index-1 != i)))  {
      //LOG4CXX_INFO(this->logger, "<<Other limits LINE >> " << i);
      for (int j = 0; j < this->search_restrictions[reg_index][i].size() - 1;j++){
        //if (this->search_areas[reg_index][res_index].contains(this->search_restrictions[reg_index][i][j])){
          vector<cv::Point> tmp;
          tmp.push_back(cv::Point(this->search_restrictions[reg_index][i][j].x + x_offset, this->search_restrictions[reg_index][i][j].y + y_offset));
          tmp.push_back(cv::Point(this->search_restrictions[reg_index][i][j + 1].x + x_offset, this->search_restrictions[reg_index][i][j + 1].y + y_offset));
          if(cv::clipLine(this->search_areas[reg_index][res_index], tmp[0], tmp[1]))
            res.push_back(tmp);
          //show_lines(res);
        //}
      }
    }
  }
  //show_lines(res);
  //LOG4CXX_INFO(this->logger, "<<Other limits DONE >> " << reg_index << " - " << res_index);
  return res;
}

vector<vector<cv::Point> > Polyline_Extractor::expand_search_restriction(int reg_index, int res_index, int x_offset, int y_offset)
{
  vector<vector<cv::Point> > res;

  for (int i = 0; i < this->search_restrictions[reg_index][res_index].size() - 1; i++)
  {
    vector<cv::Point> tmp;
    tmp.push_back(cv::Point(this->search_restrictions[reg_index][res_index][i].x + x_offset, this->search_restrictions[reg_index][res_index][i].y + y_offset));
    tmp.push_back(cv::Point(this->search_restrictions[reg_index][res_index][i + 1].x + x_offset, this->search_restrictions[reg_index][res_index][i + 1].y + y_offset));
    res.push_back(tmp);
  }
  return res;
}

void Polyline_Extractor::run()
{
  LOG4CXX_INFO(logger, "<<RUNNING>>");
  calculate_frontiers();
  calculate_distance_matrix();
  calculate_line_frontiers();
  generate_line_images();
}

void Polyline_Extractor::calculate_frontiers()
{
  prhlt::Algorithm_SAUVOLA sauvola_instance(this->input_mat);
  this->sauvola_mat = sauvola_instance.run(this->standard_deviation_constant, this->dynamic_range_constant, this->window_size, this->window_size);
  this->global_output_image.load_from_matrix(this->sauvola_mat);
  //LOG4CXX_INFO(this->logger,"SAVING SAUVOLA MAP");
  //this->global_output_image.save_image("sauvola_map.png");
}

void Polyline_Extractor::calculate_distance_matrix()
{
  prhlt::Algorithm_DBOSCH dbosch_instance(this->input_mat);
  dbosch_instance.run(this->curvature_ratio, this->sauvola_mat, this->delta, this->beta);
  //cv::Mat temp = dbosch_instance.get_distance_matrix_as_greyscale_image();
  //this->global_output_image.load_from_matrix(temp);
  LOG4CXX_INFO(this->logger,"SAVING DISTANCE MAP");
  //dbosch_instance.save_distance_matrix_to_file("distance_mat.txt");
  //this->global_output_image.save_image("distance_map.png");
  this->distance_mat = dbosch_instance.get_distance_matrix();
}

void Polyline_Extractor::calculate_line_frontiers()
{
  cv::Mat tmp = this->global_input_image.get_matrix();
  this->polylines.resize(this->search_regions.size());
  this->collision_points.resize(this->search_regions.size());

  for (int i = 0; i < this->search_regions.size(); i++)
  {
    //for(int i = 7; i < this->search_regions.size(); i++){

    LOG4CXX_INFO(this->logger, "<<Region>> " << i << " " << this->search_regions[i][0] << " " << this->search_regions[i][1]);
    prhlt::Algorithm_DP_Path_Finder path_finder_instance(tmp, this->distance_mat, this->search_regions[i][0], 0, this->search_regions[i][1] - this->search_regions[i][0], this->distance_mat.cols());
    LOG4CXX_INFO(this->logger, "<<Setting search limits >>");
    if (i < this->search_regions.size() - 1)
      path_finder_instance.set_lower_search_limits(this->search_restrictions[i]);
    LOG4CXX_INFO(this->logger, "<<Launching search >>");
    path_finder_instance.run(1.0, 1.0);
    LOG4CXX_INFO(this->logger, "<<Done >>");
    //this->polylines.push_back(path_finder_instance.recover_best_path());
    this->polylines[i] = path_finder_instance.recover_best_path();
    LOG4CXX_INFO(logger, "Calculating Collision Points ");
    //this->collision_points.push_back(path_finder_instance.get_best_path_collision_points());
    this->collision_points[i] = path_finder_instance.get_best_path_collision_points();
    LOG4CXX_INFO(logger, "Calculating Collision Points - ENDED " << this->collision_points.size());
  }
}

void Polyline_Extractor::calculate_line_contours()
{
  LOG4CXX_INFO(logger, "<<CALCULATING LINE CONTOURS>>");
  this->line_contours.resize(this->search_restrictions.size());
  for (int i = 0; i < this->search_restrictions.size(); i++)
  {
    for (int j = 0; j < this->search_restrictions[i].size(); j+=2)
    {
      //LOG4CXX_INFO(logger, "<<CALC>> " << i << " - " << j);
      calculate_line_contour(i, j);
    }
  }
  LOG4CXX_INFO(logger, "<<CONTOURS CALCULATED>>");
}

void Polyline_Extractor::calculate_line_contour(int i, int j)
{
  if (this->area_polylines.size() <= i)
  {
    LOG4CXX_ERROR(logger, "Region mismatch between baselines and polylines");
    exit(EXIT_FAILURE);
  }
  if (this->area_polylines[i].size() <= j)
  {
    LOG4CXX_ERROR(logger, "In region " << i << " no upper polyline for " << j);
    exit(EXIT_FAILURE);
  }
  if (this->area_polylines[i].size() <= j + 1)
  {
    LOG4CXX_ERROR(logger, "In region " << i << " no upper polyline for " << j);
    exit(EXIT_FAILURE);
  }

  //vector <cv::Point2d> upper_frontier = get_clipped_contour(i,j,"UP");
  //vector <cv::Point2d> lower_frontier = get_clipped_contour(i,j,"DOWN");
  vector<cv::Point> upper_frontier = vector_2d_to_simp(get_clipped_contour(i, j, "UP"));
  vector<cv::Point> lower_frontier = vector_2d_to_simp(get_clipped_contour(i, j, "DOWN"));
  vector<cv::Point> tmp_contour;

  tmp_contour.insert(tmp_contour.end(), upper_frontier.begin(), upper_frontier.end());
  tmp_contour.insert(tmp_contour.end(), lower_frontier.rbegin(), lower_frontier.rend());
  //LOG4CXX_ERROR(logger, "HAVE CONTOUR");
  vector<cv::Point> tmp_contour2;
  cv::Point2f tmp_contour3[4];
  float tmp_distance = this->approx_dist_error == -1 ? arcLength(tmp_contour, true) * 0.005 : this->approx_dist_error;
  //LOG4CXX_ERROR(logger, "BEFORE SIMP " << tmp_contour.size());
  if(tmp_contour.size() > 0 && upper_frontier.size() > 0 && lower_frontier.size() > 0 ){
 	 approxPolyDP(tmp_contour, tmp_contour2, tmp_distance, true);
  	//LOG4CXX_ERROR(logger, "SIMPLIFIED CONTOUR");
  	if (this->enclosing_rect)
  	{
    	minAreaRect(tmp_contour2).points(tmp_contour3);
    	tmp_contour2.clear();
    	for (int p = 0; p < 4; p++)
    	{
     	 tmp_contour2.push_back(cv::Point(int(tmp_contour3[p].x), int(tmp_contour3[p].y)));
    	}
  	}
  	this->line_contours[i].push_back(tmp_contour2);
  }
  else{
  	vector<cv::Point> tmp_contour5;
  	  this->line_contours[i].push_back(tmp_contour5);
  }

}

vector<cv::Point> Polyline_Extractor::vector_2d_to_simp(const vector<cv::Point2d> &aux_vector)
{
  vector<cv::Point> res;
  for (int i = 0; i < aux_vector.size(); i++)
  {
    res.push_back(cv::Point(aux_vector[i].x, aux_vector[i].y));
  }
  return res;
}

vector<cv::Point2d> Polyline_Extractor::get_clipped_contour(int reg_id, int line_id, string direction)
{
  vector<cv::Point2d> tmp, res;
  //LOG4CXX_INFO(logger,"<<CLIPPING >> " << reg_id << " " << line_id << " " << direction);
  if (direction == "UP")
  {
    tmp = this->area_polylines[reg_id][line_id];
  }
  else
  {
    tmp = this->area_polylines[reg_id][line_id + 1];
  }
  //LOG4CXX_INFO(logger,"<<BASELINE RECT>> " <<  this->search_restrictions[reg_id].size());

  cv::Rect baseline_rect = cv::boundingRect(this->search_restrictions[reg_id][line_id]);
  int min_x = baseline_rect.x - 15;
  int max_x = baseline_rect.x + baseline_rect.width + 15;
  for (int i = 0; i < tmp.size(); i++)
  {
    if (tmp[i].y >= min_x && tmp[i].y <= max_x)
      res.push_back(tmp[i]);
  }
  //LOG4CXX_INFO(logger,"<<HAD: " << this.
  return res;
}

void Polyline_Extractor::generate_line_images()
{
  LOG4CXX_INFO(logger, "<<GENERATING LINE IMAGES>>");
  for (int i = 0; i < this->polylines.size() - 1; i++)
  {
    extract_line(i, i + 1);
  }
  //extract_line_full_page(i,i+1);
  this->images_calculated = true;
  LOG4CXX_INFO(logger, "<<IMAGES GENERATED>>");
}

void Polyline_Extractor::extract_line(int start_frontier_index, int end_frontier_index)
{
  vector<cv::Point2d> upper_frontier = this->polylines[start_frontier_index];
  vector<cv::Point2d> lower_frontier = this->polylines[end_frontier_index];
  LOG4CXX_INFO(logger, "<<Extracting Line >> " << start_frontier_index << " - " << end_frontier_index);
  vector<vector<int> > upper_sequenced = sequence_frontier(upper_frontier, start_frontier_index, "UP");
  vector<vector<int> > lower_sequenced = sequence_frontier(lower_frontier, end_frontier_index, "DOWN");
  //   LOG4CXX_INFO(logger,"< Done sequencing  >" );
  int high_point = get_highest_point_in_path(upper_sequenced);
  int low_point = get_lowest_point_in_path(lower_sequenced);

  //   LOG4CXX_INFO(logger,"< Creating matrix  >" );
  // this->image = cv::Mat(ex_image.rows,ex_image.cols,CV_8U,0);
  this->line_images.push_back(cv::Mat(low_point - high_point, this->input_mat.cols, CV_8UC3, cv::Scalar(255, 255, 255)));
  //this->mask_images.push_back(cv::Mat::zeros(low_point-high_point,this->input_mat.cols,CV_8UC1));
  this->mask_images.push_back(cv::Mat(low_point - high_point, this->input_mat.cols, CV_8UC1, cv::Scalar(0)));
  //   LOG4CXX_INFO(logger,"< Done  >" );

  for (int i = 0; i < this->extract_mat.cols; i++)
  {

    if (upper_sequenced[i].size() > 1)
    {
      for (int j = 0; j < upper_sequenced[i].size(); j += 2)
        if (j + 1 < upper_sequenced[i].size())
        {
          copy_region_to_last_image(i, upper_sequenced[i][j], upper_sequenced[i][j + 1], high_point);
          mask_region(i, upper_sequenced[i][j], upper_sequenced[i][j + 1], high_point, 255);
          label_region(i, upper_sequenced[i][j], upper_sequenced[i][j + 1], start_frontier_index + 1);
        }
    }
    mask_region(i, upper_sequenced[i].back(), lower_sequenced[i].front(), high_point, 255);
    copy_region_to_last_image(i, upper_sequenced[i].back(), lower_sequenced[i].front(), high_point);
    label_region(i, upper_sequenced[i].back(), lower_sequenced[i].front(), start_frontier_index + 1);

    if (lower_sequenced[i].size() > 1)
      for (int j = 1; j < lower_sequenced[i].size(); j += 2)
        if (j + 1 < lower_sequenced[i].size())
        {
          copy_region_to_last_image(i, lower_sequenced[i][j], lower_sequenced[i][j + 1], high_point);
          mask_region(i, lower_sequenced[i][j], lower_sequenced[i][j + 1], high_point, 255);
          label_region(i, lower_sequenced[i][j], lower_sequenced[i][j + 1], start_frontier_index + 1);
        }
  }
  //   LOG4CXX_INFO(logger,"< Done extracting line >" );
}

void Polyline_Extractor::extract_line_full_page(int start_frontier_index, int end_frontier_index)
{
  vector<cv::Point2d> upper_frontier = this->polylines[start_frontier_index];
  vector<cv::Point2d> lower_frontier = this->polylines[end_frontier_index];
  LOG4CXX_INFO(logger, "<<Extracting Line >> " << start_frontier_index << " - " << end_frontier_index);
  vector<vector<int> > upper_sequenced = sequence_frontier(upper_frontier, start_frontier_index, "UP");
  vector<vector<int> > lower_sequenced = sequence_frontier(lower_frontier, end_frontier_index, "DOWN");
  //   LOG4CXX_INFO(logger,"< Done sequencing  >" );
  int high_point = 0;
  int low_point = this->input_mat.rows;

  //   LOG4CXX_INFO(logger,"< Creating matrix  >" );
  // this->image = cv::Mat(ex_image.rows,ex_image.cols,CV_8U,0);
  this->line_images.push_back(cv::Mat(this->input_mat.rows, this->input_mat.cols, CV_8UC3, 255));
  //   LOG4CXX_INFO(logger,"< Done  >" );

  for (int i = 0; i < this->extract_mat.cols; i++)
  {

    if (upper_sequenced[i].size() > 1)
    {
      for (int j = 0; j < upper_sequenced[i].size(); j += 2)
        if (j + 1 < upper_sequenced[i].size())
        {
          copy_region_to_last_image(i, upper_sequenced[i][j], upper_sequenced[i][j + 1], high_point);
          label_region(i, upper_sequenced[i][j], upper_sequenced[i][j + 1], start_frontier_index + 1);
        }
    }
    copy_region_to_last_image(i, upper_sequenced[i].back(), lower_sequenced[i].front(), high_point);
    label_region(i, upper_sequenced[i].back(), lower_sequenced[i].front(), start_frontier_index + 1);

    if (lower_sequenced[i].size() > 1)
      for (int j = 1; j < lower_sequenced[i].size(); j += 2)
        if (j + 1 < lower_sequenced[i].size())
        {
          copy_region_to_last_image(i, lower_sequenced[i][j], lower_sequenced[i][j + 1], high_point);
          label_region(i, lower_sequenced[i][j], lower_sequenced[i][j + 1], start_frontier_index + 1);
        }
  }
  //   LOG4CXX_INFO(logger,"< Done extracting line >" );
}

void Polyline_Extractor::copy_region_to_last_image(int column, int start_row, int end_row, int high_point)
{
  for (int j = start_row; j < end_row; j++)
    this->line_images[this->line_images.size() - 1].at<cv::Vec3b>(j - high_point, column) = this->extract_mat.at<cv::Vec3b>(j, column);
  //this->line_images[this->line_images.size()-1].at<uchar>(j-high_point,column)= this->extract_mat.at<uchar>(j,column);
}

void Polyline_Extractor::label_region(int column, int start_row, int end_row, int line_number)
{
  for (int j = start_row; j < end_row; j++)
    if (this->extract_mat.at<uchar>(j, column) == 0)
      this->label_image[j * this->extract_mat.cols + column] = line_number;
}

void Polyline_Extractor::mask_region(int column, int start_row, int end_row, int high_point, int mask_value)
{
  for (int j = start_row; j < end_row; j++)
    this->mask_images[this->mask_images.size() - 1].at<uchar>(j - high_point, column) = (uchar)mask_value;
}

vector<vector<int> > Polyline_Extractor::sequence_frontier(const vector<cv::Point2d> &frontier, int frontier_index, string mode)
{

  //    LOG4CXX_INFO(logger,"< Sequencing  : " << frontier_index );
  vector<vector<int> > tmp(this->extract_mat.cols);

  for (int i = 0; i < frontier.size(); i++)
    tmp[frontier[i].y].push_back(frontier[i].x);

  for (int i = 0; i < tmp.size(); i++)
    sort(tmp[i].begin(), tmp[i].end());

  cicular_sequenced_frontier_correction(tmp, frontier_index, mode, 20);
  //  LOG4CXX_INFO(logger, "DONE");

  return tmp;
}

void Polyline_Extractor::cicular_sequenced_frontier_correction(vector<vector<int> > &sequence, int frontier_index, string mode, int radius)
{
  LOG4CXX_INFO(logger, "Correcting Collision Points " << frontier_index << " - " << this->collision_points.size());
  if ((0 <= frontier_index) && (frontier_index < this->collision_points.size()))
  {
    vector<cv::Point2d> collisions = this->collision_points[frontier_index];
    if (collisions.size() > 0)
    {
      //    LOG4CXX_INFO(logger, "IN  " << collisions.size());
      int upper_line_limit = this->line_limits[frontier_index][1];
      int lower_line_limit = this->line_limits[frontier_index + 1][0];
      int direction = (mode == "UP") ? -1 : 1;
      int r2 = radius * radius;
      int x;

      for (int i = 0; i < collisions.size(); i++)
      {
        //      LOG4CXX_INFO(logger, "INNER IN "<< i  << " : "<< this->line_limits[frontier_index].size());
        //      LOG4CXX_INFO(logger, "INNER IN " << this->line_limits[frontier_index].size());
        //  if((collisions[i].x <= this->line_limits[frontier_index][1])|| (collisions[i].x >= this->line_limits[frontier_index+1][0])){
        for (int y = 1; y <= radius; y++)
        {
          x = collisions[i].x + direction * (int)(sqrt(r2 - y * y) + 0.5);
          //    LOG4CXX_INFO(logger, "Old value (+y): " << sequence[collisions[i].y+y].front() << " - " <<  sequence[collisions[i].y+y].back())   ;
          //    LOG4CXX_INFO(logger, "Old value (-y): " << sequence[collisions[i].y-y].front() << " - " <<  sequence[collisions[i].y-y].back())   ;
          //    LOG4CXX_INFO(logger, "New value: " << x ) ;
          if ((0 <= collisions[i].y + y) && (collisions[i].y + y < sequence.size()))
            if (((mode == "UP") && x < sequence[collisions[i].y + y].front()) ||
                ((mode == "DOWN") && x > sequence[collisions[i].y + y].back()))
            {
              //      LOG4CXX_INFO(logger, "Modified (+y) ") ;
              sequence[collisions[i].y + y].clear();
              sequence[collisions[i].y + y].push_back(x);
            }
          if ((0 <= collisions[i].y - y) && (collisions[i].y - y < sequence.size()))
            if (((mode == "UP") && x < sequence[collisions[i].y - y].front()) ||
                ((mode == "DOWN") && x > sequence[collisions[i].y - y].back()))
            {
              //      LOG4CXX_INFO(logger, "Modified (-y) ") ;
              sequence[collisions[i].y - y].clear();
              sequence[collisions[i].y - y].push_back(x);
            }
        }
        //}
      }
    }
  }
}

int Polyline_Extractor::get_highest_point_in_path(const vector<vector<int> > &sequenced_path)
{
  int highest_x = sequenced_path[0].front();
  //LOG4CXX_INFO(logger, "path " << 0 << " - " << path[0].x);
  for (int i = 1; i < sequenced_path.size(); i++)
  {
    //LOG4CXX_INFO(logger, "path " << i << " - " << path[i].x);
    if (sequenced_path[i].front() < highest_x)
      highest_x = sequenced_path[i].front();
  }
  return highest_x;
}

int Polyline_Extractor::get_lowest_point_in_path(const vector<vector<int> > &sequenced_path)
{
  int lowest_x = sequenced_path[0].back();
  for (int i = 1; i < sequenced_path.size(); i++)
  {
    if (sequenced_path[i].back() > lowest_x)
      lowest_x = sequenced_path[i].back();
  }
  return lowest_x;
}

int Polyline_Extractor::get_highest_point_in_path(const vector<cv::Point2d> &path)
{
  int highest_x = path[0].x;
  //LOG4CXX_INFO(logger, "path " << 0 << " - " << path[0].x);
  for (int i = 1; i < path.size(); i++)
  {
    //LOG4CXX_INFO(logger, "path " << i << " - " << path[i].x);
    if (path[i].x < highest_x)
      highest_x = path[i].x;
  }
  return highest_x;
}

int Polyline_Extractor::get_lowest_point_in_path(const vector<cv::Point2d> &path)
{
  int lowest_x = path[0].x;
  for (int i = 1; i < path.size(); i++)
  {
    if (path[i].x > lowest_x)
      lowest_x = path[i].x;
  }
  return lowest_x;
}

void Polyline_Extractor::save_line_images_to_file(string base_file_name)
{
  for (int i = 0; i < this->line_images.size(); i++)
  {
    stringstream ss; //create a stringstream
    ss << base_file_name << "_" << (i + 1 < 10 ? "0" : "") << i + 1 << ".pgm";
    cv::imwrite(ss.str(), this->line_images[i]);
  }
}

void Polyline_Extractor::save_line_images_with_alpha_to_file(string base_file_name)
{
  for (int i = 0; i < this->line_images.size(); i++)
  {
    stringstream ss; //create a stringstream
    ss << base_file_name << "_" << (i + 1 < 10 ? "0" : "") << i + 1 << ".png";

    //stringstream ss_mask;//create a stringstream
    //ss_mask << base_file_name << "_mask_" << (i+1 < 10 ? "0" : "") << i+1 << ".png";
    cv::Mat srcImg[] = {this->line_images[i], this->mask_images[i]};
    int from_to[] = {0, 0, 1, 1, 2, 2, 3, 3};
    cv::Mat final(this->line_images[i].rows, this->line_images[i].cols, CV_8UC4);
    cv::mixChannels(srcImg, 2, &final, 1, from_to, 4);
    cv::imwrite(ss.str(), final);
    //cv::imwrite(ss_mask.str(),this->mask_images[i]);
    final.release();
  }
}

void Polyline_Extractor::save_global_polylines_image(cv::Mat tmp, string file_name)
{
  this->global_output_image.load_from_matrix(tmp);
  for (int i = 0; i < this->polylines.size(); i++)
    this->global_output_image.draw_randomed_colored_region(this->polylines[i]);
  for (int i = 0; i < this->collision_points.size(); i++)
    if (this->collision_points[i].size() > 0)
      this->global_output_image.encircle_points(this->collision_points[i], 20, cv::Scalar(0, 0, 0));
  this->global_output_image.save_image(file_name);
}

void Polyline_Extractor::save_global_polylines_image(string file_name)
{
  cv::Mat tmp = this->global_extract_image.get_matrix();
  this->global_output_image.load_from_matrix(tmp);
  for (int i = 0; i < this->polylines.size(); i++)
    this->global_output_image.draw_randomed_colored_region(this->polylines[i]);
  /* for(int i = 0 ; i < this->line_limits.size();i++){
              this->global_output_image.draw_line(cv::Point2d(0,this->line_limits[i][0]),cv::Point2d(this->distance_mat.cols()-1,this->line_limits[i][0]),cv::Scalar(0,255,0));
              this->global_output_image.draw_line(cv::Point2d(0,this->line_limits[i][1]),cv::Point2d(this->distance_mat.cols()-1,this->line_limits[i][1]),cv::Scalar(0,255,0));
        }*/
  for (int i = 0; i < this->search_regions.size(); i++)
  {
    this->global_output_image.draw_line(cv::Point2d(0, this->search_regions[i][0]), cv::Point2d(this->distance_mat.cols() - 1, this->search_regions[i][0]), cv::Scalar(0, 0, 255));
    this->global_output_image.draw_line(cv::Point2d(0, this->search_regions[i][1]), cv::Point2d(this->distance_mat.cols() - 1, this->search_regions[i][1]), cv::Scalar(255, 0, 0));
  }
  for (int i = 0; i < this->collision_points.size(); i++)
    if (this->collision_points[i].size() > 0)
      this->global_output_image.encircle_points(this->collision_points[i], 20, cv::Scalar(0, 0, 255));
  this->global_output_image.save_image(file_name);
}

void Polyline_Extractor::show_rectangle(cv::Rect ex_rect)
{
  cv::Mat tmp = this->global_input_image.get_matrix();
  this->global_output_image.load_from_matrix(tmp);
  this->global_output_image.draw_rectangle(ex_rect, cv::Scalar(0, 0, 255));
  this->global_output_image.display_with_scale("Rectangle", 0.5, 500000, true);
}

void Polyline_Extractor::show_lines(vector<vector<cv::Point> > ex_lines)
{
  cv::Mat tmp = this->global_input_image.get_matrix();
  this->global_output_image.load_from_matrix(tmp);

  for (int i = 0; i < ex_lines.size(); i++)
  {
    this->global_output_image.draw_line(cv::Point2d(ex_lines[i][0].x, ex_lines[i][0].y), cv::Point2d(ex_lines[i][1].x, ex_lines[i][1].y), cv::Scalar(0, 255, 0));
  }
  this->global_output_image.display_with_scale("Rectangle", 0.7, 500000, true);
}

void Polyline_Extractor::show_polylines()
{
  //            LOG4CXX_INFO(logger,"<<DISPLAYING IMAGE>>");
  cv::Mat tmp = this->global_input_image.get_matrix();
  this->global_output_image.load_from_matrix(tmp);
  for (int i = 0; i < this->polylines.size(); i++)
  {
    //                LOG4CXX_INFO(logger,"Path size is :  " << this->polylines[i].size());
    this->global_output_image.draw_randomed_colored_region(this->polylines[i]);
  }
  this->global_output_image.display_with_scale("Distance Map", 0.5, 500000, true);
}

void Polyline_Extractor::save_labeled_image_to_file(string file_name)
{
  FILE *f1;
  f1 = fopen(file_name.c_str(), "wb");
  fwrite(this->label_image, this->input_mat.cols * this->input_mat.rows, sizeof(int), f1);
  fclose(f1);
}

void Polyline_Extractor::save_polylines_tool_format(string file_name)
{
  std::ofstream file(file_name.c_str());
  if (file.is_open())
  {
    file << "# Number of lines and type of points \n";
    file << this->polylines.size() << " Num\n";
    for (int i = 0; i < this->polylines.size(); i++)
    {
      file << "# Number of points \n";
      file << this->polylines[i].size() << "\n";
      file << "# Points \n";
      for (int j = 0; j < this->polylines[i].size(); j++)
      {
        file << this->polylines[i][j].y << " " << this->polylines[i][j].x << " " << i << "\n";
      }
    }
  }
  else
    LOG4CXX_ERROR(this->logger, "ERROR - Failed to write file: " << file_name);

  if (file)
    file.close();
}

void Polyline_Extractor::load_polylines(vector<vector<cv::Point2f> > ex_polylines)
{
  LOG4CXX_INFO(logger, "<<LOADING POLYLINES>>");
  if (this->images_calculated)
  {
    for (int i = 0; i < this->line_images.size(); i++)
      this->line_images[i].release();
    for (int i = 0; i < this->polylines.size(); i++)
      this->polylines[i].clear();
    for (int i = 0; i < this->collision_points.size(); i++)
      this->collision_points[i].clear();
  }
  //    LOG4CXX_INFO(logger,"<<REVIEWING POLYLINES>>");
  this->polylines = review_loaded_polylines(ex_polylines);
  generate_line_images();
}

vector<vector<cv::Point2d> > Polyline_Extractor::review_loaded_polylines(vector<vector<cv::Point2f> > ex_polylines)
{
  vector<vector<cv::Point2d> > reviewed_polylines;
  vector<cv::Point2d> tmp_polyline;
  vector<cv::Point2d> tmp_expanded;

  for (int i = 0; i < ex_polylines.size(); i++)
  {
    if (ex_polylines[i].size() > 1)
    {
      tmp_polyline.push_back(cv::Point2d(ex_polylines[i][0].y, ex_polylines[i][0].x));
      for (int j = 1; j < ex_polylines[i].size(); j++)
      {
        //LOG4CXX_INFO(logger,"<<EXPANDING>>");
        tmp_expanded = expand_points(ex_polylines[i][j - 1], ex_polylines[i][j]);
        //LOG4CXX_INFO(logger,"<<INSERTING POINTS>> " << (tmp_expanded.size()-1));
        tmp_polyline.insert(tmp_polyline.end(), tmp_expanded.begin() + 1, tmp_expanded.end());
        //tmp_polyline.push_back(cv::Point2d(ex_polylines[i][j].y,ex_polylines[i][j].x));
        tmp_expanded.clear();
      }
      reviewed_polylines.push_back(tmp_polyline);
      //        LOG4CXX_INFO(logger,"<<INSERTING POLYGON>> " << tmp_polyline.size());
      tmp_polyline.clear();
    }
  }

  return reviewed_polylines;
}

vector<cv::Point2d> Polyline_Extractor::expand_points(cv::Point2f from, cv::Point2f to)
{
  vector<cv::Point2d> tmp_expanded;
  cv::LineIterator iter(this->extract_mat, from, to, 8);
  for (int i = 0; i < iter.count; i++, ++iter)
  {
    tmp_expanded.push_back(cv::Point2d(iter.pos().y, iter.pos().x));
  }
  return tmp_expanded;
}

} // namespace prhlt
