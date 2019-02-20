#include "points_file.hpp"

using namespace log4cxx;
using namespace log4cxx::helpers;
using namespace boost;
namespace prhlt{
    
    Points_File::Points_File(string ex_file_name){
        this->logger = Logger::getLogger("PRHLT.Points_File");
        this->file_name = ex_file_name;
        load_file();
    }
    
    Points_File::~Points_File(){
    	for(int i = 0; i < this->points.size(); i++)
    		this->points[i].clear();
        this->points.clear();
    }
    
    vector <vector<cv::Point2f> > Points_File::get_points(){
        return this->points;
    }
    
    int Points_File::get_num_points(){
        return this->points.size();
    }
    
    vector< vector<cv::Point2f> > Points_File::get_points_as_polygons(int points_per_polygon){
        vector < vector<cv::Point2f> > res;
        vector < cv::Point2f> all_points;

    	for(int i = 0; i < this->points.size(); i++)
    		all_points.insert(all_points.end(),this->points[i].begin(),this->points[i].end());

        if (all_points.size() % points_per_polygon !=0){
            LOG4CXX_ERROR(logger, "<<!>> Could return the points loaded as polygons of " << points_per_polygon << " vertices" );
        }else{
            vector <cv::Point2f> tmp;
            for(int i =0 ; i < all_points.size();i+=points_per_polygon){
                for(int j = 0; j < points_per_polygon;j++){
                    tmp.push_back(all_points[i+j]);
                }
                res.push_back(tmp);
                tmp.clear();
            }

        }

        return res;
    }
 
    
    void Points_File::load_file(){
      ifstream file;
      file.open(this->file_name.c_str());
      string temp_line;
      int num_figures=-1;
      int num_points=0;
      int i;
      vector<string> string_values;
      vector<cv::Point2f> temp_pol;
      while(std::getline(file,temp_line)){
          string_values.clear();
          trim(temp_line);
          //cout << temp_line << endl;
          if(not is_comment_line(temp_line)){
              boost::split(string_values,temp_line,boost::is_any_of(" "),boost::token_compress_on);
              if(num_figures > 0){
                  if(is_num_points_in_figure_line(string_values) && num_points == 0){
                  	  num_points = extract_num_points_in_figure(string_values);
                  	  if(!(temp_pol.size()==0 && this->points.size()==0)){
                  	  	this->points.push_back(temp_pol);
                  	  	num_figures--;
                  	  }
                  	  temp_pol.clear();
				  }
				  else{
                  	if(is_point_line(string_values)){
                      temp_pol.push_back(string_2_point(string_values));
                      num_points--;
                    }
				  }
              }
              else{
                  if(is_num_figures_line(string_values)){
                      num_figures=extract_num_figures_in_file(string_values);
				  }
              }
          }
          //cin << i ; 
      }
	  if(temp_pol.size() > 0){
	  	  this->points.push_back(temp_pol);
	  	  temp_pol.clear();
      }
      LOG4CXX_INFO(logger, "Loaded  " << this->points.size() << " of " << num_figures );
      for(int i = 0; i < this->points.size(); i++)
      	LOG4CXX_INFO(logger, "Figure  " << i << " of " << this->points[i].size());

    }
    
    cv::Point2f Points_File::string_2_point(vector<string> string_values){
        cv::Point2f tmp;
        istringstream(string_values[0]) >> tmp.x;
        istringstream(string_values[1]) >> tmp.y;
        return tmp; 
    }
    
    bool Points_File::is_num_points_in_figure_line(vector<string> string_values){
        if(string_values.size()==1 && not is_comment_line(string_values[0]))
            return true;
        return false;
	}
    
    int Points_File::extract_num_figures_in_file(vector<string> string_values){
        int i;
        istringstream(string_values[0]) >> i;
        return i;
    }
    int Points_File::extract_num_points_in_figure(vector<string> string_values){
        int i;
        istringstream(string_values[0]) >> i;
        return i;
    }
    
    bool Points_File::is_comment_line(string line){
        if(line[0] == '#' )
            return true;
        return false;
    }
    
    bool Points_File::is_num_figures_line(vector<string> string_values){
        if((string_values.size() == 2) && ((string_values[1]=="Text") || (string_values[1]=="Num")))
            return true;
        return false;
    }
    
    bool Points_File::is_point_line(vector<string> string_values){
        if(string_values.size()==3)
            return true;
        return false;
    }
}
