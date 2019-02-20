#include "algorithm_extract_polygons.hpp"

namespace prhlt{
    using namespace log4cxx;
    using namespace log4cxx::helpers;
    using namespace boost;

    Algorithm_Extract_Polygons::Algorithm_Extract_Polygons(cv::Mat &ex_image){
        this->logger = Logger::getLogger("PRHLT.Algorithm_Ext_Poly");
        this->input_image= ex_image.clone();
        this->output_image= ex_image.clone();
    }

    Algorithm_Extract_Polygons::~Algorithm_Extract_Polygons(){
        this->input_image.release();
        this->output_image.release();
        for(int i = 0; i < this->polygon_list.size();i++)
        	this->polygon_list[i].clear();
        this->polygon_list.clear();
    }
    void Algorithm_Extract_Polygons::add_polygons( vector <vector< vector <cv::Point2f> > > ex_polygons){
        for(int i = 0; i < ex_polygons.size();i++)
        {
          for (int j = 0; j < ex_polygons[i].size(); j++)
          {
            add_polygon(ex_polygons[i][j]);
          }
        }
    }

    void Algorithm_Extract_Polygons::add_polygons( vector < vector< cv::Point2f> > ex_polygons){
        for(int i = 0; i < ex_polygons.size();i++)
            add_polygon(ex_polygons[i]);

    }
    void Algorithm_Extract_Polygons::add_polygon( vector< cv::Point2f> ex_polygon){
        this->polygon_list.push_back(ex_polygon);
    }

    cv::Mat Algorithm_Extract_Polygons::run(){
        if (this->polygon_list.size()>0)
            calculate_output_image();
        return this->output_image;
    }

    void Algorithm_Extract_Polygons::calculate_output_image(){
        for(int r = 0; r < this->output_image.rows; r++)
            for(int c = 0; c < this->output_image.cols; c++){
                if(not is_point_in_polygon(cv::Point2f(c,r))){
                    paint_output_pixel_white(r,c);
                }

            }
    }

    void Algorithm_Extract_Polygons::paint_output_pixel_white(int row , int col){
        if(this->output_image.channels()>1){
            this->output_image.at<cv::Vec3b>(row,col)[0]= 255;
            this->output_image.at<cv::Vec3b>(row,col)[1]= 255;
            this->output_image.at<cv::Vec3b>(row,col)[2]= 255;
        }else
            this->output_image.at<uchar>(row,col)=255;
    }


    bool Algorithm_Extract_Polygons::is_point_in_polygon(cv::Point2f ex_point){

        for(int i = 0; i < this->polygon_list.size();i++)
            if(pointPolygonTest(this->polygon_list[i],ex_point,false) >=0)
                return true;

        return false;
    }


}
