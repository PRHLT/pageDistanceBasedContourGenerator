#include "algorithm_distance_map.hpp"

namespace prhlt {
    using namespace log4cxx;
    using namespace log4cxx::helpers;
    using namespace boost;
    using namespace Eigen;
    Algorithm_Distance_Map::Algorithm_Distance_Map(cv::Mat & ex_image){
    	this->min_change=1; 
        this->logger = Logger::getLogger("PRHLT.Algorithm_Distance_Map");
        if(ex_image.channels()>1){
            this->image = cv::Mat(ex_image.rows,ex_image.cols,CV_8U,cv::Scalar(0,0,0));
            cvtColor(ex_image, this->image, CV_RGB2GRAY);
        }
        else{
            this->image= ex_image.clone();
        }
    }

    Algorithm_Distance_Map::~Algorithm_Distance_Map(){
        this->image.release();
    }
    void Algorithm_Distance_Map::initialize_distance_matrix(int threshold){
        LOG4CXX_DEBUG(this->logger,"<<Distance_Map: Initializing distance matrix>>");
       double max_val = this->image.rows*this->image.cols;
       //double max_val = 255.0;
        this->distance_matrix.setConstant(this->image.rows,this->image.cols,max_val);
		    for(int r = 0; r < this->distance_matrix.rows();r++)
		        for(int c = 0; c < this->distance_matrix.cols();c++)
		            if ((int)this->image.at<uchar>(r,c)<=threshold)
		                this->distance_matrix(r,c)=0;
		   //         else
		   //             this->distance_matrix(r,c)=this->image.at<uchar>(r,c);
//		    initialize_border();
        LOG4CXX_DEBUG(this->logger,"<<Distance_Map: Finished initializing distance matrix>>");

    }
   
    void Algorithm_Distance_Map::initialize_distance_matrix(cv::Mat frontier_points_mat){
        LOG4CXX_DEBUG(this->logger,"<<Distance_Map: Initializing distance matrix>>");
        double max_val = this->image.rows*this->image.cols;
        //double max_val = 255.0;
        this->distance_matrix.setConstant(this->image.rows,this->image.cols,max_val);
		    for(int r = 0; r < this->distance_matrix.rows();r++)
		        for(int c = 0; c < this->distance_matrix.cols();c++)
		            if ((int)frontier_points_mat.at<uchar>(r,c)==0)
		                this->distance_matrix(r,c)=0;
//		    initialize_border();
        LOG4CXX_DEBUG(this->logger,"<<Distance_Map: Finished initializing distance matrix>>");
    }
    
    void Algorithm_Distance_Map::initialize_border(){
        for(int c = 0; c < this->distance_matrix.cols();c+=1){
            this->distance_matrix(0,c)=0;
            this->distance_matrix(this->distance_matrix.rows()-1,c)=0;
        }
    }
		
		void Algorithm_Distance_Map::run(int ex_curvature_ratio,int ex_threshold){
		    this->curvature_ratio=ex_curvature_ratio;
		    initialize_distance_matrix(ex_threshold);
		    calculate_distance_map();
    }
		
		void Algorithm_Distance_Map::run(int ex_curvature_ratio,cv::Mat frontier_points_mat){
		    this->curvature_ratio=ex_curvature_ratio;
		    initialize_distance_matrix(frontier_points_mat);
		    //save_distance_matrix_to_file("ini_mat.txt");
		    calculate_distance_map();
    }
    void Algorithm_Distance_Map::calculate_distance_map(){
        LOG4CXX_INFO(this->logger,"<<Distance_Map: Running!!>>");
		    do{
		        reset_distance_matrix_changed_flag();
		        forward_raster_sequence();
		        backward_raster_sequence();
        }while(not solution_converged());
        LOG4CXX_INFO(this->logger,"<<Distance_Map: Convergence reached!!>>");
    }


		void Algorithm_Distance_Map::forward_raster_sequence(){
        LOG4CXX_INFO(this->logger,"<<Distance_Map: Forward raster sequence: " << this->distance_matrix.rows() << " X " << this->distance_matrix.cols() <<" >>");
		    for(int r = 0; r < this->distance_matrix.rows();r++)
		        for(int c = 0; c < this->distance_matrix.cols();c++)
		            update_value_forward(r,c);
    }
		void Algorithm_Distance_Map::backward_raster_sequence(){
        LOG4CXX_INFO(this->logger,"<<Distance_Map: Backward raster sequence>>");
		    for(int r = this->distance_matrix.rows()-1; r >=0;r--)
		        for(int c = this->distance_matrix.cols()-1; c >=0 ;c--)
		            update_value_backward(r,c);
    }
    void Algorithm_Distance_Map::update_value_forward(int r,int c){
        float old_value = this->distance_matrix(r,c);
        LOG4CXX_DEBUG(this->logger,"Updating cell: " << r << " " << c);
        if(r>=1){
            this->distance_matrix(r,c)=get_minimum_for_update(r,c,r-1,c);
            if(c<this->distance_matrix.cols()-1)
                this->distance_matrix(r,c)=get_minimum_for_update(r,c,r-1,c+1);
            if(c>=1)
                this->distance_matrix(r,c)=get_minimum_for_update(r,c,r-1,c-1);
        }
        if(c>=1)
            this->distance_matrix(r,c)=get_minimum_for_update(r,c,r,c-1);
        if(abs(old_value - this->distance_matrix(r,c))>=this->min_change)
            activate_distance_matrix_changed_flag();
    }
    void Algorithm_Distance_Map::update_value_backward(int r,int c){
        float old_value = this->distance_matrix(r,c);
        
        if(r<this->distance_matrix.rows()-1){
            this->distance_matrix(r,c)=get_minimum_for_update(r,c,r+1,c);
            if(c<this->distance_matrix.cols()-1)
                this->distance_matrix(r,c)=get_minimum_for_update(r,c,r+1,c+1);
            if(c>=1)
                this->distance_matrix(r,c)=get_minimum_for_update(r,c,r+1,c-1);
        }
        if(c<this->distance_matrix.cols()-1)
            this->distance_matrix(r,c)=get_minimum_for_update(r,c,r,c+1);
        //if(old_value!=this->distance_matrix(r,c))
        //    activate_distance_matrix_changed_flag();
        if(abs(old_value - this->distance_matrix(r,c))>=this->min_change)
            activate_distance_matrix_changed_flag();
    }
    float Algorithm_Distance_Map::get_minimum_for_update(int r1, int c1, int r2, int c2){
        float neighbour_value = calculate_neighbour_value(r1,c1,r2,c2);
        float current_value = this->distance_matrix(r1,c1);
        LOG4CXX_DEBUG(this->logger,"<<Distance_Map: Get Minimum for update:" << r2 << " - " << c2 << " : " << current_value << "->"  <<  neighbour_value );
        if (neighbour_value < current_value)
            return neighbour_value;
        else
            return current_value;
    }
    float  Algorithm_Distance_Map::calculate_neighbour_value(int r1,int c1, int r2, int c2){
        LOG4CXX_DEBUG(this->logger,"<<Distance_Map: calculate neighbour value>>");
    }
		bool Algorithm_Distance_Map::solution_converged(){
		    return not this->distance_matrix_changed_flag;
    }
    void Algorithm_Distance_Map::activate_distance_matrix_changed_flag(){
        this->distance_matrix_changed_flag=true;
    }
    void Algorithm_Distance_Map::reset_distance_matrix_changed_flag(){
        this->distance_matrix_changed_flag=false;
    }
    
    cv::Mat Algorithm_Distance_Map::get_distance_matrix_as_greyscale_image(){
        LOG4CXX_INFO(this->logger,"<<Distance_Map: Converting image distance matrix>>");
        //cv::Mat tmp(this->distance_matrix.rows(),this->distance_matrix.cols(),CV_8U,0);
        //tmp = this->image.clone();
        int value = 0;

		    for(int r = 0; r < this->distance_matrix.rows();r++)
		        for(int c = 0; c < this->distance_matrix.cols();c++){
                //LOG4CXX_DEBUG(this->logger,"<<Distance_Map: Accesing element: "  << r << " - " << c << " : " << this->image.at<uchar>(r,c));
		            this->image.at<uchar>(r,c)= math::iround((this->distance_matrix(r,c) > 255.0) ? 255.0 : this->distance_matrix(r,c));
		            //this->image.at<uchar>(r,c)=this->distance_matrix(r,c);
            }
        LOG4CXX_INFO(this->logger,"<<Distance_Map: Finished converting image distance matrix>>");
        return this->image;
    }
    
    MatrixXd Algorithm_Distance_Map::get_distance_matrix(){
         return this->distance_matrix;
    }
    
    void Algorithm_Distance_Map::save_image_matrix_to_file(string ex_filename){
      std::ofstream file(ex_filename.c_str());
      if (file.is_open())
		    for(int r = 0 ; r < this->distance_matrix.rows();r++){
		        for(int c = 0;c < this->distance_matrix.cols() ;c++)
		            file << setiosflags(ios::right) << setw(4) <<  (int)this->image.at<uchar>(r,c);
		        file << '\n';
        }
      else
          LOG4CXX_ERROR(this->logger,"ERROR - Failed to write image matrix to file: " << ex_filename );
      
      if (file)
          file.close();

    }

    void Algorithm_Distance_Map::save_distance_matrix_to_file(string ex_filename){
      std::ofstream file(ex_filename.c_str());
      if (file.is_open())
          file <<  this->distance_matrix << '\n';
      else
          LOG4CXX_ERROR(this->logger,"ERROR - Failed to write distance matrix to file: " << ex_filename );
      
      if (file)
          file.close();
    }

}
