#include "algorithm_dp_path_finder.hpp"

namespace prhlt {
    using namespace Eigen;
    using namespace log4cxx;
    using namespace log4cxx::helpers;

	Algorithm_DP_Path_Finder::Algorithm_DP_Path_Finder(cv::Mat ex_image_mat, const MatrixXd& ex_cost_matrix){
        this->logger = Logger::getLogger("PRHLT.Algorithm_DP_PF");
        LOG4CXX_INFO(this->logger,"<<INSTANTIATING DP PATH FINDER >>");
        cv::Mat tmp = ex_image_mat.clone();
        this->image.load_from_matrix(tmp);
        this->cost_matrix = ex_cost_matrix;
        this->roof_distance = this->cost_matrix.maxCoeff();
        this->path_matrix_x.setConstant(this->cost_matrix.rows(),this->cost_matrix.cols(),-1);
        this->path_matrix_y.setConstant(this->cost_matrix.rows(),this->cost_matrix.cols(),-1);
        this->limits_matrix.setConstant(this->cost_matrix.rows(),this->cost_matrix.cols(),1);
		LOG4CXX_INFO(this->logger,"Matrix size is : " << this->limits_matrix.rows() << " - " << this->limits_matrix.cols() );
        this->axis_x=0;
        this->axis_y=0;
        initialize_bound_matrix();
        LOG4CXX_INFO(this->logger,"<<Image size is : " << this->cost_matrix.rows() << " x " << this->cost_matrix.cols());
        LOG4CXX_INFO(this->logger,"<<Roof distance is : " << this->roof_distance);
        LOG4CXX_INFO(this->logger,"<<INSTANTIATING DP PATH FINDER - DONE >>");
    }

    Algorithm_DP_Path_Finder::Algorithm_DP_Path_Finder(cv::Mat ex_image_mat, const Eigen::MatrixXd& ex_cost_matrix, cv::Rect ex_search_area ){
      this->logger = Logger::getLogger("PRHLT.Algorithm_DP_PF");
      this->axis_x = ex_search_area.y;
      this->axis_y = ex_search_area.x;
      this->cost_matrix=ex_cost_matrix.block(ex_search_area.y,ex_search_area.x,ex_search_area.height,ex_search_area.width);
      this->roof_distance = this->cost_matrix.maxCoeff();
      LOG4CXX_INFO(this->logger,"<<Input matrix size is : " << ex_cost_matrix.rows() << " x " << ex_cost_matrix.cols());
      LOG4CXX_INFO(this->logger,"<<Resizing to          : " << ex_search_area.y << " - " << ex_search_area.x << " : " << ex_search_area.height   << " - " << ex_search_area.width);
      this->path_matrix_x.setConstant(this->cost_matrix.rows(),this->cost_matrix.cols(),-1);
      this->path_matrix_y.setConstant(this->cost_matrix.rows(),this->cost_matrix.cols(),-1);
      this->limits_matrix.setConstant(this->cost_matrix.rows(),this->cost_matrix.cols(),1);
      this->image.load_from_matrix_region(ex_image_mat,ex_search_area);
      //this->image.display_with_scale("SEARCH",1.0,50000, true);
      initialize_bound_matrix();
    }

    Algorithm_DP_Path_Finder::Algorithm_DP_Path_Finder(cv::Mat ex_image_mat, const MatrixXd& ex_cost_matrix,int orig_x, int orig_y , int size_x, int size_y){
        this->logger = Logger::getLogger("PRHLT.Algorithm_DP_PF");
        LOG4CXX_INFO(this->logger,"<<INSTANTIATING DP PATH FINDER >>");
        LOG4CXX_INFO(this->logger,"<<Input matrix size is : " << ex_cost_matrix.rows() << " x " << ex_cost_matrix.cols());
        LOG4CXX_INFO(this->logger,"<<Resizing to          : " << orig_x << " - " << orig_y << " : " << size_x   << " - " << size_y);
        //this->image.load_from_matrix(ex_image_mat);
        this->axis_x=orig_x;
        this->axis_y=orig_y;
        this->cost_matrix = ex_cost_matrix.block(orig_x,orig_y,size_x,size_y);
		LOG4CXX_INFO(this->logger,"<<Done copying>>");
        this->roof_distance = this->cost_matrix.maxCoeff();
        this->path_matrix_x.setConstant(this->cost_matrix.rows(),this->cost_matrix.cols(),-1);
        this->path_matrix_y.setConstant(this->cost_matrix.rows(),this->cost_matrix.cols(),-1);
        this->limits_matrix.setConstant(this->cost_matrix.rows(),this->cost_matrix.cols(),1);
		LOG4CXX_INFO(this->logger,"Matrix size is : " << this->limits_matrix.rows() << " - " << this->limits_matrix.cols() );
        //this->image.load_from_matrix(ex_image_mat);
        this->image.load_from_matrix_region(ex_image_mat,cv::Rect(orig_y,orig_x,size_y,size_x));
        //this->image.display_with_scale("SEARCH",0.7,5000, true);
        //this->axis_x=0;
        //this->axis_y=0;
        initialize_bound_matrix();
        LOG4CXX_INFO(this->logger,"<<Image size is : " << this->cost_matrix.rows() << " x " << this->cost_matrix.cols());
        LOG4CXX_INFO(this->logger,"<<Roof distance is : " << this->roof_distance);
        LOG4CXX_INFO(this->logger,"<<INSTANTIATING DP PATH FINDER - DONE >>");
    }
	void Algorithm_DP_Path_Finder::set_search_limits(vector < vector <cv::Point> > ex_upper_limits, vector < vector <cv::Point> > ex_lower_limits){
		this->upper_search_limits=ex_upper_limits;
		this->lower_search_limits=ex_lower_limits;
    LOG4CXX_INFO(this->logger,"<<Setting all search limits >>");
//    show_search_area();
		calculate_valid_search_area();
	}
  void Algorithm_DP_Path_Finder::set_search_limits(vector <cv::Point> ex_upper_limits, vector <cv::Point> ex_lower_limits){
    this->upper_search_limits.push_back(ex_upper_limits);
    this->lower_search_limits.push_back(ex_lower_limits);
    LOG4CXX_INFO(this->logger,"<<Setting all search limits >>");
    //show_search_area();
    calculate_valid_search_area();
  }

	void Algorithm_DP_Path_Finder::set_lower_search_limits(vector < vector <cv::Point> > ex_lower_limits){
		this->lower_search_limits=ex_lower_limits;
    LOG4CXX_INFO(this->logger,"<<Setting lower search limits >>");
//    show_search_area();
		calculate_valid_search_area();
	}
  void Algorithm_DP_Path_Finder::set_lower_search_limits(vector <cv::Point> ex_lower_limits){
    this->lower_search_limits.push_back(ex_lower_limits);
        LOG4CXX_INFO(this->logger,"<<Set lower search limits >>");
//    show_search_area();
    calculate_valid_search_area();
  }

  void Algorithm_DP_Path_Finder::set_upper_search_limits(vector < vector <cv::Point> > ex_upper_limits){
    this->upper_search_limits=ex_upper_limits;
        LOG4CXX_INFO(this->logger,"<<Set upper search limits >>");
//    show_search_area();
    calculate_valid_search_area();
  }
  void Algorithm_DP_Path_Finder::set_upper_search_limits(vector <cv::Point> ex_upper_limits){
    this->upper_search_limits.push_back(ex_upper_limits);
        LOG4CXX_INFO(this->logger,"<<Set upper search limits >>");
//    show_search_area();
    calculate_valid_search_area();
  }
	void Algorithm_DP_Path_Finder::calculate_valid_search_area(){
        LOG4CXX_INFO(this->logger,"<<Calculating search area >>");
        for (int c = 0; c < this->limits_matrix.cols(); c++){
            for(int r = 0; r < this->limits_matrix.rows();r++){
        		if(!is_valid_point_as_per_search_limits(r,c)){
        			this->limits_matrix(r,c)=0;
        		}
            }
        }
        LOG4CXX_INFO(this->logger,"<<Done >>");


	}
	void Algorithm_DP_Path_Finder::show_search_area(){
        vector<cv::Point2d> points;

        for (int c = 0; c < this->cost_matrix.cols(); c++){
            for(int r = 0; r < this->cost_matrix.rows();r++){
        		if(!is_valid_point_as_per_search_limits(r,c)){
        			points.push_back(cv::Point2d(r,c));
        		}
            }
        }
        LOG4CXX_INFO(this->logger,"Area   total : " << this->cost_matrix.cols() * this->cost_matrix.rows());
        LOG4CXX_INFO(this->logger,"Points total : " << points.size());
        int i;
        
        this->image.draw_randomed_colored_region(points);
        //this->image.draw_line(cv::Point2d(this->lower_search_limits[0][0].x-this->axis_y,this->lower_search_limits[0][0].y-this->axis_x),cv::Point2d(this->lower_search_limits[0][1].x-this->axis_y,this->lower_search_limits[0][1].y-this->axis_x),cv::Scalar(255,0,0));
		points.clear();
    LOG4CXX_INFO(this->logger, "DISPLAY");
    this->image.display_with_scale("SEARCH", 2.0,-1, true);
    //cin >> i;
	}

    Algorithm_DP_Path_Finder::~Algorithm_DP_Path_Finder(){
    }

    void Algorithm_DP_Path_Finder::initialize_bound_matrix(){
        this->bound_matrix.setConstant(this->cost_matrix.rows(),this->cost_matrix.cols(),-1);
        for(int r = 0; r < this->bound_matrix.rows();r++){
            this->bound_matrix(r,0)=0;
        }
    }

    void Algorithm_DP_Path_Finder::run(double ex_alpha , double ex_beta){
        this->alpha = ex_alpha;
        this->beta = ex_beta;
        this->average_euclidian_distance_cost=0;
        this->average_grey_distance_cost=0;
        LOG4CXX_INFO(this->logger,"RUNNING");

        do{
        	reset_change_counter();
        	forward();
        	backward();
		}while(solution_not_converged());

        LOG4CXX_INFO(this->logger,"ENDED");
        this->average_euclidian_distance_cost /= (double)(this->cost_matrix.cols()*this->cost_matrix.rows());
        this->average_grey_distance_cost /= (double)(this->cost_matrix.cols()*this->cost_matrix.rows());
        LOG4CXX_DEBUG(this->logger,"Average : Grey cost : "<< this->average_grey_distance_cost    << " Euclidean: " << this->average_euclidian_distance_cost);
		   // this->image.display_with_scale("PATH FINDER", 0.5, 10000000,false);
       //save_best_path();
       //display_best_path();
    }
    void Algorithm_DP_Path_Finder::reset_change_counter(){
    	this->cells_changed = 0;
	}
    bool Algorithm_DP_Path_Finder::solution_not_converged(){
    	if( this->cells_changed !=0)
    		return true;
		else
			return false;
	}

    void Algorithm_DP_Path_Finder::forward(){
        LOG4CXX_INFO(this->logger,"FORWARD");
        for (int c = 1; c < this->cost_matrix.cols(); c++){
            update_column(c);
            review_column(c);
        }
    }
    void Algorithm_DP_Path_Finder::backward(){
        LOG4CXX_INFO(this->logger,"BACKWARD");
        for (int c = this->cost_matrix.cols()-2; c >= 0; c--){
            backtrack_column(c);
            review_column(c);
        }
    }
    void Algorithm_DP_Path_Finder::update_column(int c){
      LOG4CXX_DEBUG(this->logger,"Updating column: " << c);
      for(int r = 0; r < this->cost_matrix.rows();r++)
      {
        update_cell(r,c);
      }
	}

    void Algorithm_DP_Path_Finder::update_cell(int x , int y){
	   LOG4CXX_DEBUG(this->logger,"UPDATING : " << x << " - " << y );
        if(y-1 >= 0){
            if(x+1 < this->cost_matrix.rows())
                update_cell_from(x+1,y-1,x,y);
            update_cell_from(x,y-1,x,y);
            if(x-1 >=0 )
                update_cell_from(x-1,y-1,x,y);
        }
		}
    void Algorithm_DP_Path_Finder::review_column(int c){
        bool updated = true;
        	//LOG4CXX_INFO(logger, "Reviewing column: " << c   );
        while (updated){
            updated = false;
            for(int r = 0; r < this->cost_matrix.rows();r++){
                updated |= review_cell(r,c) ? 1 : 0;
            }
            //LOG4CXX_INFO(this->logger,"Reviewing " << updated);
        }
    }
    bool Algorithm_DP_Path_Finder::review_cell(int x , int y){
       bool updated = false;
        if(x-1 >=0)
            updated |= update_cell_from(x-1,y,x,y);
        if(x+1 < this->cost_matrix.rows())
            updated |= update_cell_from(x+1,y,x,y);

        return updated;
    }

    void Algorithm_DP_Path_Finder::backtrack_column(int c){
    	for(int r = 0; r < this->cost_matrix.rows();r++){
    		backtrack_cell(r,c);
    	}
    }

    bool Algorithm_DP_Path_Finder::backtrack_cell(int x , int y){
       bool updated = false;
        updated |= update_cell_from(x,y+1,x,y);
        if(x-1 >=0)
            updated |= update_cell_from(x-1,y+1,x,y);
        if(x+1 < this->cost_matrix.rows())
            updated |= update_cell_from(x+1,y+1,x,y);

        return updated;
    }
		bool Algorithm_DP_Path_Finder::update_cell_from(int from_x, int from_y, int to_x , int to_y){
        //if(this->bound_matrix(from_x, from_y) !=-1){
        LOG4CXX_DEBUG(this->logger,"UPDATING FROM : " << from_x << " - " << from_y  << " TO " << to_x << " - " << to_y);
        //LOG4CXX_INFO(this->logger,"Bound matrix size is : " << this->bound_matrix.rows() << " - " << this->bound_matrix.cols() );
        if((this->bound_matrix(from_x, from_y) !=-1) && (precalc_is_valid_point_as_per_search_limits(to_x,to_y))){
        //double euclidian_factor = contextual_average(to_x,to_y,5);
        double euclidian_factor = 1;
        //LOG4CXX_INFO(this->logger,"BASE MOVEMENTE COST");
        double euclidian_cost = euclidian_factor*base_movement_cost(from_x,from_y,to_x,to_y);
        //double factor = (double)(1.0/(double)sqrt(contextual_average(to_x,to_y,5)+1.0));
        //double grey_factor = 1/future_contextual_average(to_x,to_y,11,30);
        //LOG4CXX_INFO(this->logger,"CONTEXTUAL AVG");
        double grey_factor = (this->roof_distance-contextual_average(to_x,to_y,10))/(this->roof_distance);
        //LOG4CXX_INFO(this->logger,"COST MATRIX");
        double grey_cost = this->cost_matrix(to_x,to_y) == 0 ? this->roof_distance*40 : this->roof_distance-this->cost_matrix(to_x,to_y);
        //double grey_cost = (this->roof_distance-this->cost_matrix(to_x,to_y))/this->roof_distance;

        double grey_distance_cost = grey_factor * grey_cost;
        //double grey_avoidance = pow(this->roof_distance - this->contextual_average(to_x,to_y,5),2);
        double grey_avoidance = 0;
        this->average_euclidian_distance_cost+=euclidian_cost;
        this->average_grey_distance_cost+=grey_distance_cost;
        double cost = this->bound_matrix(from_x,from_y)+(this->alpha*grey_distance_cost)+(this->beta*euclidian_cost)+grey_avoidance;
        //current_path.print_info();
        //tmp.print_info(;
        if(update_bound_matrix(to_x, to_y,cost)){
        	this->cells_changed++;
            update_path_matrix(to_x, to_y,from_x,from_y);
            return true;
        }
        else
            return false;
        }
        return false;
    }

    bool Algorithm_DP_Path_Finder::precalc_is_valid_point_as_per_search_limits(int r, int c){
        //LOG4CXX_INFO(this->logger,"Got asked about : " << r << " - " << c );
		    //LOG4CXX_INFO(this->logger,"Matrix size is : " << this->limits_matrix.rows() << " - " << this->limits_matrix.cols() );
    	return (bool)this->limits_matrix(r,c);
	}

    bool Algorithm_DP_Path_Finder::is_valid_point_as_per_search_limits(int x, int y){
    	x+=this->axis_x;
    	y+=this->axis_y;
    	for(int i = 0; i < this->upper_search_limits.size();i++)
      {
        if(restriction_segment_applicable_to_point(this->upper_search_limits[i],x,y) && point_position_in_respect_to_line(this->upper_search_limits[i],this->axis_x+this->cost_matrix.rows(),y)>0)
        {
          if(point_position_in_respect_to_line(this->upper_search_limits[i],x,y)<=0)
            return false;
        }
      }
    	for(int i = 0; i < this->lower_search_limits.size();i++)
      {
    		//Check if the limit lets us pass at any point, otherwise discard the limit
    		if(restriction_segment_applicable_to_point(this->lower_search_limits[i],x,y) && point_position_in_respect_to_line(this->lower_search_limits[i],this->axis_x,y)<0)
        {
          if(point_position_in_respect_to_line(this->lower_search_limits[i],x,y)>=0)
            return false;
        }
		}
		return true;
	}
  bool Algorithm_DP_Path_Finder::restriction_segment_applicable_to_point(vector<cv:: Point> ex_line, int x, int y){
    if ( ex_line[0].x < ex_line[1].x){
      if(ex_line[0].x <= y && y <= ex_line[1].x)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      if(ex_line[1].x <= y && y <= ex_line[0].x)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
  }

    int Algorithm_DP_Path_Finder::point_position_in_respect_to_line(vector<cv:: Point> ex_line, int ex_x, int ex_y){
     //  	LOG4CXX_INFO(logger, "Point is : " << ex_x << " " << ex_y );
     //  	LOG4CXX_INFO(logger, "Line limit 0 : " << ex_line[0].x << " " << ex_line[0].y );
     //  	LOG4CXX_INFO(logger, "Line limit 1 : " << ex_line[1].x << " " << ex_line[1].y );
       	//int res = ((ex_line[1].x - ex_line[0].x)*(ex_y - ex_line[0].y) - (ex_line[1].y - ex_line[0].y)*(ex_x - ex_line[0].x));
       	//int res2 = ((ex_line[1].y - ex_line[0].y)*(ex_y - ex_line[0].x) - (ex_line[1].x - ex_line[0].x)*(ex_x - ex_line[0].y));
       	//int res3 = ((ex_line[1].y - ex_line[0].y)*(ex_x - ex_line[0].x) - (ex_line[1].x - ex_line[0].x)*(ex_y - ex_line[0].y));
       	int res = 0;

       	if ( ex_line[0].x < ex_line[1].x)
        {
          res = (ex_line[1].x - ex_line[0].x)*(ex_x - ex_line[0].y) - ((ex_line[1].y - ex_line[0].y)*(ex_y - ex_line[0].x));
        }
        else
        {
     //  		LOG4CXX_INFO(logger, "SECOND" );
       		res = (ex_line[0].x - ex_line[1].x)*(ex_x - ex_line[1].y) - ((ex_line[0].y - ex_line[1].y)*(ex_y - ex_line[1].x));
        }
       //	LOG4CXX_INFO(logger, "Result : " << res );
       	//LOG4CXX_INFO(logger, "Result 2 : " << res2 );
       	//LOG4CXX_INFO(logger, "Result 3 : " << res3 );
       	//LOG4CXX_INFO(logger, "Result 4 : " << res4 );
       //	cin >> res;
       	return res;

	}

    double Algorithm_DP_Path_Finder::base_movement_cost(int from_x, int from_y, int to_x, int to_y){
        if ((from_x != to_x) and (from_y != to_y))
            return 1.4142135;
        else
            return 1.0;
    }
    double Algorithm_DP_Path_Finder::contextual_average(int x, int y,const int context_size){

        int half_context = context_size/2;

        int ini_x =  x-half_context >= 0 ? x-half_context : 0 ;
        int end_x =  x+half_context < this->cost_matrix.rows() ? x + half_context : this->cost_matrix.rows()-1;

        int ini_y =  y-half_context >=0 ? y-half_context : 0 ;
        int end_y =  y+half_context < this->cost_matrix.cols() ? y + half_context : this->cost_matrix.cols()-1;

        int x_context = (x - ini_x) + (end_x -x);
        int y_context = (y - ini_y) + (end_y -y) ;
        return this->cost_matrix.block(ini_x,ini_y,x_context,y_context).mean();
    }

    double Algorithm_DP_Path_Finder::future_contextual_average(int x, int y,const int x_context_size, const int y_context_size){

        int half_context = x_context_size/2;

        int ini_x =  x-half_context >= 0 ? x-half_context : 0 ;
        int end_x =  x+half_context < this->cost_matrix.rows() ? x + half_context : this->cost_matrix.rows()-1;

        int ini_y =  y ;
        int end_y =  y+y_context_size < this->cost_matrix.cols() ? y + y_context_size : this->cost_matrix.cols()-1;

        int x_context = (x - ini_x) + (end_x -x);
        int y_context = 0;
        if (ini_y == end_y)
            y_context = 1;
        else
            y_context=(y - ini_y) + (end_y -y);
        return this->cost_matrix.block(ini_x,ini_y,x_context,y_context).mean();

    }

    bool Algorithm_DP_Path_Finder::update_bound_matrix(int x, int y, double cost){
        LOG4CXX_DEBUG(this->logger,"Cell : " << x << " - " << y << " CC: " << this->bound_matrix(x,y) << " UC: " << cost);
        if((this->bound_matrix(x,y) == -1) or (this->bound_matrix(x,y)>cost)){
            LOG4CXX_DEBUG(this->logger,"UPDATED" );
            this->bound_matrix(x,y)=cost;
            return true;
        }
        else
            return false;
    }
    void Algorithm_DP_Path_Finder::update_path_matrix(int x, int y,int from_x, int from_y){
        LOG4CXX_DEBUG(this->logger,"Cell : " << x << " - " << y << " <- " << from_x  << " - " << from_y);
        this->path_matrix_x(x,y)=from_x;
        this->path_matrix_y(x,y)=from_y;
    }

    vector<cv::Point2d> Algorithm_DP_Path_Finder::recover_path(int x, int y){
    	vector<cv::Point2d> tmp;
    	cv::Point2d current_point(x,y);
    	cv::Point2d new_point(x,y);
    	LOG4CXX_INFO(this->logger,"RECOVERING PATH" );
        tmp.push_back(cv::Point2d(current_point.x+this->axis_x,current_point.y+this->axis_y));
        //save_path_matrix_to_file("matrix.txt");
        while(current_point.y != 0){
            LOG4CXX_DEBUG(this->logger,"Cycle");
            LOG4CXX_DEBUG(this->logger," At  : " << current_point.x << " - " << current_point.y );
            LOG4CXX_DEBUG(this->logger," At  : " << current_point.x << " - " << current_point.y  << " ->  " << this->path_matrix_x(current_point.x,current_point.y)  << " - " << this->path_matrix_y(current_point.x,current_point.y) );
            LOG4CXX_DEBUG(this->logger,"Updating current point");
            new_point.x=this->path_matrix_x(current_point.x,current_point.y);
            new_point.y=this->path_matrix_y(current_point.x,current_point.y);
            current_point=new_point;
            LOG4CXX_DEBUG(this->logger,"Adding it to the list");
            if(tmp[tmp.size()-1].y != current_point.y)
                tmp.push_back(cv::Point2d(current_point.x+this->axis_x,current_point.y+this->axis_y));
                LOG4CXX_DEBUG(this->logger," Adding  : " << current_point.x+this->axis_x << " - " << current_point.y+this->axis_y);
                //tmp.push_back(cv::Point2d(current_point.y,current_point.x));
                //tmp.push_back(cv::Point2d(current_point.y+this->axis_y,current_point.x+this->axis_x));
            LOG4CXX_DEBUG(this->logger,"Added");
           //display_path(this->best_path);
        }
		    return tmp;
	}
	vector<cv::Point2d> Algorithm_DP_Path_Finder::recover_best_path(){
		LOG4CXX_DEBUG(this->logger,"RECOVERING BEST PATH" );
		double min_cost = this->bound_matrix(0,this->bound_matrix.cols()-1);
		int min_index = 0;
        LOG4CXX_DEBUG(this->logger,"In row : " << min_index << " cost is : " << min_cost );

        for (int r = 1; r < this->bound_matrix.rows();r++){
        	LOG4CXX_DEBUG(this->logger,"In row : " << r << " cost is : " << this->bound_matrix(r,this->bound_matrix.cols()-1));
		        if((min_cost < 0 ) || ((this->bound_matrix(r,this->bound_matrix.cols()-1) < min_cost) && (this->bound_matrix(r,this->bound_matrix.cols()-1) > 0))){
		            min_index = r;
		            min_cost = this->bound_matrix(r,this->bound_matrix.cols()-1);
            }
        }
        if(min_cost< 0){
          LOG4CXX_ERROR(this->logger,"NO PATH WAS FOUND DUE TO RESTRICTIONS" );
          vector<cv::Point2d> res; 
          //show_search_area();
          return res; 
        }
        //int i;
        //cin >> i;
        LOG4CXX_DEBUG(this->logger,"Choose row : " << min_index << " cost is : " << min_cost );
        return recover_path(min_index,this->bound_matrix.cols()-1);
    }

    vector< vector<cv::Point2d> > Algorithm_DP_Path_Finder::recover_all_paths(){
        LOG4CXX_INFO(this->logger,"RECOVERING ALL PATHS" );
        vector< vector<cv::Point2d> > all_paths;
            for (int r = 0; r < this->path_matrix_x.rows();r++)
                all_paths.push_back(recover_path(r,this->path_matrix_y.cols()-1));
        return all_paths;
    }

    void Algorithm_DP_Path_Finder::display_best_path(){
    	vector<cv::Point2d> tmp = recover_best_path();
      LOG4CXX_INFO(this->logger, "SIZE " << tmp.size());
      //this->image.draw_randomed_colored_region(tmp);
      this->image.draw_randomed_colored_polyline(tmp);
    	this->image.display_with_scale("PATH FINDER", 2.0, 10000000,false);
    	tmp.clear();
    }
    void Algorithm_DP_Path_Finder::save_best_path(){
    	vector<cv::Point2d> tmp = recover_best_path();
    	//this->image.draw_randomed_colored_region(tmp);
      this->image.draw_polyline(tmp,cv::Scalar(0,255,0));
    	this->image.save_image("best_path.png");
    	tmp.clear();
    }


    vector<cv::Point2d> Algorithm_DP_Path_Finder::get_best_path_collision_points(){
    	vector<cv::Point2d> best_path = recover_best_path();
    	vector<cv::Point2d> collision_points;
    	for(int i = 1; i < best_path.size();i++){
    		if(is_collision_point(best_path,i))
    			collision_points.push_back(best_path[i]);
    	}
    	return collision_points;
    }

    bool Algorithm_DP_Path_Finder::is_collision_point(vector<cv::Point2d> points, int index ){
    	bool center=false,left=false,right=false;
		cv::Point2d tmp_point = localize_point(points[index]);
        //LOG4CXX_INFO(logger, "In point: " << index << " : " << tmp_point.x << " - " << tmp_point.y  );
    	if(this->cost_matrix(tmp_point.x,tmp_point.y) == 0)
    		center = true;
        //LOG4CXX_INFO(logger, "CHECKED");

    	if(center == true){
    		if((tmp_point.x ==0)||(this->cost_matrix(tmp_point.x-1,tmp_point.y) == 0))
    			left = true;
 		   	if((tmp_point.x == this->cost_matrix.rows()-1)||(this->cost_matrix(tmp_point.x+1,tmp_point.y) == 0))
    			right = true;

    		if(index > 0){
    			tmp_point = localize_point(points[index-1]);
    			//LOG4CXX_INFO(logger, "Minus In point: " << tmp_point.x << " - " << tmp_point.y  );
    			if( (tmp_point.x ==0) || (this->cost_matrix(tmp_point.x-1,tmp_point.y) == 0) )
    				left = true;
    			if((tmp_point.x == this->cost_matrix.rows()-1)||(this->cost_matrix(tmp_point.x+1,tmp_point.y) == 0))
    				right = true;
    		}
    		if(index < points.size()-1){
    			tmp_point = localize_point(points[index+1]);
    			//LOG4CXX_INFO(logger, "Plus In point: " << tmp_point.x << " - " << tmp_point.y  );
    			if((tmp_point.x ==0)||(this->cost_matrix(tmp_point.x-1,tmp_point.y) == 0))
    				left = true;
    			if((tmp_point.x == this->cost_matrix.rows()-1)||(this->cost_matrix(tmp_point.x+1,tmp_point.y) == 0))
    				right = true;
    		}
		}
    	return (center && left && right);
	}

	cv::Point2d Algorithm_DP_Path_Finder::localize_point(cv::Point2d point){
		cv::Point2d tmp(point.x-this->axis_x,point.y-this->axis_y);
		return tmp;
	}
	cv::Point2d Algorithm_DP_Path_Finder::localize_point(int x , int y){
		cv::Point2d tmp(x-this->axis_x,y-this->axis_y);
		return tmp;
	}

    void Algorithm_DP_Path_Finder::display_path(int x, int y){
    	vector<cv::Point2d> tmp = recover_path(x,y);
    	//this->image.draw_randomed_colored_region(tmp);
      this->image.draw_randomed_colored_polyline(tmp);
    	this->image.display_with_scale("PATH FINDER", 0.5, 10000000,false);
    	tmp.clear();
    }
    void Algorithm_DP_Path_Finder::save_path_matrix_to_file(string file_name){
    	std::ofstream file(file_name.c_str());
    	if (file.is_open())
    	{
    		for (int r = 0; r < this->bound_matrix.rows();r++){
              for (int c = 0; c < this->bound_matrix.cols();c++){
                  file << "[" << this->path_matrix_x(r,c) << "," << this->path_matrix_y(r,c) << "] ";
              }
              file << "\n";
            }
        }
        else
        	LOG4CXX_ERROR(this->logger,"ERROR - Failed to write file: " << file_name );

	  	if (file)
	  		file.close();
	}
}
