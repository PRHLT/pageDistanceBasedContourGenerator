#include "algorithm_sauvola.hpp"

namespace prhlt{
    using namespace log4cxx;
    using namespace log4cxx::helpers;
    using namespace boost;
    
    Algorithm_SAUVOLA::Algorithm_SAUVOLA(cv::Mat &ex_image){
        this->logger = Logger::getLogger("PRHLT.Algorithm_SAUVOLA");
        if(ex_image.channels()>1){
            this->image = cv::Mat(ex_image.rows,ex_image.cols,CV_8UC1);
            cvtColor(ex_image, this->image, CV_RGB2GRAY);
        }
        else
            this->image= ex_image.clone();
        
        this->binarized_image=this->image.clone();
        initialize_integral_images();
    }
    
    void Algorithm_SAUVOLA::initialize_integral_images(){
       
        LOG4CXX_INFO(this->logger,"<<SAUVOLA: initializing integral images>>");
		   
		    this->integral_image = cv::Mat(this->image.rows,this->image.cols,CV_32SC1,cv::Scalar(0,0,0));
		    this->integral_image_squared = cv::Mat(this->image.rows,this->image.cols,CV_64FC1,cv::Scalar(0,0,0));
		    cv::integral(this->image, this->integral_image,this->integral_image_squared);
		}
    Algorithm_SAUVOLA::~Algorithm_SAUVOLA(){
        this->image.release();
        this->binarized_image.release();
        this->integral_image.release();
        this->integral_image_squared.release();
    }
    cv::Mat Algorithm_SAUVOLA::run(float ex_standard_deviation_constant, float ex_dynamic_range_constant,int ex_window_height, int ex_window_width){
        this->standard_deviation_constant = ex_standard_deviation_constant;
        this->dynamic_range_constant = ex_dynamic_range_constant;
        this->window_height=ex_window_height;
        this->window_width=ex_window_width;
        
        LOG4CXX_INFO(this->logger,"<<SAUVOLA: performing binarization!>>");
        
        for(int r = 0 ; r < this->image.rows;r++)
            for(int c = 0;c < this->image.cols;c++)
                update_pixel(r,c);
        LOG4CXX_INFO(this->logger,"<<SAUVOLA: DONE!>>");
        
        return this->binarized_image;
    }
    
    void Algorithm_SAUVOLA::update_pixel(int x, int y){
        
        int from_x = x + (this->window_height-1)/2 >= this->image.rows? this->image.rows-1 : x+(this->window_height-1)/2;
        int from_y = y + (this->window_width-1)/2 >= this->image.cols ? this->image.cols-1 : y+(this->window_width-1)/2;
        double mean = average_grey_of_region(from_x,from_y);
        double std_dev = standard_deviation_grey_of_region(mean,from_x,from_y);
        
        double threshold = mean*(1+this->standard_deviation_constant * (std_dev/this->dynamic_range_constant - 1 ));
        //LOG4CXX_DEBUG(this->logger,"Updating [" << x << "," << y << "]:" << (int)mean << " " << (int)std_dev << " -> " << (int)threshold << " ? " << (int)this->image.at<uchar>(x,y));

        if(((int)this->image.at<uchar>(x,y)) < threshold)
            this->binarized_image.at<uchar>(x,y) = 0;
        else
            this->binarized_image.at<uchar>(x,y) = 255;
    }
    
    double Algorithm_SAUVOLA::average_grey_of_region(int from_x, int from_y){
        //LOG4CXX_DEBUG(this->logger,"Average for start point" << from_x << "-" << from_y << " : " << this->integral_image.at<int>(from_x,from_y));
        double res = this->integral_image.at<int>(from_x,from_y);
        int region_cuts = 0;
        if (from_y - this->window_width > 0){
            res -= this->integral_image.at<int>(from_x,from_y - (this->window_width+1));
            //LOG4CXX_DEBUG(this->logger,"Substracting " << from_x << "-" << (from_y - (this->window_width+1)) << " : " << (int)res);
            region_cuts++;
        }
        if (from_x - this->window_height > 0){
            res -= this->integral_image.at<int>(from_x-(this->window_height+1),from_y);
            //LOG4CXX_DEBUG(this->logger,"Substracting " << (from_x - (this->window_height+1)) << "-" << from_y << " : " <<(int) res);
            region_cuts++;
        }
        if (region_cuts == 2){
            res += this->integral_image.at<int>(from_x-(this->window_height+1),from_y-(this->window_width+1));
            //LOG4CXX_DEBUG(this->logger,"Adding " << (from_x - (this->window_height+1)) << "-" << (from_y - (this->window_width+1)) << " : " << (int)res );
        }
        
        return res/(double)(this->window_height * this->window_width);
    }
    double Algorithm_SAUVOLA::standard_deviation_grey_of_region(double mean ,int from_x, int from_y){
        
        //LOG4CXX_DEBUG(this->logger,"std_dev for start point" << from_x << "-" << from_y << " : " << this->integral_image_squared.at<double>(from_x,from_y));
        double res = this->integral_image_squared.at<double>(from_x,from_y);
        int region_cuts = 0;
        
        if (from_y - this->window_width > 0){
            res -= this->integral_image_squared.at<double>(from_x,from_y-(this->window_width+1));
            //LOG4CXX_DEBUG(this->logger,"Substracting " << from_x << "-" << (from_y - (this->window_width+1)) << " : " << (int)res);
            region_cuts++;
        }
        if (from_x - this->window_height > 0){
            res -= this->integral_image_squared.at<double>(from_x-(this->window_height+1),from_y);
            //LOG4CXX_DEBUG(this->logger,"Substracting " << (from_x - (this->window_height+1)) << "-" << from_y << " : " <<(int) res);
            region_cuts++;
        }
        if (region_cuts == 2){
            res += this->integral_image_squared.at<double>(from_x-(this->window_height+1),from_y-(this->window_width+1));
            //LOG4CXX_DEBUG(this->logger,"Adding " << (from_x - (this->window_height+1)) << "-" << (from_y - (this->window_width+1)) << " : " << (int)res );
        }
        
        //LOG4CXX_DEBUG(this->logger,"E[x^2]= " << (res/(double)(this->window_height*window_width)) << " E[x]^2= " << (int)pow(mean,2) );
        return sqrt(abs((res/(float)(this->window_height* this->window_width)) - pow(mean,2)));
    }

}
