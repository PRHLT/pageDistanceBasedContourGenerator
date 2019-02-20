#include "algorithm_rlsa.hpp"


namespace prhlt {
    
    using namespace log4cxx;
    using namespace log4cxx::helpers;
    using namespace boost;
    
    Algorithm_RLSA::Algorithm_RLSA(cv::Mat &ex_image){
        this->logger = Logger::getLogger("PRHLT.Algorithm_RLSA");
        LOG4CXX_INFO(this->logger,"<<COPYING IMAGE>>");
        /*float scale = 0.3;
        cv::Mat resized(ex_image.rows*scale,ex_image.cols*scale, CV_8UC3);
        cv::resize(ex_image,resized,resized.size(), 0, 0);
        cv::namedWindow("1test_connected_components", CV_WINDOW_AUTOSIZE);
        cv::imshow("1test_connected_components",resized);
        cv::waitKey(-1);*/
        if(ex_image.channels()>1){
            this->image = cv::Mat(ex_image.rows,ex_image.cols,CV_8U,cv::Scalar(0));
            cvtColor(ex_image, this->image, CV_RGB2GRAY);
        }
        else
            this->image= ex_image.clone();
        LOG4CXX_INFO(this->logger,"<<COPYING IMAGE DONE>>");
    }
    Algorithm_RLSA::~Algorithm_RLSA(){
        this->image.release();
    }
    cv::Mat Algorithm_RLSA::run(const int vertical_mean_letter_length, const int horizontal_mean_letter_length, const int threshold){
        LOG4CXX_INFO(this->logger,"<<RUNNING>>");
        this->threshold = threshold;
        if (horizontal_mean_letter_length > 0)
            perform_horizontal_rlsa(horizontal_mean_letter_length);
        if (vertical_mean_letter_length > 0)
            perform_vertical_rlsa(vertical_mean_letter_length);
        return this->image;
    }
    
    void Algorithm_RLSA::perform_vertical_rlsa(const int meanLetterLength){
        int cont=0;
        int firstTime=1;
        LOG4CXX_DEBUG(this->logger,"<<PERFORMING VERTICAL RLSA>>");
        for (int c=0;c<this->image.cols;c++){
            cont=0;
            firstTime=1;
            for (int r=0;r<this->image.rows;r++)
                if (this->image.at<uchar>(r,c)<=this->threshold){
                    if(firstTime)
                        firstTime=0;
                    else{
                        if (cont < meanLetterLength)
                            for (int r2=r-cont; r2 <= r; r2++)
                                this->image.at<uchar>(r2,c)=0;
                    }
                    cont=0;
                } 
                else
                    cont++;
        }
    }
    
    void Algorithm_RLSA::perform_horizontal_rlsa(const int meanLetterLength){
        int cont=0;
        int firstTime=1;
        LOG4CXX_DEBUG(this->logger,"<<PERFORMING HORIZONTAL RLSA>>");
        for (int row=0;row<this->image.rows;row++){
            cont=0;
            firstTime=1;
            for (int col=0;col<this->image.cols;col++)
                if (this->image.at<uchar>(row,col)<=this->threshold){
                    if(firstTime)
                        firstTime=0;
                    else{
                        if (cont < meanLetterLength)
                            for (int c2=col-cont; c2 <= col; c2++)
                                this->image.at<uchar>(row,c2)=0;
                    }
                    cont=0;
                } 
                else
                    cont++;
        }
    }
}
