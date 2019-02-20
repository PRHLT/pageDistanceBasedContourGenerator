#include "algorithm_otsu.hpp"

namespace prhlt{
    using namespace log4cxx;
    using namespace log4cxx::helpers;
    using namespace boost;
    
    Algorithm_OTSU::Algorithm_OTSU(cv::Mat &ex_image){
        this->logger = Logger::getLogger("PRHLT.Algorithm_Otsu");
        this->threshold = 127;
        this->grey_level_histogram.set_image(ex_image);
    }
    
    int Algorithm_OTSU::run(){
        return calculate_otsu_threshold();
    }

    int Algorithm_OTSU::calculate_otsu_threshold(){
        this->threshold = -1;
        float csum = 0.0, sbmax=0.0, percentageCalculated=0.0,sb=0.0,m1=0.0,m2=0.0;
        float percentagePending= 1.0 - percentageCalculated;
        float sum = this->grey_level_histogram.run();
        
        BOOST_FOREACH(sparse_histogram::value_type i, this->grey_level_histogram.histogram){
            LOG4CXX_DEBUG(logger,"[" << i.first << "] -> " << i.second );
            percentageCalculated+=i.second;
            percentagePending= 1.0 - percentageCalculated;
            csum += float(i.first)*i.second;
            m1 = csum/percentageCalculated;
            m2 = (sum-csum)/percentagePending;
            sb = (percentageCalculated * percentagePending * ((m1-m2)*(m1-m2)));
            if(sb > sbmax){
                sbmax=sb;
                this->threshold = i.first;
            }

        }
        if(this->threshold == 255)
        	this->threshold = 254;
        LOG4CXX_INFO(this->logger,"Otsu calculated threshold:" << threshold );
        return this->threshold;
    }
}
