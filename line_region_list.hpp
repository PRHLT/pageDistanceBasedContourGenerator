#ifndef LINE_REGION_LIST_HPP_4D3UIS3SDF
#define LINE_REGION_LIST_HPP_4D3UIS3SDF

#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/helpers/exception.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/unordered_map.hpp>
#include <boost/multi_array.hpp>
#include <fstream>
#include <iostream>


//namespace std {using namespace __gnu_cxx;}
using namespace std;

namespace prhlt {
    class Line_Region_List{
        public:
            Line_Region_List(string ex_file_name, int ex_page_start, int ex_page_end);
            ~Line_Region_List();
            vector< vector <int> >  get_line_limits();
            vector< vector <int> >  get_search_zones();
        private:
            vector< vector <int> >  line_limits;
            vector< vector <int> >  search_zones;
            log4cxx::LoggerPtr logger;
            float average_line_size;
            float average_interline_size; 
            int page_start;
            int page_end;
            string file_name;
            void clear_limits_list();
            void clear_search_list();
            void load_file();
            void calculate_search_zones();
    };
}

#endif /* end of include guard*/
