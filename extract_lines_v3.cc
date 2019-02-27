#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/helpers/exception.h>
#include <boost/thread/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include "image.hpp"
#include "algorithm_otsu.hpp"
#include "algorithm_sauvola.hpp"
#include "polyline_extractor.hpp"
#include "points_file.hpp"
#include "algorithm_extract_polygons.hpp"
#include "page_file.hpp"

namespace po = boost::program_options;
using namespace std;
using namespace boost::filesystem;
using namespace log4cxx;
using namespace log4cxx::helpers;

LoggerPtr logger(Logger::getLogger("PRHLT"));

void setVerbosity(int verb){

	switch(verb){
		case 0:
			logger->setLevel(log4cxx::Level::getError());
			break;
		case 1:
			logger->setLevel(log4cxx::Level::getInfo());
			break;
		case 2:
			logger->setLevel(log4cxx::Level::getDebug());
			break;
		default:
			logger->setLevel(log4cxx::Level::getError());
	}
}

bool areValidOptions(po::variables_map vm){

	if (  vm["operation_mode"].as<string>() != "CALCULATE" and vm["operation_mode"].as<string>() != "EXTRACT" and  vm["operation_mode"].as<string>() != "ICDAR" )
	{
		LOG4CXX_ERROR(logger,"The operation method specified: \"" << vm["operation_mode"].as<string>() << "\" is not valid. Operation method must be either CALCULATE,EXTRACT or ICDAR");
		return false;
	}
	if (vm["input_image"].as<string>()=="" ||  !exists(vm["input_image"].as<string>()))
	{
		LOG4CXX_ERROR(logger,"The image file specified for input: \"" << vm["input_image"].as<string>() << "\" does not exist.");
		return false;
	}
	/*if (vm["operation_mode"].as<string>() == "EXTRACT" && vm["extract_image"].as<string>()!="" && !exists(vm["extract_image"].as<string>())){
			LOG4CXX_ERROR(logger,"The image file specified for extraction: \"" << vm["extract_image"].as<string>() << "\" does not exist.");
			return false;
	}*/
	if (vm["page_file"].as<string>()=="" || !exists(vm["page_file"].as<string>()))
	{
			LOG4CXX_ERROR(logger,"The polyline file specified for extraction: \"" << vm["polyline_file"].as<string>() << "\" does not exist.");
			return false;
	}
	if ( vm["curvature_ratio"].as<int>() <  0 or vm["curvature_ratio"].as<int>() >  2)
	{
		LOG4CXX_ERROR(logger,"The curvature ratio can only take the following values: 0,1,2");
		return false;
	}
	if ( vm["delta"].as<float>() <=  0.0 )
	{
		LOG4CXX_ERROR(logger,"The direct distance constant must be a value larger than 0");
		return false;
	}
	if ( vm["beta"].as<float>() <=  0.0 )
	{
		LOG4CXX_ERROR(logger,"The diagonal distance constant must be a value larger than 0");
		return false;
	}

	if (vm["output_file"].as<string>()=="")
	{
		LOG4CXX_ERROR(logger,"Output file must not be empty");
		return false;
	}
	if ( vm["verbosity"].as<int>() < 0 or vm["verbosity"].as<int>() > 2)
	{
		LOG4CXX_ERROR(logger,"Verbosity value must be betwee 0 and 2 ");
		return false;
	}

	return true;
}

void displayInputedValues(po::variables_map vm){
	LOG4CXX_INFO(logger,"<<INPUTED PARAMETERS>>");
	LOG4CXX_INFO(logger,"Input image file       : " << vm["input_image"].as<string>());
	//LOG4CXX_INFO(logger,"Extract image file     : " << vm["extract_image"].as<string>());
	LOG4CXX_INFO(logger,"Page file              : " << vm["page_file"].as<string>());
	LOG4CXX_INFO(logger,"Operation mode         : " << vm["operation_mode"].as<string>());
	LOG4CXX_INFO(logger,"Worker threads         : " << vm["workers"].as<int>());
	LOG4CXX_INFO(logger,"Curvature ratio        : " << vm["curvature_ratio"].as<int>());
	LOG4CXX_INFO(logger,"Delta                  : " << vm["delta"].as<float>());
	LOG4CXX_INFO(logger,"Beta                   : " << vm["beta"].as<float>());
	LOG4CXX_INFO(logger,"Output file            : " << vm["output_file"].as<string>());
	LOG4CXX_INFO(logger,"Verbosity              : " << vm["verbosity"].as<int>());
}

int main(int argc, char **argv)
{

	string input_image,extract_image,output_file,page_file,operation_mode,distance_function;
	int curvature_ratio,verbosity,workers,up_dist,low_dist;
	bool rect_selected=false; 
	float direct_distance_constant,approx_dist,diagonal_distance_constant;
	string extract_file;

	BasicConfigurator::configure();
	po::options_description desc( "Allowed options" );
	desc.add_options()
		( "help,h", "Generates this help message" )
		( "input_image,i", po::value<string>(&input_image)->default_value("image.jpg"),"Input image on wich to calculate the text line contour or extract the lines as per the contours in the XML file(by default ./image.jpg)" )
		//( "extract_image,e", po::value<string>(&extract_image)->default_value(""),"Image from wich to perform the extraction (by default same as input image)" )
		( "page_file,p", po::value<string>(&page_file)->default_value("page.xml"),"File path to XML in page format containing baselines for which to calculate the contours (by default ./page.xml)" )
		( "operation_mode,m", po::value<string>(&operation_mode)->default_value("CALCULATE"), "Operation modes of the command line tool: calculate the extraction polygon (CALCULATE), extract the lines to individual images (EXTRACT) or save in ICDAR competition format (ICDAR) (default value is CALCULATE)")
		( "curvature_ratio,a", po::value<int>(&curvature_ratio)->default_value(1), "Curvature ratio to be used for the DTOCS and WDTOCS calculation (by default 1)")
		( "workers,w", po::value<int>(&workers)->default_value(1), "Number of parallel working threads to use on line extraction frontier calculation (by default 1)")
		( "delta,d", po::value<float>(&direct_distance_constant)->default_value(0.95509), "Direct distance constant to be used for the WDTOCS calculation (by default 0.95509)")
		( "beta,b", po::value<float>(&diagonal_distance_constant)->default_value(1.3693), "Diagonal distance constant to be used for the WDTOCS calculation (by default 1.36930)")
		( "approx_dist,x", po::value<float>(&approx_dist)->default_value(-1), "Allowed distance difference between the actual calculated frontier and the simplified approximation (by default -1 -> automatically calculates distance based on polygon size)")
		( "enclosing_rect,e",po::bool_switch(&rect_selected),"Return the enclosing rectangle of the detected extraction polygon instead of the actual polygon (Default false)")
		( "upper_dist,u",po::value<int>(&up_dist)->default_value(100),"Maximum allowed distance above baseline for upper region frontier calculation (by default 100)")
		( "lower_dist,l",po::value<int>(&low_dist)->default_value(25),"Maximum allowed distance below baseline for lower region frontier calculation (by default 25)")
		( "output_file,o", po::value<string>(&output_file)->default_value("output.xml"),"Base name for the lines images to be saved (by default image)" )
		( "verbosity,v", po::value<int>(&verbosity)->default_value(0), "\% Verbosity os messages during execution [0-2]");
	po::variables_map vm;
	po::store( po::parse_command_line( argc, argv, desc ), vm ); // to read command line options
	po::notify( vm );

	setVerbosity(vm["verbosity"].as<int>());




	if (vm.count("help"))
		std::cout << desc << std::endl;
	else
	{ 
		if(areValidOptions(vm))
		{
			displayInputedValues(vm);
			prhlt::Image input_image;
			prhlt::Image output_image;
			input_image.load_from_file(vm["input_image"].as<string>());
			cv::Mat orig_temp = input_image.get_matrix();
			if(vm["operation_mode"].as<string>()=="CALCULATE")
			{
				LOG4CXX_INFO(logger,"<<CALCULATING TEXT LINE CONTOURS>>");
				prhlt::Page_File page(vm["page_file"].as<string>());
				//page.generate_countour_from_baseline(30,-70);
				vector <vector< vector <cv::Point> > >  sorted_bs = page.get_sorted_baselines();
				vector <vector <cv::Point> > sorted_reg = page.get_sorted_regions();
				page.generate_countour_from_baseline(60,-100);
				//page.save_xml("test.xml");
				LOG4CXX_INFO(logger,"<<CLEANING PAGE>>");
				prhlt::Polyline_Extractor extractor_instance(orig_temp,orig_temp);
				extractor_instance.set_distance_map_parameters(vm["curvature_ratio"].as<int>(),vm["delta"].as<float>(), vm["beta"].as<float>());
				extractor_instance.run(sorted_reg, sorted_bs, vm["workers"].as<int>(), vm["approx_dist"].as<float>(), vm["enclosing_rect"].as<bool>(), vm["upper_dist"].as<int>(), vm["lower_dist"].as<int>());

				page.load_external_contours(extractor_instance.get_line_contours());
				page.save_xml(vm["output_file"].as<string>());
			}
			else
			{
				if(vm["operation_mode"].as<string>()=="EXTRACT")
				{
					LOG4CXX_INFO(logger,"<<SAVING TEXT LINE IMAGES TO FILE>>");
					prhlt::Page_File page(vm["page_file"].as<string>(),"contours");
					page.load_image(orig_temp);

					page.extract_line_images();

				}
				else
				{
					if(vm["operation_mode"].as<string>()=="ICDAR")
					{
						LOG4CXX_INFO(logger,"<<SAVING FILE TO ICDAR COMP FORMAT>>");
						prhlt::Page_File page(vm["page_file"].as<string>(),"contours");
						page.load_image(orig_temp);
						page.save_to_competition_format("");
					}
					else
					{
						LOG4CXX_ERROR(logger,"Exiting due to incorrect operation mode");
						exit(EXIT_FAILURE);
					}
				}
			}
			LOG4CXX_INFO(logger,"<<APPLICATION EXITING CORRECTLY>>");
			exit(EXIT_SUCCESS);
		}
		else
		{
			LOG4CXX_ERROR(logger,"Exiting due to incorrect input parameters");
			displayInputedValues(vm);
			exit(EXIT_FAILURE);
		}
	}
}
