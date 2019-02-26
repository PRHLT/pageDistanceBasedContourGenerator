SOURCES_EXTRACT_LINES_V3 = extract_lines_v3.cc
OUTPUTFILE_EXTRACT_LINES_V3 = extract_lines
#--------------------------------------------------------------------------------------
COMMONDIR = ./
INSTALLDIR= ../BIN
AUTO_INCLUDE = `pkg-config --cflags-only-I opencv liblog4cxx eigen3`
MANUAL_INCLUDE = -I$(COMMONDIR) -I/opt/local/include/
INCLUDE = $(AUTO_INCLUDE) $(MANUAL_INCLUDE)

AUTO_LIBS = `pkg-config liblog4cxx opencv eigen3 --libs`
#MANUAL_LIBS = -lboost_system-mt -lboost_program_options-mt -lboost_filesystem-mt	-lboost_thread-mt
MANUAL_LIBS = -lboost_system -lboost_program_options -lboost_filesystem	-lboost_thread -lboost_regex
EXTERNAL_LIBS = $(AUTO_LIBS) $(MANUAL_LIBS)

#--------------------------------------------------------------------------------------

INTERNAL_EXTRACT_LINES_V3_LIBS= $(COMMONDIR)/page_file.o $(COMMONDIR)/algorithm_calculate_search_area.o $(COMMONDIR)/algorithm_extract_polygons.o $(COMMONDIR)/pugixml.o $(COMMONDIR)/points_file.o $(COMMONDIR)/algorithm_distance_map.o $(COMMONDIR)/algorithm_wdtocs.o $(COMMONDIR)/algorithm_dtocs.o $(COMMONDIR)/image.o $(COMMONDIR)/algorithm_otsu.o $(COMMONDIR)/grey_level_histogram.o $(COMMONDIR)/line_histogram.o $(COMMONDIR)/algorithm_kmeans.o $(COMMONDIR)/algorithm_sauvola.o $(COMMONDIR)/algorithm_dp_path_finder.o $(COMMONDIR)/line_region_list.o $(COMMONDIR)/algorithm_dbosch.o $(COMMONDIR)/algorithm_rlsa.o $(COMMONDIR)/polyline_extractor.o

#--------------------------------------------------------------------------------------
COPTIONS = -c -O3 -o
#COPTIONS = -c -O0 -g -o
LOPTIONS = -m64 -O3 -o
OPTIONS = -w -c -O3 -pthread -o
#LOPTIONS = -O0 -g -o
#--------------------------------------------------------------------------------------
all: $(OUTPUTFILE_EXTRACT_LINES_V3)

image.o: image.cc image.hpp line_histogram.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

grey_level_histogram.o: grey_level_histogram.cc grey_level_histogram.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

line_histogram.o: line_histogram.cc line_histogram.hpp algorithm_kmeans.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

extract_lines_v3.o: $(SOURCES_EXTRACT_LINES_V3)
	$(CXX) $(COPTIONS) $@ $< $(INCLUDE)

algorithm_calculate_search_area.o: algorithm_calculate_search_area.cc algorithm_calculate_search_area.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

algorithm_distance_map.o: algorithm_distance_map.cc algorithm_distance_map.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

algorithm_dtocs.o: algorithm_dtocs.cc algorithm_dtocs.hpp algorithm_distance_map.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

algorithm_dbosch.o: algorithm_dbosch.cc algorithm_dbosch.hpp algorithm_distance_map.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

algorithm_extract_polygons.o: algorithm_extract_polygons.cc algorithm_extract_polygons.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

algorithm_wdtocs.o: algorithm_wdtocs.cc algorithm_wdtocs.hpp algorithm_distance_map.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

algorithm_kmeans.o: algorithm_kmeans.cc algorithm_kmeans.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

algorithm_otsu.o: algorithm_otsu.cc algorithm_otsu.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

algorithm_sauvola.o: algorithm_sauvola.cc algorithm_sauvola.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

algorithm_dp_path_finder.o: algorithm_dp_path_finder.cc algorithm_dp_path_finder.hpp image.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

line_region_list.o: line_region_list.cc line_region_list.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

pugixml.o: pugixml.cpp pugixml.hpp pugiconfig.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

page_file.o: page_file.cc page_file.hpp pugixml.hpp line_region_list.hpp algorithm_sauvola.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

polyline_extractor.o: polyline_extractor.cc polyline_extractor.hpp image.hpp algorithm_dbosch.hpp algorithm_sauvola.hpp algorithm_dp_path_finder.hpp line_region_list.hpp
	$(CXX) $(OPTIONS) $@ $< $(INCLUDE)

$(OUTPUTFILE_EXTRACT_LINES_V3): extract_lines_v3.o $(INTERNAL_EXTRACT_LINES_V3_LIBS)
	$(CXX) $(LDFLAGS) $(LOPTIONS) $@ $^  $(INCLUDE) $(EXTERNAL_LIBS)

install:
	cp $(OUTPUTFILE_EXTRACT_LINES_V3) $(INSTALLDIR)

clean:
	rm -rf *.o
	rm -f $(OUTPUTFILE_EXTRACT_LINES_V3)
