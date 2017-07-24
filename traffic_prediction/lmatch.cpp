#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

#include <exception>
#include <cstdio>
#include <iostream>
#include <string>
#include <utility>

#include <cstdlib>

#include <unistd.h>  // linux access
#include <sys/types.h>  //linux mkdir
#include <sys/stat.h>  // linux mkdir
#include <typeinfo>

// #include <boost/property_tree/ptree.hpp>
// #include <boost/property_tree/xml_parser.hpp>

// #include <string>
// #include <utility>
// #include <iostream>
// #include <exception>
// #include <cstdlib>
// #include <cstdio>

#define TOL 1e-10
#define EPS 1e-12
#define MAX 1e+6
#define max_path_length 10000
#define max_geometry_length 20000

struct MyCoordinate
{
  double lat;
  double lon;
};

int main(int argc, const char *argv[]) //try
{
	if (argc < 2)
	{
	  std::cerr << "Error: Not enough arguments." << std::endl
		<< "Run " << argv[0] << " data.osrm" << std::endl;
	  return EXIT_FAILURE;
	}

	int DEBUG = 1;

	std::vector<int> raw_timestamps;
	std::vector<int> washed_timestamps;

	double* geometry;
	geometry = (double *)malloc(2*max_geometry_length*sizeof(double));
	double* mpoints;
	mpoints = (double *)malloc(2*max_geometry_length*sizeof(double));
	double* timestamps;
	timestamps = (double *)malloc(max_geometry_length*sizeof(double));
	double* carpath;
	carpath = (double *)malloc(6*max_path_length*sizeof(double));
	int cnp;
	int ng, nm;
	int path_counter = 0;

	if (geometry == NULL || mpoints == NULL || carpath == NULL)
	{
	  printf("malloc failed\n");
	  return EXIT_FAILURE;
	}

	using namespace osrm;

	EngineConfig config;
	config.storage_config = {argv[1]};
	config.use_shared_memory = false;

	const OSRM osrm{config};

	MatchParameters params;

	params.steps = true;
	params.geometries = RouteParameters::GeometriesType::GeoJSON;
    
    //params.data_file_dir = argv[2];
	

    //params.overview = RouteParameters::OverView::Full;
	//params.radius = 15.0;
	//params.classify = true;
	//params.compression = false;
	//params.matching_beta = 0.1;
	//params.gps_precision = 10;

	// Handling datafiles
	
    //std::string day_string(argv[2]);
    //std::cout << "day = " << day_string << std::endl; 

	//total output file
	

	//std::string base_data_path = "data/2013FebMarch/unLoadedRoutes/";
	//std::string base_data_path = "../../../data/LoadedRoutes/";
    std::string base_data_path = "/media/jhljx/Elements/Rsync/LoadedRoutes/";

    std::vector<std::string> date_string_vec = {"20130225","20130226","20130227","20130228","20130301","20130302","20130303"};

    std::cout << "Hello world!" << std::endl;

    for(int date_idx = 0; date_idx < date_string_vec.size(); date_idx ++)
    {
        params.data_file_dir = date_string_vec[date_idx];
        std::string day_string = params.data_file_dir;
        std::string data_path = base_data_path + day_string + "/";

        std::cout << "day = " << date_string_vec[date_idx] << std::endl;

        // std::string data_path = "../../../data/2013FebMarch/LoadedRoutes/test/";
        FILE * mpoints_output;
        std::string mpoints_output_filename = "mpoints_output_" + day_string + ".txt";
        mpoints_output = fopen (mpoints_output_filename.c_str(), "w");
        if ( mpoints_output == NULL ){
            std::cout << "open mpoints_output_file error" << std::endl;
            getchar();
        }
        

        std::string in_file_list_name = data_path + "filelist.txt";
        std::cout << in_file_list_name << std::endl;
        FILE * in_file_list;
        in_file_list = fopen (in_file_list_name.c_str(), "r");
        if ( in_file_list == NULL ){
            std::cout << "open in_file_list error" << in_file_list_name << std::endl;
            getchar();
        }

        std::string out_data_file_name;
        /*FILE * out_data_file;
        std::string out_data_dir = "out/";
        out_data_file_name = out_data_dir + "path_" + day_string + ".txt";
        //std::cout << "[out] " << out_data_file_name << std::endl;
        if (access(out_data_dir.c_str(), 0) == -1)  
        {
            int flag = -1;  
            flag=mkdir(out_data_dir.c_str(), 0777);  
            if (flag == 0)  
            {  
                std::cout<<"make successfully"<<std::endl;  
            } else {  
                std::cout<<"make errorly"<<std::endl;  
            }
        }
        /*out_data_file = fopen(out_data_file_name.c_str(), "w");
        if ( out_data_file == NULL ){
            std::cout << "open out_data_file error: " << out_data_file_name << std::endl;
            getchar();
        }*/
        char in_data_file_name_part[200];
        std::string in_data_file_name;
        FILE * in_data_file;

        double tick, lon, lat, v1, v2, b1, b2;
        int path_length, path_id, header_flag, loaded_flag;
        std::vector<MyCoordinate> readin_coords;
        std::vector<unsigned> readin_timestamps;
        //std::vector<double> readin_speeds;
        //std::vector<double> readin_speeds_gps;
        
        //int num = 0;
        while(fscanf(in_file_list, "%s", in_data_file_name_part)!=EOF)
        {
            in_data_file_name = data_path + in_data_file_name_part;
            in_data_file = fopen (in_data_file_name.c_str(), "r");
            if ( in_data_file == NULL ){
                std::cout << "open in_data_file error" << std::endl;
                getchar();
            }
            params.data_file_name = in_data_file_name_part;
            //num ++;
            std::cout << "File name is " << in_data_file_name_part << std::endl; 
            /*if (num == 2) 
            {
                break;
            }*/

            while(fscanf(in_data_file, "%d,%lf,%lf,%d,%lf,%d,%lf,%d",
                &header_flag, &lon, &lat, &path_id, &v2,
                &path_length, &b2, &loaded_flag)!=EOF)
            {
                //std::cout << "Loaded path " << path_id << " with length "
                //<< path_length << std::endl;

                /*if (path_id ==2) {
                    break;
                }*/

                /////////////////////////////////
                // *********  DEBUG ********** //
                /////////////////////////////////
                FILE * GPS_records;
                FILE * geometry_output;
                FILE * geometry_full_output;
                if (DEBUG)
                {
                    GPS_records = fopen ("GPS_records.txt", "w");
                    if (GPS_records == NULL )
                    {
                        std::cout << "open GPS_records.txt error" << std::endl;
                        getchar();
                    }
                }

                readin_coords.clear();
                readin_timestamps.clear();
                //readin_speeds.clear();
                //readin_speeds_gps.clear();
                MyCoordinate current_readin_coordinate;

                for ( int i=0; i<path_length; i++ )
                {
                    int status = fscanf(in_data_file, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d",
                     &tick, &lon, &lat,
                     &v1, &v2, &b1, &b2,&loaded_flag);
                    current_readin_coordinate.lat = lat;
                    current_readin_coordinate.lon = lon;
                    //std::cout << tick << " " << current_readin_coordinate.lon << " " <<
                    //    current_readin_coordinate.lat << std::endl;
                    readin_coords.push_back(current_readin_coordinate);
                    readin_timestamps.push_back(tick);
                    //readin_speeds.push_back(v2);
                    //readin_speeds_gps.push_back(v1);
                }

                params.coordinates.clear();
                params.timestamps.clear();
                raw_timestamps.clear();
                washed_timestamps.clear();
                int iiold = 0;
                double dt, dist;
                params.coordinates.push_back({
                    util::FloatLongitude{readin_coords[0].lon},
                    util::FloatLatitude{readin_coords[0].lat}});
                params.timestamps.push_back(readin_timestamps[0]);
                //params.AddCoordinate(readin_coords[0].lat, readin_coords[0].lon);
                //params.AddTimestamp(readin_timestamps[0]);
                raw_timestamps.push_back(readin_timestamps[0]);
                /////////////////////////////////
                // *********  DEBUG ********** //
                /////////////////////////////////
                if (DEBUG)
                {
                    fprintf(GPS_records, "%d %lf %lf \n", readin_timestamps[0],
                      readin_coords[0].lon, readin_coords[0].lat);
                }

                for ( int ii=1; ii<path_length; ii++ )
                {
                    // dt = readin_timestamps[ii] - readin_timestamps[iiold];
                    // dist = sqrt(pow((readin_coords[ii].lat-readin_coords[iiold].lat)*85.5, 2.0) +
                    //      pow((readin_coords[ii].lon-readin_coords[iiold].lon)*111.0, 2.0) );

                    //std::cout << dt << "," << dist << std::endl;

                    //if ( (dt > 10.0) && (dist > 0.1) )
                    {
                        params.coordinates.push_back({
                            util::FloatLongitude{readin_coords[ii].lon},
                            util::FloatLatitude{readin_coords[ii].lat}});
                        params.timestamps.push_back(readin_timestamps[ii]);
                        //params.AddCoordinate(readin_coords[ii].lat, readin_coords[ii].lon);
                        //params.AddTimestamp(readin_timestamps[ii]);
                        raw_timestamps.push_back(readin_timestamps[ii]);
                        iiold = ii;
                        /////////////////////////////////
                        // *********  DEBUG ********** //
                        /////////////////////////////////
                        if (DEBUG)
                        {
                            fprintf(GPS_records, "%d %lf %lf \n", readin_timestamps[ii],
                                readin_coords[ii].lon, readin_coords[ii].lat);
                        }

                    }
                }
                /////////////////////////////////
                // *********  DEBUG ********** //
                /////////////////////////////////
                if (DEBUG)
                {
                    fclose(GPS_records);
                }

                //std::cout << "Loaded Path: " << path_id << " with length: " << path_length << std::endl;
                //<< " ( Input file: " << in_data_file_name << ")"
                //<< std::endl;

                json::Object result;

                // Execute routing request, this does the heavy lifting
                //try
                //{
                const auto status = osrm.Match(params, result);
                //}
                //catch (const std::exception &current_exception)
                // catch (...)
                //   {
                //     std::cout << "there is exception!";
                //     getchar();
                //   }
                /////////////////////////////////
                // *********  DEBUG ********** //
                /////////////////////////////////
                if (DEBUG)
                {
                    geometry_output = fopen ("geometry_output.txt", "w");
                    if ( geometry_output == NULL ){
                        std::cout << "open geometry_output.txt error" << std::endl;
                        getchar();
                    }
                    geometry_full_output = fopen ("geometry_full_output.txt", "w");
                    if ( geometry_full_output == NULL ){
                        std::cout << "open geometry_full_output.txt error" << std::endl;
                        getchar();
                    }
                }

                if (status == Status::Ok)
                {
                    const auto code = result.values.at("code").get<json::String>().value;
                    //std::cout << " code: " << code << "\n";
                    //std::cout << "result: " << result << "\n";
                    const auto &tracepoints = result.values.at("tracepoints").get<json::Array>().values;
                    const auto &matchings = result.values.at("matchings").get<json::Array>().values;
                    const auto &number_of_matchings = matchings.size();

                    //std::cout << "number of matching is " << number_of_matchings << std::endl;

                    // const auto &route_legs = matchings[0]
                    //   .get<json::Object>()
                    //   .values.at("legs")
                    //   .get<json::Array>()
                    //   .values;
                    int route_num = 0;
                    //std::cout << "output matching route" << std::endl;
                    for (const auto &matching_route : matchings)
                    {
                        //std::cout << "the route number is " << ++route_num << std::endl;

                        const auto &geometries = matching_route
                        .get<json::Object>()
                        .values.at("geometry")
                        .get<json::Object>();
                        const auto &geometries_coordinates = geometries
                        .values.at("coordinates")
                        .get<json::Array>()
                        .values;

                        //std::cout << "the size of geometries_coordinates is " << geometries_coordinates.size() << std::endl;

                        for (const auto &json_coordinate : geometries_coordinates)
                        {
                            lon = json_coordinate.get<json::Array>().values[0].get<osrm::json::Number>().value;
                            lat = json_coordinate.get<json::Array>().values[1].get<osrm::json::Number>().value;
                            //std::cout << lon << ", " << lat << std::endl;

                            /////////////////////////////////
                            // *********  DEBUG ********** //
                            /////////////////////////////////
                            if (DEBUG)
                            {
                                fprintf(geometry_full_output, "%lf %lf \n", lon, lat);
                            }
                        }
                        //std::cout << std::endl;
                        const auto &distance = matching_route
                        .get<json::Object>()
                        .values.at("distance").get<osrm::json::Number>().value;
                        const auto &duration = matching_route
                        .get<json::Object>()
                        .values.at("duration").get<osrm::json::Number>().value;
                        const auto &weight = matching_route
                        .get<json::Object>()
                        .values.at("weight").get<osrm::json::Number>().value;

                        //std::cout << "route distance is " << distance << std::endl;
                        //std::cout << "route duration is " << duration << std::endl;
                        //std::cout << "route weight is " << weight << std::endl;
                        
                        const auto &route_legs = matching_route
                        .get<json::Object>()
                        .values.at("legs")
                        .get<json::Array>()
                        .values;

                        //std::cout << "the route legs' size is " << route_legs.size() << std::endl; 

                        for (const auto &route_leg : route_legs) 
                        {
                            auto &leg_distance = route_leg.get<json::Object>().values.at("distance").get<osrm::json::Number>().value;
                            /*double &leg_duration = route_leg.get<json::Object>().values.at("duration").get<json::Number>.value;
                            double &leg_weight = route_leg.get<json::Object>().values.at("weight").get<json::Number>.value;
                            std::string leg_summary = route_leg.get<json::Object>().values.at("summary").get<json::String>.value; 
                            
                            std::cout << "leg_distance: " << leg_distance << std::endl;
                            std::cout << "leg_duration: " << leg_duration << std::endl;
                            std::cout << "leg_weight: " << leg_weight << std::endl;

                            const auto &route_steps = route_leg.get<json::Object>().values.at("steps").get<json::Array>().values;
                            for (const auto &route_step: route_steps)
                            {
                                double &route_step_distance = route_step.get<json::Object>().values.at("distance").get(json::Number).value;
                                double &route_step_duration = route_step.get<json::Object>().values.at("duration").get(json::Number).value;
                                double &route_step_weight = route_step.get<json::Object>().values.at("weight").get(json::Number).value;

                            }*/
                        }


                    }
                    nm = 0;
                    //std:: cout << "trace point size = " << tracepoints.size() << std:: endl;
                    for (const auto &waypoint : tracepoints)
                    {
                        if (waypoint.is<mapbox::util::recursive_wrapper<util::json::Object>>())
                        {
                            const auto &waypoint_object = waypoint.get<json::Object>();
                            const auto matchings_index =
                            waypoint_object.values.at("matchings_index").get<json::Number>().value;
                            const auto waypoint_index =
                            waypoint_object.values.at("waypoint_index").get<json::Number>().value;
                            const auto waypoint_name =
                            waypoint_object.values.at("name").get<json::String>().value;
                            const auto waypoint_hint =
                            waypoint_object.values.at("hint").get<json::String>().value;
                            //std::cout << " Waypoint_hint: " << waypoint_hint << "\n";

                            // const auto &route_legs = matchings[matchings_index]
                            //   .get<json::Object>()
                            //   .values.at("legs")
                            //   .get<json::Array>()
                            //   .values;

                            //std::cout << " Matched_id: " << matchings_index << "-Waypoint_id: " << waypoint_index << "-" << waypoint_name << "\n";

                            const auto &waypoint_location =
                            waypoint_object.values.at("location").get<json::Array>().values;
                            lon = waypoint_location[0].get<json::Number>().value;
                            lat = waypoint_location[1].get<json::Number>().value;
                            
                            //std::cout << "trace coord:" << lon << ", " << lat << std::endl;

                            /////////////////////////////////
                            // *********  DEBUG ********** //
                            /////////////////////////////////
                            if (DEBUG)
                            {
                                fprintf(geometry_output, "%lf %lf \n", lon, lat);
                            }

                            fprintf(mpoints_output, "%s, %d, %lf, %lf, %lf, %lf, %s\n",
                              argv[2], readin_timestamps[nm], lon, lat,
                              readin_coords[nm].lon, readin_coords[nm].lat,
                              waypoint_name.c_str());
                            //waypoint_hint.c_str());

                            /////////////////////////////////
                            // *********  DEBUG ********** //
                            /////////////////////////////////
                            // if (DEBUG)
                            // {
                            //  if (path_counter == 179)
                            //  //if (path_id == 25)
                            //  {
                            //      fflush(out_data_file);
                            //      getchar();
                            //  }
                            // }

                            //std::cout << "route_legs" << route_legs << "\n";

                            // for (int j=0; j<route_legs.size(); j++)
                            //   {
                            //  // const auto &leg = route_legs[j]
                            //  //   .get<json::Object>()
                            //  //   .values;

                            //  const auto distance = route_legs[j]
                            //    .get<json::Object>()
                            //    .values.at("distance")
                            //    .get<json::Number>()
                            //    .value;
                            //  const auto duration = route_legs[j]
                            //    .get<json::Object>()
                            //    .values.at("duration")
                            //    .get<json::Number>()
                            //    .value;
                            //  const auto summary = route_legs[j]
                            //    .get<json::Object>()
                            //    .values.at("summary")
                            //    .get<json::String>()
                            //    .value;
                            //  // Warn users if extract does not contain the default Berlin coordinates from above
                            //  if (distance == 0 or duration == 0)
                            //    {
                            //      std::cout << "Note: distance or duration is zero. ";
                            //      std::cout << "You are probably doing a query outside of the OSM extract.\n\n";
                            //    }

                            //  std::cout << "Distance: " << distance << " meter\n";
                            //  std::cout << "Duration: " << duration << " seconds\n";
                            //  std::cout << "Summary: " << summary << " seconds\n";
                            //   }
                        }
                        else
                        {
                          //std::cout << " unMatched. \n";
                          //getchar();
                        }
                        nm = nm + 1;
                    }
                }
                else //status is not "Ok"
                {
                    std::cout << "status type is " << typeid(status).name() << std::endl;
                    std::cout << "HTTP status error: " << result.values.at("message").get<json::String>().value << "\n";
                    //getchar();
                }
                /////////////////////////////////
                // *********  DEBUG ********** //
                /////////////////////////////////
                if (DEBUG)
                {
                    fflush(geometry_output);
                    fclose(geometry_output);
                    fflush(geometry_full_output);
                    fclose(geometry_full_output);
                    fflush(mpoints_output);
                    //getchar();
                }
            }
            //fflush(out_data_file);
            //getchar();
            fclose (in_data_file);
        }
        
        //fclose (out_data_file);
        fclose (in_file_list);
        
    }
    free(geometry);
    free(mpoints);
    free(timestamps);
    free(carpath);
    
	return EXIT_SUCCESS;
}
// catch (const std::exception &current_exception)
//   {
//     std::cout << "exception: " << current_exception.what();
//     return EXIT_FAILURE;
//   }
 

