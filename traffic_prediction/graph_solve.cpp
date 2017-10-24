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


#define TOL 1e-10
#define EPS 1e-12
#define MAX 1e+6
#define CLR(x) memset(x, 0, sizeof(x))

const double INF = 0x3f3f3f3f;
const int maxn = 3000;


struct edge_based_node
{
    int edge_id, source_node_id, target_node_id;
    double distance; //表示路段的距离
    double source_lng, source_lat, target_lng, target_lat; 
}edges[maxn];

struct adjacent_edge_info
{
    int edge_id;
    double distance;
};

double G[maxn][maxn];  //用来记录路段之间距离的邻接矩阵


int main(int argc, const char *argv[]) //try
{
	if (argc < 2)
	{
	  std::cerr << "Error: Not enough arguments." << std::endl
		<< "Run " << argv[0] << " data.osrm" << std::endl;
	  return EXIT_FAILURE;
	}

	using namespace osrm;

	EngineConfig config;
	config.storage_config = {argv[1]};
	config.use_shared_memory = false;

    std::cout << "111111" << std::endl;

	const OSRM osrm{config};

	MatchParameters params;

	params.steps = true;
	params.geometries = RouteParameters::GeometriesType::GeoJSON;

    std::cout << "222222" << std::endl;
    
    
    std::string  graph_file_name = "../contracted_e_node_mapping.txt";
    FILE * graph_file = fopen(graph_file_name.c_str(), "r");
    if(graph_file == NULL)
    {
        std::cout << "open graph file error" << std::endl;
        getchar();
    }
	
    int edge_id, source_id, target_id;
    int node_num = 0;
    double distance, source_lng, source_lat, target_lng, target_lat;
    while(fscanf(graph_file, "%d, %d, %d, %lf, %lf, %lf, %lf, %lf", &edge_id, &source_id, &target_id, 
        &distance, &source_lng, &source_lat, &target_lng, &target_lat) != EOF)
    {
        edges[node_num++] = {edge_id, source_id, target_id, distance, source_lng, source_lat, target_lng, target_lat};
    }
    fclose(graph_file);

    std::cout << "node_num: " << node_num << std::endl;
    std::fill(G[0], G[0] + maxn * maxn, INF);
    for(int i = 0; i < node_num; i++)
    {
        double start_lng = edges[i].target_lng;
        double start_lat = edges[i].target_lat;
        std::cout << "i = " << i << std::endl;
        for(int j = 0; j < node_num; j++)
        {
            if(i == j)
            {
                G[i][i] = 0.0;
            }
            else
            {
                double end_lng = edges[j].source_lng;
                double end_lat = edges[j].source_lat;
                params.coordinates.clear();
                params.timestamps.clear();
                params.coordinates.push_back({util::FloatLongitude{start_lng}, util::FloatLatitude{start_lat}});
                params.coordinates.push_back({util::FloatLongitude{end_lng}, util::FloatLatitude{end_lat}});
                
                json::Object result;
                const auto status = osrm.Match(params, result);
                if (status == Status::Ok)
                {
                    const auto code = result.values.at("code").get<json::String>().value;
                    //std::cout << " code: " << code << "\n";
                    //std::cout << "result: " << result << "\n";
                    const auto &tracepoints = result.values.at("tracepoints").get<json::Array>().values;
                    const auto &matchings = result.values.at("matchings").get<json::Array>().values;
                    const auto &number_of_matchings = matchings.size();
                    
                    double edge_distance = INF;
                    //std::cout << "output matching route" << std::endl;
                    for (const auto &matching_route : matchings)
                    {
                        //std::cout << "the route number is " << ++route_num << std::endl;
                        const auto &distance = matching_route.get<json::Object>().values.at("distance").get<osrm::json::Number>().value;
                        edge_distance = std::min(edge_distance, distance);
                    }
                    G[i][j] = edge_distance;
                }
                else //status is not "Ok"
                {
                    G[i][j] = INF;
                    //getchar();
                }
            }
        }
    }
    std::cout << "NPPPPPPP" << std::endl;
	
    std::string  adjacent_file_name = "adjacent_matrix.txt";
    FILE * adjacent_file = fopen(adjacent_file_name.c_str(), "w");
    if(adjacent_file == NULL)
    {
        std::cout << "open adjacent_file error" << std::endl;
        getchar();
    }
    for(int i = 0; i < node_num; i++)
    {
        fprintf(adjacent_file, "%.6f", G[i][0]);
        for(int j = 0; j < node_num; j++)
        {
            fprintf(adjacent_file, ", %.6f", G[i][j]);
        }
        fprintf(adjacent_file, "\n");
    }
    
	return EXIT_SUCCESS;
}
 

