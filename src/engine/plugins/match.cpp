#include "engine/plugins/match.hpp"
#include "engine/plugins/plugin_base.hpp"

#include "engine/api/match_api.hpp"
#include "engine/api/match_parameters.hpp"
#include "engine/map_matching/bayes_classifier.hpp"
#include "engine/map_matching/sub_matching.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/integer_range.hpp"
#include "util/json_util.hpp"
#include "util/string_util.hpp"

#include <cstdlib>

#include <algorithm>
#include <functional>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include <unistd.h>  // linux access
#include <sys/types.h>  //linux mkdir
#include <sys/stat.h>  // linux mkdir

namespace osrm
{
namespace engine
{
namespace plugins
{

// Filters PhantomNodes to obtain a set of viable candiates
void filterCandidates(const std::vector<util::Coordinate> &coordinates,
                      MatchPlugin::CandidateLists &candidates_lists)
{
    for (const auto current_coordinate : util::irange<std::size_t>(0, coordinates.size()))
    {
        bool allow_uturn = false;

        if (coordinates.size() - 1 > current_coordinate && 0 < current_coordinate)
        {
            double turn_angle =
                util::coordinate_calculation::computeAngle(coordinates[current_coordinate - 1],
                                                           coordinates[current_coordinate],
                                                           coordinates[current_coordinate + 1]);

            // sharp turns indicate a possible uturn
            if (turn_angle <= 90.0 || turn_angle >= 270.0)
            {
                allow_uturn = true;
            }
        }

        auto &candidates = candidates_lists[current_coordinate];
        if (candidates.empty())
        {
            continue;
        }

        // sort by forward id, then by reverse id and then by distance
        std::sort(candidates.begin(),
                  candidates.end(),
                  [](const PhantomNodeWithDistance &lhs, const PhantomNodeWithDistance &rhs) {
                      return lhs.phantom_node.forward_segment_id.id <
                                 rhs.phantom_node.forward_segment_id.id ||
                             (lhs.phantom_node.forward_segment_id.id ==
                                  rhs.phantom_node.forward_segment_id.id &&
                              (lhs.phantom_node.reverse_segment_id.id <
                                   rhs.phantom_node.reverse_segment_id.id ||
                               (lhs.phantom_node.reverse_segment_id.id ==
                                    rhs.phantom_node.reverse_segment_id.id &&
                                lhs.distance < rhs.distance)));
                  });

        auto new_end =
            std::unique(candidates.begin(),
                        candidates.end(),
                        [](const PhantomNodeWithDistance &lhs, const PhantomNodeWithDistance &rhs) {
                            return lhs.phantom_node.forward_segment_id.id ==
                                       rhs.phantom_node.forward_segment_id.id &&
                                   lhs.phantom_node.reverse_segment_id.id ==
                                       rhs.phantom_node.reverse_segment_id.id;
                        });
        candidates.resize(new_end - candidates.begin());

        if (!allow_uturn)
        {
            const auto compact_size = candidates.size();
            for (const auto i : util::irange<std::size_t>(0, compact_size))
            {
                // Split edge if it is bidirectional and append reverse direction to end of list
                if (candidates[i].phantom_node.forward_segment_id.enabled &&
                    candidates[i].phantom_node.reverse_segment_id.enabled)
                {
                    PhantomNode reverse_node(candidates[i].phantom_node);
                    reverse_node.forward_segment_id.enabled = false;
                    candidates.push_back(
                        PhantomNodeWithDistance{reverse_node, candidates[i].distance});

                    candidates[i].phantom_node.reverse_segment_id.enabled = false;
                }
            }
        }

        // sort by distance to make pruning effective
        std::sort(candidates.begin(),
                  candidates.end(),
                  [](const PhantomNodeWithDistance &lhs, const PhantomNodeWithDistance &rhs) {
                      return lhs.distance < rhs.distance;
                  });
    }
}

Status MatchPlugin::HandleRequest(const datafacade::ContiguousInternalMemoryDataFacadeBase &facade,
                                  const RoutingAlgorithmsInterface &algorithms,
                                  const api::MatchParameters &parameters,
                                  util::json::Object &json_result) const
{
    if (!algorithms.HasMapMatching())
    {
        return Error("NotImplemented",
                     "Map matching is not implemented for the chosen search algorithm.",
                     json_result);
    }

    BOOST_ASSERT(parameters.IsValid());

    // enforce maximum number of locations for performance reasons
    if (max_locations_map_matching > 0 &&
        static_cast<int>(parameters.coordinates.size()) > max_locations_map_matching)
    {
        return Error("TooBig", "Too many trace coordinates", json_result);
    }

    if (!CheckAllCoordinates(parameters.coordinates))
    {
        return Error("InvalidValue", "Invalid coordinate value.", json_result);
    }

    // Check for same or increasing timestamps. Impl. note: Incontrast to `sort(first,
    // last, less_equal)` checking `greater` in reverse meets irreflexive requirements.
    const auto time_increases_monotonically = std::is_sorted(
        parameters.timestamps.rbegin(), parameters.timestamps.rend(), std::greater<>{});

    if (!time_increases_monotonically)
    {
        return Error("InvalidValue", "Timestamps need to be monotonically increasing.", json_result);
    }

    // assuming radius is the standard deviation of a normal distribution
    // that models GPS noise (in this model), x3 should give us the correct
    // search radius with > 99% confidence
    std::vector<double> search_radiuses;
    if (parameters.radiuses.empty())
    {
        search_radiuses.resize(parameters.coordinates.size(),
                               routing_algorithms::DEFAULT_GPS_PRECISION * RADIUS_MULTIPLIER);
    }
    else
    {
        search_radiuses.resize(parameters.coordinates.size());
        std::transform(parameters.radiuses.begin(),
                       parameters.radiuses.end(),
                       search_radiuses.begin(),
                       [](const boost::optional<double> &maybe_radius) {
                           if (maybe_radius)
                           {
                               return *maybe_radius * RADIUS_MULTIPLIER;
                           }
                           else
                           {
                               return routing_algorithms::DEFAULT_GPS_PRECISION * RADIUS_MULTIPLIER;
                           }

                       });
    }

    auto candidates_lists = GetPhantomNodesInRange(facade, parameters, search_radiuses);

    filterCandidates(parameters.coordinates, candidates_lists);
    if (std::all_of(candidates_lists.begin(),
                    candidates_lists.end(),
                    [](const std::vector<PhantomNodeWithDistance> &candidates) {
                        return candidates.empty();
                    }))
    {
        return Error("NoSegment",
                     std::string("Could not find a matching segment for any coordinate."),
                     json_result);
    }

    // call the actual map matching
    SubMatchingList sub_matchings = algorithms.MapMatching(
        candidates_lists, parameters.coordinates, parameters.timestamps, parameters.radiuses);

    if (sub_matchings.size() == 0)
    {
        std::cout << "no match error" << std::endl;
        return Error("NoMatch", "Could not match the trace.", json_result);
    }

    std::vector<InternalRouteResult> sub_routes(sub_matchings.size());
    for (auto index : util::irange<std::size_t>(0UL, sub_matchings.size()))
    {
        BOOST_ASSERT(sub_matchings[index].nodes.size() > 1);

        // FIXME we only run this to obtain the geometry
        // The clean way would be to get this directly from the map matching plugin
        PhantomNodes current_phantom_node_pair;
        std::cout << "sub_matchings nodes size = " << sub_matchings[index].nodes.size() << std::endl;

        /*for (unsigned i = 0; i < sub_matchings[index].nodes.size(); ++i) 
        {
          auto phantom_node = sub_matchings[index].nodes[i];
          auto indice = sub_matchings[index].indices[i];
          auto alter = sub_matchings[index].alternatives_count[i];
          std::cout << "--------------------------------" << std::endl;
          std::cout << phantom_node << std::endl;
          std::cout << "indice is " << indice << std::endl;
          std::cout << "alter is " << alter << std::endl;
        }*/

        for (unsigned i = 0; i < sub_matchings[index].nodes.size() - 1; ++i)
        {
            current_phantom_node_pair.source_phantom = sub_matchings[index].nodes[i];
            current_phantom_node_pair.target_phantom = sub_matchings[index].nodes[i + 1];


            BOOST_ASSERT(current_phantom_node_pair.source_phantom.IsValid());
            BOOST_ASSERT(current_phantom_node_pair.target_phantom.IsValid());
            sub_routes[index].segment_end_coordinates.emplace_back(current_phantom_node_pair);
        }
        // force uturns to be on, since we split the phantom nodes anyway and only have
        // bi-directional
        // phantom nodes for possible uturns
        
        sub_routes[index] = algorithms.ShortestPathSearch(sub_routes[index].segment_end_coordinates, {false});
        //sub_routes[index] = algorithms.TrafficPrediction(sub_routes[index].segment_end_coordinates, {false});


        auto &raw_route_data = sub_routes[index]; 
        
        auto number_of_routes = raw_route_data.has_alternative() ? 2UL : 1UL;
        std::cout << "number of routes is " << number_of_routes << "\n";
    
        auto number_of_legs = raw_route_data.segment_end_coordinates.size();
        std::cout << "number of legs is " << number_of_legs << "\n";
    
        auto cumulative_distance = 0.;
        std::cout << "parameters size = " << parameters.coordinates.size() << std::endl;

        /*for (auto idx : util::irange<std::size_t>(0LL, parameters.coordinates.size())) 
        {
          const auto &coordinate = parameters.coordinates[idx];
          const auto &timestamp = parameters.timestamps[idx];
          std::cout << "coordinate lng = " << coordinate.lon << ", lat = " << coordinate.lat << ", time = " << timestamp << std::endl;

        }*/

        FILE * out_data_file;
        std::string out_file_dir = "out/";
        std::string out_data_dir = out_file_dir + parameters.data_file_dir + "/";
        std::string out_data_file_name = out_data_dir + "path_" + parameters.data_file_name;
        if (access(out_file_dir.c_str(), 0) == -1)  
        {
            int flag = -1;  
            flag=mkdir(out_file_dir.c_str(), 0777);  
            if (flag == 0)  
            {  
                std::cout<<"make successfully"<<std::endl;  
            } else {  
                std::cout<<"make errorly"<<std::endl;  
            }
        }

        if (access(out_data_dir.c_str(), 0) == -1)  
        {
            mkdir(out_data_dir.c_str(), 0777);  
        }

        out_data_file = fopen(out_data_file_name.c_str(), "a+");
        if ( out_data_file == NULL ){
          std::cout << "open out_data_file error: " << out_data_file_name << std::endl;
          getchar();
        }

        fprintf(out_data_file,"-1,0,0,0,0,0,0,0 \n");

        auto prev_point_index = sub_matchings[index].indices[0];
        auto start_time_rec = parameters.timestamps[prev_point_index];

        for (auto idx : util::irange<std::size_t>(0UL, number_of_legs))
        {
          const auto &phantoms = raw_route_data.segment_end_coordinates[idx];
          const auto &path_data = raw_route_data.unpacked_path_segments[idx];

          prev_point_index = sub_matchings[index].indices[idx];
          const auto next_point_index = sub_matchings[index].indices[idx + 1];
          const auto data_time_interval = parameters.timestamps[next_point_index] - parameters.timestamps[prev_point_index];
          
          auto start_time_cal = parameters.timestamps[prev_point_index];
          auto end_time_rec = 0;
          auto end_time_cal = 0;
          //                          u       *      v
          //                          0 -- 1 -- 2 -- 3
          // fwd_segment_position:  1
          // source node fwd:       1      1 -> 2 -> 3
          // source node rev:       2 0 <- 1 <- 2

          // const auto source_segment_start_coordinate =
          //  source_node.fwd_segment_position + (reversed_source ? 1 : 0);
          // const std::vector<NodeID> source_geometry =
          //  facade.GetUncompressedForwardGeometry(source_node.packed_geometry_id);

          
          auto current_distance = 0.;
          auto prev_coordinate = phantoms.source_phantom.location;
          //std::cout << "idx = " << idx << std::endl;
          auto tmp_sum_distance = 0.;
          std::vector<double> path_distances;
          std::vector<int> time_record;  // this time comes from OSRM calculation results
          std::vector<int> time_calc;  // this time is based the proportion of distance to divide the time from car data
          std::vector<int> edge_id_list; 
          std::vector<int> name_list;
          std::vector<int> via_node_list;

          //std::cout << "start_time_rec: " << start_time_rec << ", end_time_rec: " << end_time_rec << std::endl;
          //std::cout << "start_time_cal: " << start_time_cal << ", end_time_cal: " << end_time_cal << std::endl;

          for (const auto &path_point : path_data)
          {
              auto coordinate = facade.GetCoordinateOfNode(path_point.turn_via_node);
              auto eta = path_point.duration_until_turn;      
              current_distance =
                  util::coordinate_calculation::haversineDistance(prev_coordinate, coordinate);
              cumulative_distance += current_distance;

              path_distances.push_back(current_distance);
              time_record.push_back(eta);
              edge_id_list.push_back(path_point.edge_id);
              via_node_list.push_back(path_point.turn_via_node);
              name_list.push_back(path_point.name_id);

              tmp_sum_distance += current_distance;
              prev_coordinate = coordinate;
              /*std::cout << "edge id: " << path_point.edge_id <<
                  " name: " << path_point.name_id <<
                  " eta: " << eta <<
                  " current distance: " << current_distance << "\n";  */    
          }
          

          

          for (unsigned i = 0; i < path_distances.size(); i++)
          {
            end_time_cal = start_time_cal + (int)(path_distances[i]/tmp_sum_distance*data_time_interval);
            end_time_rec = start_time_rec + time_record[i];
            fprintf(out_data_file,"%d,%d,%d,%d,%d,%d,%d,%f \n", start_time_cal, end_time_cal, 
              start_time_rec, end_time_rec,edge_id_list[i],via_node_list[i],name_list[i],path_distances[i]);
            start_time_rec = end_time_rec;
            start_time_cal = end_time_cal;
          }
          fprintf(out_data_file, "------------------ \n");

          current_distance =
          util::coordinate_calculation::haversineDistance(prev_coordinate, phantoms.target_phantom.location);
          cumulative_distance += current_distance;
        }
        fflush(out_data_file);
        fclose(out_data_file);
        //std::cout << "cumulative_distance: " << cumulative_distance << std::endl; 

        BOOST_ASSERT(sub_routes[index].shortest_path_length != INVALID_EDGE_WEIGHT);
    }


    api::MatchAPI match_api{facade, parameters};
    match_api.MakeResponse(sub_matchings, sub_routes, json_result);

    return Status::Ok;
}
}
}
}
