#ifndef MATCH_HPP
#define MATCH_HPP

#include "engine/api/match_parameters.hpp"
#include "engine/plugins/plugin_base.hpp"
#include "engine/routing_algorithms.hpp"

#include "engine/map_matching/bayes_classifier.hpp"
#include "engine/routing_algorithms/map_matching.hpp"
#include "engine/routing_algorithms/shortest_path.hpp"
#include "util/json_util.hpp"

#include <vector>

namespace osrm
{
namespace engine
{
namespace plugins
{

class MatchPlugin : public BasePlugin
{
  public:
    using SubMatching = map_matching::SubMatching;
    using SubMatchingList = routing_algorithms::SubMatchingList;
    using CandidateLists = routing_algorithms::CandidateLists;
    static const constexpr double RADIUS_MULTIPLIER = 3;

    MatchPlugin(const int max_locations_map_matching)
        : max_locations_map_matching(max_locations_map_matching)
    {
      
      // 这个地方从edge_node_mapping文件中读取映射存储到哈希表中
      std::string edge_node_list_name = "e_to_node_id_mapping.txt";
      FILE * edge_node_list = fopen (edge_node_list_name.c_str(), "r");
      if ( edge_node_list == NULL ){
        std::cout << "open edge_node_list error " << edge_node_list_name << std::endl;
        getchar();
      }

      edge_node_mapping_list.clear();
      int e_id = 0, from_node_id = 0, to_node_id = 0, e_direction = 0;
      double edge_distance = 0.0;
      //std::cout << "Start edge file" << std::endl;
      while(fscanf(edge_node_list, "%d,%d,%d,%d,%lf", &e_id, &from_node_id, &to_node_id, &e_direction, &edge_distance)!=EOF)
      {
        //std::cout << "record edge " << e_id << std::endl;
        edge_node_mapping_list.insert({e_id, std::make_tuple(from_node_id,to_node_id,e_direction)});
      }
      fclose(edge_node_list);
      //std::cout << "Finish edge file" << std::endl;
    }

    Status HandleRequest(const datafacade::ContiguousInternalMemoryDataFacadeBase &facade,
                         const RoutingAlgorithmsInterface &algorithms,
                         const api::MatchParameters &parameters,
                         util::json::Object &json_result) const;

  private:
    const int max_locations_map_matching;
    std::unordered_map<int, std::tuple<NodeID,NodeID,int> > edge_node_mapping_list;
};
}
}
}

#endif // MATCH_HPP
