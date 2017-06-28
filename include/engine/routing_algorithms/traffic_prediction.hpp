#ifndef TRAFFIC_PREDICTION_HPP
#define TRAFFIC_PREDICTION_HPP

#include "engine/algorithm.hpp"
#include "engine/routing_algorithms/routing_base.hpp"
#include "engine/search_engine_data.hpp"
#include "util/typedefs.hpp"

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

InternalRouteResult
trafficPrediction(SearchEngineData &engine_working_data,
                   const datafacade::ContiguousInternalMemoryDataFacade<algorithm::CH> &facade,
                   const std::vector<PhantomNodes> &phantom_nodes_vector,
                   const boost::optional<bool> continue_straight_at_waypoint);

InternalRouteResult
trafficPrediction(SearchEngineData &engine_working_data,
                   const datafacade::ContiguousInternalMemoryDataFacade<algorithm::CoreCH> &facade,
                   const std::vector<PhantomNodes> &phantom_nodes_vector,
                   const boost::optional<bool> continue_straight_at_waypoint);

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm

#endif /* TRAFFIC_PREDICTION_HPP */
