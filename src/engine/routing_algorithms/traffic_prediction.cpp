#include "engine/routing_algorithms/traffic_prediction.hpp"
#include "engine/routing_algorithms/routing_base.hpp"

#include <boost/assert.hpp>
#include <boost/optional.hpp>
#include <memory>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

namespace
{

const static constexpr bool DO_NOT_FORCE_LOOP = false;
using QueryHeap = SearchEngineData::QueryHeap;

// allows a uturn at the target_phantom
// searches source forward/reverse -> target forward/reverse
template <typename AlgorithmT>
void searchWithUTurn(const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT> &facade,
                     QueryHeap &forward_heap,
                     QueryHeap &reverse_heap,
                     QueryHeap &forward_core_heap,
                     QueryHeap &reverse_core_heap,
                     const bool search_from_forward_node,
                     const bool search_from_reverse_node,
                     const bool search_to_forward_node,
                     const bool search_to_reverse_node,
                     const PhantomNode &source_phantom,
                     const PhantomNode &target_phantom,
                     const int total_weight_to_forward,
                     const int total_weight_to_reverse,
                     int &new_total_weight,
                     std::vector<NodeID> &leg_packed_path)
{
    forward_heap.Clear();
    reverse_heap.Clear();
    if (search_from_forward_node)
    {
        forward_heap.Insert(source_phantom.forward_segment_id.id,
                            -source_phantom.GetForwardWeightPlusOffset(),
                            source_phantom.forward_segment_id.id);
    }
    if (search_from_reverse_node)
    {
        forward_heap.Insert(source_phantom.reverse_segment_id.id,
                            -source_phantom.GetReverseWeightPlusOffset(),
                            source_phantom.reverse_segment_id.id);
    }
    if (search_to_forward_node)
    {
        reverse_heap.Insert(target_phantom.forward_segment_id.id,
                            target_phantom.GetForwardWeightPlusOffset(),
                            target_phantom.forward_segment_id.id);
    }
    if (search_to_reverse_node)
    {
        reverse_heap.Insert(target_phantom.reverse_segment_id.id,
                            target_phantom.GetReverseWeightPlusOffset(),
                            target_phantom.reverse_segment_id.id);
    }

    BOOST_ASSERT(forward_heap.Size() > 0);
    BOOST_ASSERT(reverse_heap.Size() > 0);

    // this is only relevent if source and target are on the same compressed edge
    auto is_oneway_source = !(search_from_forward_node && search_from_reverse_node);
    auto is_oneway_target = !(search_to_forward_node && search_to_reverse_node);
    // we only enable loops here if we can't search from forward to backward node
    auto needs_loop_forwad = is_oneway_source && needsLoopForward(source_phantom, target_phantom);
    auto needs_loop_backwards =
        is_oneway_target && needsLoopBackwards(source_phantom, target_phantom);

    forward_core_heap.Clear();
    reverse_core_heap.Clear();
    BOOST_ASSERT(forward_core_heap.Size() == 0);
    BOOST_ASSERT(reverse_core_heap.Size() == 0);
    routing_algorithms::search(facade,
                               forward_heap,
                               reverse_heap,
                               forward_core_heap,
                               reverse_core_heap,
                               new_total_weight,
                               leg_packed_path,
                               needs_loop_forwad,
                               needs_loop_backwards);

    // if no route is found between two parts of the via-route, the entire route becomes
    // invalid. Adding to invalid edge weight sadly doesn't return an invalid edge weight. Here
    // we prevent the possible overflow, faking the addition of infinity + x == infinity
    if (new_total_weight != INVALID_EDGE_WEIGHT)
        new_total_weight += std::min(total_weight_to_forward, total_weight_to_reverse);
}

// searches shortest path between:
// source forward/reverse -> target forward
// source forward/reverse -> target reverse
template <typename AlgorithmT>
void search(const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT> &facade,
            QueryHeap &forward_heap,
            QueryHeap &reverse_heap,
            QueryHeap &forward_core_heap,
            QueryHeap &reverse_core_heap,
            const bool search_from_forward_node,
            const bool search_from_reverse_node,
            const bool search_to_forward_node,
            const bool search_to_reverse_node,
            const PhantomNode &source_phantom,
            const PhantomNode &target_phantom,
            const int total_weight_to_forward,
            const int total_weight_to_reverse,
            int &new_total_weight_to_forward,
            int &new_total_weight_to_reverse,
            std::vector<NodeID> &leg_packed_path_forward,
            std::vector<NodeID> &leg_packed_path_reverse)
{
    if (search_to_forward_node)
    {
        forward_heap.Clear();
        reverse_heap.Clear();
        reverse_heap.Insert(target_phantom.forward_segment_id.id,
                            target_phantom.GetForwardWeightPlusOffset(),
                            target_phantom.forward_segment_id.id);

        if (search_from_forward_node)
        {
            forward_heap.Insert(source_phantom.forward_segment_id.id,
                                total_weight_to_forward -
                                    source_phantom.GetForwardWeightPlusOffset(),
                                source_phantom.forward_segment_id.id);
        }
        if (search_from_reverse_node)
        {
            forward_heap.Insert(source_phantom.reverse_segment_id.id,
                                total_weight_to_reverse -
                                    source_phantom.GetReverseWeightPlusOffset(),
                                source_phantom.reverse_segment_id.id);
        }
        BOOST_ASSERT(forward_heap.Size() > 0);
        BOOST_ASSERT(reverse_heap.Size() > 0);

        forward_core_heap.Clear();
        reverse_core_heap.Clear();
        BOOST_ASSERT(forward_core_heap.Size() == 0);
        BOOST_ASSERT(reverse_core_heap.Size() == 0);
        routing_algorithms::search(facade,
                                   forward_heap,
                                   reverse_heap,
                                   forward_core_heap,
                                   reverse_core_heap,
                                   new_total_weight_to_forward,
                                   leg_packed_path_forward,
                                   needsLoopForward(source_phantom, target_phantom),
                                   DO_NOT_FORCE_LOOP);
    }

    if (search_to_reverse_node)
    {
        forward_heap.Clear();
        reverse_heap.Clear();
        reverse_heap.Insert(target_phantom.reverse_segment_id.id,
                            target_phantom.GetReverseWeightPlusOffset(),
                            target_phantom.reverse_segment_id.id);
        if (search_from_forward_node)
        {
            forward_heap.Insert(source_phantom.forward_segment_id.id,
                                total_weight_to_forward -
                                    source_phantom.GetForwardWeightPlusOffset(),
                                source_phantom.forward_segment_id.id);
        }
        if (search_from_reverse_node)
        {
            forward_heap.Insert(source_phantom.reverse_segment_id.id,
                                total_weight_to_reverse -
                                    source_phantom.GetReverseWeightPlusOffset(),
                                source_phantom.reverse_segment_id.id);
        }
        BOOST_ASSERT(forward_heap.Size() > 0);
        BOOST_ASSERT(reverse_heap.Size() > 0);
        forward_core_heap.Clear();
        reverse_core_heap.Clear();
        BOOST_ASSERT(forward_core_heap.Size() == 0);
        BOOST_ASSERT(reverse_core_heap.Size() == 0);
        routing_algorithms::search(facade,
                                   forward_heap,
                                   reverse_heap,
                                   forward_core_heap,
                                   reverse_core_heap,
                                   new_total_weight_to_reverse,
                                   leg_packed_path_reverse,
                                   DO_NOT_FORCE_LOOP,
                                   needsLoopBackwards(source_phantom, target_phantom));
    }
}

void unpackLegs(const datafacade::ContiguousInternalMemoryDataFacade<algorithm::CH> &facade,
                const std::vector<PhantomNodes> &phantom_nodes_vector,
                const std::vector<NodeID> &total_packed_path,
                const std::vector<std::size_t> &packed_leg_begin,
                const int shortest_path_length,
                InternalRouteResult &raw_route_data)
{
    raw_route_data.unpacked_path_segments.resize(packed_leg_begin.size() - 1);

    raw_route_data.shortest_path_length = shortest_path_length;

    for (const auto current_leg : util::irange<std::size_t>(0UL, packed_leg_begin.size() - 1))
    {
        auto leg_begin = total_packed_path.begin() + packed_leg_begin[current_leg];
        auto leg_end = total_packed_path.begin() + packed_leg_begin[current_leg + 1];
        const auto &unpack_phantom_node_pair = phantom_nodes_vector[current_leg];
        unpackPath(facade,
                   leg_begin,
                   leg_end,
                   unpack_phantom_node_pair,
                   raw_route_data.unpacked_path_segments[current_leg]);

        raw_route_data.source_traversed_in_reverse.push_back(
            (*leg_begin != phantom_nodes_vector[current_leg].source_phantom.forward_segment_id.id));
        raw_route_data.target_traversed_in_reverse.push_back(
            (*std::prev(leg_end) !=
             phantom_nodes_vector[current_leg].target_phantom.forward_segment_id.id));
    }
}

template <typename AlgorithmT>
InternalRouteResult
trafficPredictionImpl(SearchEngineData &engine_working_data,
                       const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT> &facade,
                       const std::vector<PhantomNodes> &phantom_nodes_vector,
                       const boost::optional<bool> continue_straight_at_waypoint)
{
    InternalRouteResult raw_route_data;
    raw_route_data.segment_end_coordinates = phantom_nodes_vector;
    const bool allow_uturn_at_waypoint =
        !(continue_straight_at_waypoint ? *continue_straight_at_waypoint
                                        : facade.GetContinueStraightDefault());

    engine_working_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes());
    engine_working_data.InitializeOrClearSecondThreadLocalStorage(facade.GetNumberOfNodes());

    QueryHeap &forward_heap = *(engine_working_data.forward_heap_1);
    QueryHeap &reverse_heap = *(engine_working_data.reverse_heap_1);
    QueryHeap &forward_core_heap = *(engine_working_data.forward_heap_2);
    QueryHeap &reverse_core_heap = *(engine_working_data.reverse_heap_2);

    int total_weight_to_forward = 0;
    int total_weight_to_reverse = 0;
    bool search_from_forward_node =
        phantom_nodes_vector.front().source_phantom.forward_segment_id.enabled;
    bool search_from_reverse_node =
        phantom_nodes_vector.front().source_phantom.reverse_segment_id.enabled;

    std::vector<NodeID> prev_packed_leg_to_forward;
    std::vector<NodeID> prev_packed_leg_to_reverse;

    std::vector<NodeID> total_packed_path_to_forward;
    std::vector<std::size_t> packed_leg_to_forward_begin;
    std::vector<NodeID> total_packed_path_to_reverse;
    std::vector<std::size_t> packed_leg_to_reverse_begin;

    std::size_t current_leg = 0;
    // this implements a dynamic program that finds the shortest route through
    // a list of vias
    for (const auto &phantom_node_pair : phantom_nodes_vector)
    {
        int new_total_weight_to_forward = INVALID_EDGE_WEIGHT;
        int new_total_weight_to_reverse = INVALID_EDGE_WEIGHT;

        std::vector<NodeID> packed_leg_to_forward;
        std::vector<NodeID> packed_leg_to_reverse;

        const auto &source_phantom = phantom_node_pair.source_phantom;
        const auto &target_phantom = phantom_node_pair.target_phantom;

        bool search_to_forward_node = target_phantom.forward_segment_id.enabled;
        bool search_to_reverse_node = target_phantom.reverse_segment_id.enabled;

        BOOST_ASSERT(!search_from_forward_node || source_phantom.forward_segment_id.enabled);
        BOOST_ASSERT(!search_from_reverse_node || source_phantom.reverse_segment_id.enabled);

        BOOST_ASSERT(search_from_forward_node || search_from_reverse_node);

        if (search_to_reverse_node || search_to_forward_node)
        {
            if (allow_uturn_at_waypoint)
            {
                searchWithUTurn(facade,
                                forward_heap,
                                reverse_heap,
                                forward_core_heap,
                                reverse_core_heap,
                                search_from_forward_node,
                                search_from_reverse_node,
                                search_to_forward_node,
                                search_to_reverse_node,
                                source_phantom,
                                target_phantom,
                                total_weight_to_forward,
                                total_weight_to_reverse,
                                new_total_weight_to_forward,
                                packed_leg_to_forward);
                // if only the reverse node is valid (e.g. when using the match plugin) we
                // actually need to move
                if (!target_phantom.forward_segment_id.enabled)
                {
                    BOOST_ASSERT(target_phantom.reverse_segment_id.enabled);
                    new_total_weight_to_reverse = new_total_weight_to_forward;
                    packed_leg_to_reverse = std::move(packed_leg_to_forward);
                    new_total_weight_to_forward = INVALID_EDGE_WEIGHT;
                }
                else if (target_phantom.reverse_segment_id.enabled)
                {
                    new_total_weight_to_reverse = new_total_weight_to_forward;
                    packed_leg_to_reverse = packed_leg_to_forward;
                }
            }
            else
            {
                search(facade,
                       forward_heap,
                       reverse_heap,
                       forward_core_heap,
                       reverse_core_heap,
                       search_from_forward_node,
                       search_from_reverse_node,
                       search_to_forward_node,
                       search_to_reverse_node,
                       source_phantom,
                       target_phantom,
                       total_weight_to_forward,
                       total_weight_to_reverse,
                       new_total_weight_to_forward,
                       new_total_weight_to_reverse,
                       packed_leg_to_forward,
                       packed_leg_to_reverse);
            }
        }

        // No path found for both target nodes?
        if ((INVALID_EDGE_WEIGHT == new_total_weight_to_forward) &&
            (INVALID_EDGE_WEIGHT == new_total_weight_to_reverse))
        {
            raw_route_data.shortest_path_length = INVALID_EDGE_WEIGHT;
            raw_route_data.alternative_path_length = INVALID_EDGE_WEIGHT;
            return raw_route_data;
        }

        // we need to figure out how the new legs connect to the previous ones
        if (current_leg > 0)
        {
            bool forward_to_forward =
                (new_total_weight_to_forward != INVALID_EDGE_WEIGHT) &&
                packed_leg_to_forward.front() == source_phantom.forward_segment_id.id;
            bool reverse_to_forward =
                (new_total_weight_to_forward != INVALID_EDGE_WEIGHT) &&
                packed_leg_to_forward.front() == source_phantom.reverse_segment_id.id;
            bool forward_to_reverse =
                (new_total_weight_to_reverse != INVALID_EDGE_WEIGHT) &&
                packed_leg_to_reverse.front() == source_phantom.forward_segment_id.id;
            bool reverse_to_reverse =
                (new_total_weight_to_reverse != INVALID_EDGE_WEIGHT) &&
                packed_leg_to_reverse.front() == source_phantom.reverse_segment_id.id;

            BOOST_ASSERT(!forward_to_forward || !reverse_to_forward);
            BOOST_ASSERT(!forward_to_reverse || !reverse_to_reverse);

            // in this case we always need to copy
            if (forward_to_forward && forward_to_reverse)
            {
                // in this case we copy the path leading to the source forward node
                // and change the case
                total_packed_path_to_reverse = total_packed_path_to_forward;
                packed_leg_to_reverse_begin = packed_leg_to_forward_begin;
                forward_to_reverse = false;
                reverse_to_reverse = true;
            }
            else if (reverse_to_forward && reverse_to_reverse)
            {
                total_packed_path_to_forward = total_packed_path_to_reverse;
                packed_leg_to_forward_begin = packed_leg_to_reverse_begin;
                reverse_to_forward = false;
                forward_to_forward = true;
            }
            BOOST_ASSERT(!forward_to_forward || !forward_to_reverse);
            BOOST_ASSERT(!reverse_to_forward || !reverse_to_reverse);

            // in this case we just need to swap to regain the correct mapping
            if (reverse_to_forward || forward_to_reverse)
            {
                total_packed_path_to_forward.swap(total_packed_path_to_reverse);
                packed_leg_to_forward_begin.swap(packed_leg_to_reverse_begin);
            }
        }

        if (new_total_weight_to_forward != INVALID_EDGE_WEIGHT)
        {
            BOOST_ASSERT(target_phantom.forward_segment_id.enabled);

            packed_leg_to_forward_begin.push_back(total_packed_path_to_forward.size());
            total_packed_path_to_forward.insert(total_packed_path_to_forward.end(),
                                                packed_leg_to_forward.begin(),
                                                packed_leg_to_forward.end());
            search_from_forward_node = true;
        }
        else
        {
            total_packed_path_to_forward.clear();
            packed_leg_to_forward_begin.clear();
            search_from_forward_node = false;
        }

        if (new_total_weight_to_reverse != INVALID_EDGE_WEIGHT)
        {
            BOOST_ASSERT(target_phantom.reverse_segment_id.enabled);

            packed_leg_to_reverse_begin.push_back(total_packed_path_to_reverse.size());
            total_packed_path_to_reverse.insert(total_packed_path_to_reverse.end(),
                                                packed_leg_to_reverse.begin(),
                                                packed_leg_to_reverse.end());
            search_from_reverse_node = true;
        }
        else
        {
            total_packed_path_to_reverse.clear();
            packed_leg_to_reverse_begin.clear();
            search_from_reverse_node = false;
        }

        prev_packed_leg_to_forward = std::move(packed_leg_to_forward);
        prev_packed_leg_to_reverse = std::move(packed_leg_to_reverse);

        total_weight_to_forward = new_total_weight_to_forward;
        total_weight_to_reverse = new_total_weight_to_reverse;

        ++current_leg;
    }

    BOOST_ASSERT(total_weight_to_forward != INVALID_EDGE_WEIGHT ||
                 total_weight_to_reverse != INVALID_EDGE_WEIGHT);

    // We make sure the fastest route is always in packed_legs_to_forward
    if (total_weight_to_forward > total_weight_to_reverse)
    {
        // insert sentinel
        packed_leg_to_reverse_begin.push_back(total_packed_path_to_reverse.size());
        BOOST_ASSERT(packed_leg_to_reverse_begin.size() == phantom_nodes_vector.size() + 1);

        unpackLegs(facade,
                   phantom_nodes_vector,
                   total_packed_path_to_reverse,
                   packed_leg_to_reverse_begin,
                   total_weight_to_reverse,
                   raw_route_data);
    }
    else
    {
        // insert sentinel
        packed_leg_to_forward_begin.push_back(total_packed_path_to_forward.size());
        BOOST_ASSERT(packed_leg_to_forward_begin.size() == phantom_nodes_vector.size() + 1);

        unpackLegs(facade,
                   phantom_nodes_vector,
                   total_packed_path_to_forward,
                   packed_leg_to_forward_begin,
                   total_weight_to_forward,
                   raw_route_data);
    }
    
    std::cout << "hello world!\n";
        auto number_of_routes = raw_route_data.has_alternative() ? 2UL : 1UL;
    std::cout << "number of routes" << number_of_routes << "\n";
    
        auto number_of_legs = raw_route_data.segment_end_coordinates.size();
    std::cout << "number of legs" << number_of_legs << "\n";
    
    for (auto idx : util::irange<std::size_t>(0UL, number_of_legs))
    {
        const auto &phantoms = raw_route_data.segment_end_coordinates[idx];
        const auto &path_data = raw_route_data.unpacked_path_segments[idx];

        //                          u       *      v
        //                          0 -- 1 -- 2 -- 3
        // fwd_segment_position:  1
        // source node fwd:       1      1 -> 2 -> 3
        // source node rev:       2 0 <- 1 <- 2

        // const auto source_segment_start_coordinate =
        //  source_node.fwd_segment_position + (reversed_source ? 1 : 0);
        // const std::vector<NodeID> source_geometry =
        //  facade.GetUncompressedForwardGeometry(source_node.packed_geometry_id);

        auto cumulative_distance = 0.;
        auto current_distance = 0.;
        auto prev_coordinate = phantoms.source_phantom.location;
        for (const auto &path_point : path_data)
        {
            auto coordinate = facade.GetCoordinateOfNode(path_point.turn_via_node);
            auto eta = path_point.duration_until_turn;      
            current_distance =
                util::coordinate_calculation::haversineDistance(prev_coordinate, coordinate);
            cumulative_distance += current_distance;
            prev_coordinate = coordinate;
            std::cout << "edge id: " << path_point.edge_id <<
                " name: " << path_point.name_id <<
                " eta: " << eta <<
                " current distance: " << current_distance << "\n";      
        }
        current_distance =
        util::coordinate_calculation::haversineDistance(prev_coordinate, phantoms.target_phantom.location);
        cumulative_distance += current_distance;
    }
    
    return raw_route_data;
}
}

InternalRouteResult
trafficPrediction(SearchEngineData &engine_working_data,
                   const datafacade::ContiguousInternalMemoryDataFacade<algorithm::CH> &facade,
                   const std::vector<PhantomNodes> &phantom_nodes_vector,
                   const boost::optional<bool> continue_straight_at_waypoint)
{
    return trafficPredictionImpl(
        engine_working_data, facade, phantom_nodes_vector, continue_straight_at_waypoint);
}

InternalRouteResult
trafficPrediction(SearchEngineData &engine_working_data,
                   const datafacade::ContiguousInternalMemoryDataFacade<algorithm::CoreCH> &facade,
                   const std::vector<PhantomNodes> &phantom_nodes_vector,
                   const boost::optional<bool> continue_straight_at_waypoint)
{
    return trafficPredictionImpl(
        engine_working_data, facade, phantom_nodes_vector, continue_straight_at_waypoint);
}

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm
