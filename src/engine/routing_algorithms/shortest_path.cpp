#include "engine/routing_algorithms/shortest_path.hpp"
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
shortestPathSearchImpl(SearchEngineData &engine_working_data,
                       const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT> &facade,
                       const std::vector<PhantomNodes> &phantom_nodes_vector,
                       const boost::optional<bool> continue_straight_at_waypoint)
{
    std::vector<int> id_list={45818,79831,45808,45811,131057,131070,9810,45747,188306,188315,9813,9814,45638,129398,9824,
        45632,9825,26660,26658,45566,92383,129408,129411,129413,9828,129435,129441,9829,129442,31886,9835,9836,9837,129452,
        9838,157485,43080,101676,101674,45526,45510,131245,45539,105577,105574,105596,120381,157489,120366,146484,105615,
        156540,146363,146395,156602,146454,146462,201164,43084,201741,45465,201045,132790};
    
    std::cout << facade.GetNumberOfNodes() << std::endl;  //128362个节点
    std::cout << facade.GetNumberOfEdges() << std::endl;  //695972条边

    for (int i = 0; i < id_list.size(); i++) {
        auto edge_data = facade.GetEdgeData(id_list[i]);
        std::cout << edge_data.id << std::endl;
        std::cout << facade.GetTarget(id_list[i]) << std::endl;
        std::cout << "------------------" << std::endl;
    }

    std::vector<int> node_id_list = {8164,21546,8327,8327,22475,55410,48780,8143,6758,32509,48772,64945,7978,22200,16085,28681,
        16090,59684,17868,8220,29166,22217,22219,22217,16085,22231,22218,16089,22237,5791,16089,52178,56948,22240,16085,2879,
        49144,17491,17467,8318,8288,22524,7999,21802,49278,19910,20825,69905,59545,25354,18165,27049,21792,104702,2646,25358,
        21836,35320,49148,35432,29148,34303,61908};

    std::vector<int> via_node_list = {31971,31972,31973,31974,52387,31975,31976,31977,31978,31979,31980,31960,31961,31962,31963,
        31964,7427,7428,7429,7433,99480,99486,99490,7434,15392,99658,23028,99667,7435,7436,99673,99677,23011,7437,7438,7439,7448,
        31840,7449,174745,174747,148975,7450,52989,7451,149013,7452,7453,7459,7460,7461,7462,7463,7464,7465,31748,96330,7466,7485,
        96415,7486,31744,7487,18812,18810,31699,96493,7490,60624,96496,96498,96504,7491,96626,96647,7492,96661,7498,7499,20644};

    /*for(int i = 0; i < node_id_list.size()-1; i++)
    {
        int source = node_id_list[i];
        std::cout << source << std::endl;
        bool flag = false;
        auto edge_range = facade.GetAdjacentEdgeRange(source);

        for(auto current_edge : edge_range)
        {
            const NodeID target = facade.GetTarget(current_edge);
            if (target == node_id_list[i+1])
            {
                flag = true;
                break;
            }
        }

        if(!flag)
        {
            source = node_id_list[i+1];
            auto edge_range = facade.GetAdjacentEdgeRange(source);
            for(auto current_edge : edge_range)
            {
                const NodeID target = facade.GetTarget(current_edge);
                if (target == node_id_list[i])
                {
                    flag = true;
                    break;
                }
            }
        }
        if(!flag)
        {
            std::cout << "Not connected!" << std::endl;
        }else
        {
            std::cout << "Connected!" << std::endl;
        }
    }*/


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

    return raw_route_data;
}
}

InternalRouteResult
shortestPathSearch(SearchEngineData &engine_working_data,
                   const datafacade::ContiguousInternalMemoryDataFacade<algorithm::CH> &facade,
                   const std::vector<PhantomNodes> &phantom_nodes_vector,
                   const boost::optional<bool> continue_straight_at_waypoint)
{
    return shortestPathSearchImpl(
        engine_working_data, facade, phantom_nodes_vector, continue_straight_at_waypoint);
}

InternalRouteResult
shortestPathSearch(SearchEngineData &engine_working_data,
                   const datafacade::ContiguousInternalMemoryDataFacade<algorithm::CoreCH> &facade,
                   const std::vector<PhantomNodes> &phantom_nodes_vector,
                   const boost::optional<bool> continue_straight_at_waypoint)
{
    return shortestPathSearchImpl(
        engine_working_data, facade, phantom_nodes_vector, continue_straight_at_waypoint);
}

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm
