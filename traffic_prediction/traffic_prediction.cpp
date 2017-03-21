#include "osrm/match_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <utility>

#include <cstdlib>

struct MyCoordinate{
    double lng, lat;
}coord[100];

int main(int argc, const char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " data.osrm\n";
        return EXIT_FAILURE;
    }

    using namespace osrm;

    // Configure based on a .osrm base path, and no datasets in shared mem from osrm-datastore
    EngineConfig config;
    config.storage_config = {argv[1]};

    config.use_shared_memory = false;
    
    // Routing machine with several services (such as Route, Table, Nearest, Trip, Match)
    const OSRM osrm{config};

    // The following shows how to use the Route service; configure this service
    RouteParameters params;

    MyCoordinate coord[] = {
    {116.232933,39.8361359},
    {116.2321854,39.8359718},
    {116.2313156,39.8335724},
    {116.2340164,39.8317986},
    {116.2337799,39.8315735},
    {116.226181,39.8370972},
    {116.2181473,39.8449097},
    {116.2144012,39.8537102},
    {116.2091141,39.8630371},
    {116.2029495,39.8719635},
    {116.2027817,39.8819351},
    {116.2053833,39.89217},
    {116.2052002,39.9015961},
    {116.2051697,39.9074783},
    {116.2051315,39.9108009},
    {116.205101,39.9133682},
    {116.2058182,39.9178467},
    {116.2062149,39.9251366},
    {116.205986,39.9337921},
    {116.2063675,39.9429054},
    {116.2098694,39.9504433},
    {116.2136002,39.9537125},
    {116.2152328,39.9537773},
    {116.2121353,39.9540558},
    {116.2050629,39.9572449},
    {116.2015305,39.9606056},
    {116.2012634,39.9639244},
    {116.2016296,39.9675407},
    {116.2013702,39.9730377},
    {116.200882,39.9775352},
    {116.2009506,39.9815865},
    {116.203949,39.9812126}};
    // Route in monaco
    //params.coordinates.push_back({util::FloatLongitude{116.29758}, util::FloatLatitude{39.831142}});
    //params.coordinates.push_back({util::FloatLongitude{116.29505}, util::FloatLatitude{39.836825}});

    for (MyCoordinate coor: coord){
        params.coordinates.push_back({util::FloatLongitude{coor.lng}, util::FloatLatitude{coor.lat}});
    }

    // Response is in JSON format
    json::Object result;
    // Execute routing request, this does the heavy lifting
    const auto status = osrm.Prediction(params, result);

    if (status == Status::Ok)
    {
        auto &routes = result.values["routes"].get<json::Array>();

        // Let's just use the first route
        auto &route = routes.values.at(0).get<json::Object>();
        const auto distance = route.values["distance"].get<json::Number>().value;
        const auto duration = route.values["duration"].get<json::Number>().value;

        // Warn users if extract does not contain the default Berlin coordinates from above
        if (distance == 0 or duration == 0)
        {
            std::cout << "Note: distance or duration is zero. ";
            std::cout << "You are probably doing a query outside of the OSM extract.\n\n";
        }

        std::cout << "Distance: " << distance << " meter\n";
        std::cout << "Duration: " << duration << " seconds\n";
        return EXIT_SUCCESS;
    }
    else if (status == Status::Error)
    {
        const auto code = result.values["code"].get<json::String>().value;
        const auto message = result.values["message"].get<json::String>().value;

        std::cout << "Code: " << code << "\n";
        std::cout << "Message: " << code << "\n";
        return EXIT_FAILURE;
    }
}
