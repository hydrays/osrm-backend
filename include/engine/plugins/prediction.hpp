#ifndef PREDICTION_HPP
#define PREDICTION_HPP

#include "engine/api/route_api.hpp"
#include "engine/datafacade/datafacade_base.hpp"
#include "engine/plugins/plugin_base.hpp"

#include "engine/traffic_prediction/traffic_prediction.hpp"
#include "engine/search_engine_data.hpp"
#include "util/json_container.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace osrm
{
namespace engine
{
namespace plugins
{

class PredictionPlugin final : public BasePlugin
{
  private:
    mutable SearchEngineData heaps;
    mutable traffic_prediction::TrafficPrediction<datafacade::BaseDataFacade> traffic_prediction;
    const int max_locations_viaroute;

  public:
    explicit PredictionPlugin(int max_locations_viaroute);

    Status HandleRequest(const std::shared_ptr<datafacade::BaseDataFacade> facade,
                         const api::RouteParameters &route_parameters,
                         util::json::Object &json_result) const;
};
}
}
}

#endif // PREDICTION_HPP
