#ifndef OSRM_GUIDANCE_TOOLKIT_HPP_
#define OSRM_GUIDANCE_TOOLKIT_HPP_

#include "util/attributes.hpp"
#include "util/bearing.hpp"
#include "util/coordinate.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/guidance/toolkit.hpp"
#include "util/guidance/turn_lanes.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

#include "extractor/compressed_edge_container.hpp"
#include "extractor/query_node.hpp"

#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/discrete_angle.hpp"
#include "extractor/guidance/intersection.hpp"
#include "extractor/guidance/road_classification.hpp"
#include "extractor/guidance/turn_instruction.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>

#include <boost/algorithm/string.hpp>
#include <boost/functional/hash.hpp>
#include <boost/tokenizer.hpp>

namespace osrm
{
namespace extractor
{
namespace guidance
{

using util::guidance::LaneTupelIdPair;
using LaneDataIdMap = std::unordered_map<LaneTupelIdPair, LaneDataID, boost::hash<LaneTupelIdPair>>;

using util::guidance::angularDeviation;
using util::guidance::entersRoundabout;
using util::guidance::leavesRoundabout;

// To simplify handling of Left/Right hand turns, we can mirror turns and write an intersection
// handler only for one side. The mirror function turns a left-hand turn in a equivalent right-hand
// turn and vice versa.
OSRM_ATTR_WARN_UNUSED
inline ConnectedRoad mirror(ConnectedRoad road)
{
    const constexpr DirectionModifier::Enum mirrored_modifiers[] = {DirectionModifier::UTurn,
                                                                    DirectionModifier::SharpLeft,
                                                                    DirectionModifier::Left,
                                                                    DirectionModifier::SlightLeft,
                                                                    DirectionModifier::Straight,
                                                                    DirectionModifier::SlightRight,
                                                                    DirectionModifier::Right,
                                                                    DirectionModifier::SharpRight};

    if (angularDeviation(road.turn.angle, 0) > std::numeric_limits<double>::epsilon())
    {
        road.turn.angle = 360 - road.turn.angle;
        road.turn.instruction.direction_modifier =
            mirrored_modifiers[road.turn.instruction.direction_modifier];
    }
    return road;
}

inline bool hasRoundaboutType(const TurnInstruction instruction)
{
    using namespace extractor::guidance::TurnType;
    const constexpr TurnType::Enum valid_types[] = {TurnType::EnterRoundabout,
                                                    TurnType::EnterAndExitRoundabout,
                                                    TurnType::EnterRotary,
                                                    TurnType::EnterAndExitRotary,
                                                    TurnType::EnterRoundaboutIntersection,
                                                    TurnType::EnterAndExitRoundaboutIntersection,
                                                    TurnType::EnterRoundaboutAtExit,
                                                    TurnType::ExitRoundabout,
                                                    TurnType::EnterRotaryAtExit,
                                                    TurnType::ExitRotary,
                                                    TurnType::EnterRoundaboutIntersectionAtExit,
                                                    TurnType::ExitRoundaboutIntersection,
                                                    TurnType::StayOnRoundabout};

    const auto *first = valid_types;
    const auto *last = first + sizeof(valid_types) / sizeof(valid_types[0]);

    return std::find(first, last, instruction.type) != last;
}

// Public service vehicle lanes and similar can introduce additional lanes into the lane string that
// are not specifically marked for left/right turns. This function can be used from the profile to
// trim the lane string appropriately
//
// left|throught|
// in combination with lanes:psv:forward=1
// will be corrected to left|throught, since the final lane is not drivable.
// This is in contrast to a situation with lanes:psv:forward=0 (or not set) where left|through|
// represents left|through|through
OSRM_ATTR_WARN_UNUSED
inline std::string
trimLaneString(std::string lane_string, std::int32_t count_left, std::int32_t count_right)
{
    if (count_left)
    {
        bool sane = count_left < static_cast<std::int32_t>(lane_string.size());
        for (std::int32_t i = 0; i < count_left; ++i)
            // this is adjusted for our fake pipe. The moment cucumber can handle multiple escaped
            // pipes, the '&' part can be removed
            if (lane_string[i] != '|')
            {
                sane = false;
                break;
            }

        if (sane)
        {
            lane_string.erase(lane_string.begin(), lane_string.begin() + count_left);
        }
    }
    if (count_right)
    {
        bool sane = count_right < static_cast<std::int32_t>(lane_string.size());
        for (auto itr = lane_string.rbegin();
             itr != lane_string.rend() && itr != lane_string.rbegin() + count_right;
             ++itr)
        {
            if (*itr != '|')
            {
                sane = false;
                break;
            }
        }
        if (sane)
            lane_string.resize(lane_string.size() - count_right);
    }
    return lane_string;
}

// https://github.com/Project-OSRM/osrm-backend/issues/2638
// It can happen that some lanes are not drivable by car. Here we handle this tagging scheme
// (vehicle:lanes) to filter out not-allowed roads
// lanes=3
// turn:lanes=left|through|through|right
// vehicle:lanes=yes|yes|no|yes
// bicycle:lanes=yes|no|designated|yes
OSRM_ATTR_WARN_UNUSED
inline std::string applyAccessTokens(std::string lane_string, const std::string &access_tokens)
{
    typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
    boost::char_separator<char> sep("|", "", boost::keep_empty_tokens);
    tokenizer tokens(lane_string, sep);
    tokenizer access(access_tokens, sep);

    // strings don't match, don't do anything
    if (std::distance(std::begin(tokens), std::end(tokens)) !=
        std::distance(std::begin(access), std::end(access)))
        return lane_string;

    std::string result_string = "";
    const static std::string yes = "yes";

    for (auto token_itr = std::begin(tokens), access_itr = std::begin(access);
         token_itr != std::end(tokens);
         ++token_itr, ++access_itr)
    {
        if (*access_itr == yes)
        {
            // we have to add this in front, because the next token could be invalid. Doing this on
            // non-empty strings makes sure that the token string will be valid in the end
            if (!result_string.empty())
                result_string += '|';

            result_string += *token_itr;
        }
    }
    return result_string;
}

inline bool obviousByRoadClass(const RoadClassification in_classification,
                               const RoadClassification obvious_candidate,
                               const RoadClassification compare_candidate)
{
    const bool has_high_priority = PRIORITY_DISTINCTION_FACTOR * obvious_candidate.GetPriority() <
                                   compare_candidate.GetPriority();

    const bool continues_on_same_class = in_classification == obvious_candidate;
    return (has_high_priority && continues_on_same_class) ||
           (!obvious_candidate.IsLowPriorityRoadClass() &&
            !in_classification.IsLowPriorityRoadClass() &&
            compare_candidate.IsLowPriorityRoadClass());
}

inline std::uint8_t getLaneCountAtIntersection(const NodeID intersection_node,
                                        const util::NodeBasedDynamicGraph &node_based_graph)
{
    std::uint8_t lanes = 0;
    for (const EdgeID onto_edge : node_based_graph.GetAdjacentEdgeRange(intersection_node))
        lanes = std::max(
            lanes, node_based_graph.GetEdgeData(onto_edge).road_classification.GetNumberOfLanes());
    return lanes;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif // OSRM_GUIDANCE_TOOLKIT_HPP_
