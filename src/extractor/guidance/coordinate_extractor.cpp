#include "extractor/guidance/coordinate_extractor.hpp"
#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/toolkit.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <numeric>
#include <utility>

namespace osrm
{
namespace extractor
{
namespace guidance
{

namespace
{
// to use the corrected coordinate, we require it to be at least a bit further down the
// road than the offset coordinate. We postulate a minimum Distance of 2 Meters
const constexpr double DESIRED_COORDINATE_DIFFERENCE = 2.0;
// the default distance we lookahead on a road. This distance prevents small mapping
// errors to impact the turn angles.
const constexpr double LOOKAHEAD_DISTANCE_WITHOUT_LANES = 10.0;
// The standard with of a interstate highway is 3.7 meters. Local roads have
// smaller widths, ranging from 2.5 to 3.25 meters. As a compromise, we use
// the 3.25 here for our angle calculations
const constexpr double ASSUMED_LANE_WIDTH = 3.25;
const constexpr double FAR_LOOKAHEAD_DISTANCE = 30.0;

// The count of lanes assumed when no lanes are present. Since most roads will have lanes for both
// directions or a lane count specified, we use 2. Overestimating only makes our calculations safer,
// so we are fine for 1-lane ways. larger than 2 lanes should usually be specified in the data.
const constexpr std::uint16_t ASSUMED_LANE_COUNT = 2;
}

CoordinateExtractor::CoordinateExtractor(
    const util::NodeBasedDynamicGraph &node_based_graph,
    const extractor::CompressedEdgeContainer &compressed_geometries,
    const std::vector<extractor::QueryNode> &node_coordinates)
    : node_based_graph(node_based_graph), compressed_geometries(compressed_geometries),
      node_coordinates(node_coordinates)
{
    times_called = new std::size_t;
    times_failed = new std::size_t;
    *times_called = 0;
    *times_failed = 0;
}

CoordinateExtractor::~CoordinateExtractor()
{
    std::cout << "Handled: " << (*times_called - *times_failed) << " of " << *times_called
              << " Angle Cases(" << (100.0 * (*times_called - *times_failed) / *times_called)
              << "%)" << std::endl;
    delete times_called;
    delete times_failed;
}

util::Coordinate
CoordinateExtractor::GetCoordinateAlongRoad(const NodeID intersection_node,
                                            const EdgeID turn_edge,
                                            const bool traversed_in_reverse,
                                            const NodeID to_node,
                                            const std::uint8_t intersection_lanes) const
{
    const auto considered_lanes =
        (intersection_lanes == 0) ? ASSUMED_LANE_COUNT : intersection_lanes;

    ++(*times_called);
    // we first extract all coordinates from the road
    auto coordinates =
        GetCoordinatesAlongRoad(intersection_node, turn_edge, traversed_in_reverse, to_node);

    /* if we are looking at a straight line, we don't care where exactly the coordinate
     * is. Simply return the final coordinate. Turn angles/turn vectors are the same no matter which
     * coordinate we look at.
     */
    if (coordinates.size() <= 2)
        return coordinates.back();

    // fallback, mostly necessary for dead ends
    if (intersection_node == to_node)
        return TrimCoordinatesToLength(coordinates, 5).back();

    const auto lookahead_distance =
        FAR_LOOKAHEAD_DISTANCE + considered_lanes * ASSUMED_LANE_WIDTH * 0.5;

    // reduce coordinates to the ones we care about
    coordinates = TrimCoordinatesToLength(std::move(coordinates), lookahead_distance);

    // If this reduction leaves us with only two coordinates, the turns/angles are represented in a
    // valid way. Only curved roads and other difficult scenarios will require multiple coordinates.
    if (coordinates.size() == 2)
        return coordinates.back();

    const auto &turn_edge_data = node_based_graph.GetEdgeData(turn_edge);
    const util::Coordinate turn_coordinate =
        node_coordinates[traversed_in_reverse ? to_node : intersection_node];

    // Low priority roads are usually modelled very strangely. The roads are so small, though, that
    // our basic heuristic looking at the road should be fine.
    if (turn_edge_data.road_classification.IsLowPriorityRoadClass())
    {
        // Look ahead a tiny bit. Low priority road classes can be modelled fairly distinct in the
        // very first part of the road
        coordinates = TrimCoordinatesToLength(std::move(coordinates), 10);
        if (coordinates.size() > 2 &&
            util::coordinate_calculation::haversineDistance(turn_coordinate, coordinates[1]) <
                ASSUMED_LANE_WIDTH)
            return GetCorrectedCoordinate(turn_coordinate, coordinates[1], coordinates.back());
        else
            return coordinates.back();
    }

    /*
     * The coordinates along the road are in different distances from the source. If only very few
     * coordinates are close to the intersection, It might just be we simply looked to far down the
     * road. We can decide to weight coordinates differently based on their distance from the
     * intersection.
     * In addition, changes very close to an intersection indicate graphical representation of the
     * intersection over perceived turn angles.
     *
     * a -
     *    \
     *     -------------------- b
     *
     * Here the initial angle close to a might simply be due to OSM-Ways being located in the middle
     * of the actual roads. If a road splits in two, the ways for the separate direction can be
     * modeled very far apart with a steep angle at the split, even though the roads actually don't
     * take a turn. The distance between the coordinates can be an indicator for these small changes
     */
    const auto segment_distances = [&coordinates]() {
        std::vector<double> segment_distances;
        segment_distances.reserve(coordinates.size());
        segment_distances.push_back(0);

        for (std::size_t i = 1; i < coordinates.size(); ++i)
            segment_distances.push_back(util::coordinate_calculation::haversineDistance(
                coordinates[i - 1], coordinates[i]));
        return segment_distances;
    }();

    /* if the very first coordinate along the road is reasonably far away from the road, we assume
     * the coordinate to correctly represent the turn. This could probably be improved using
     * information on the very first turn angle (requires knowledge about previous road) and the
     * respective lane widths.
     */
    const bool first_coordinate_is_far_away = [&segment_distances, considered_lanes]() {
        const auto required_distance =
            considered_lanes * 0.5 * ASSUMED_LANE_WIDTH + LOOKAHEAD_DISTANCE_WITHOUT_LANES;
        return segment_distances[1] > required_distance;
    }();

    if (first_coordinate_is_far_away)
    {
        return coordinates[1];
    }

    const double max_deviation_from_straight = GetMaxDeviation(
        coordinates.begin(), coordinates.end(), coordinates.front(), coordinates.back());

    // if the deviation from a straight line is small, we can savely use the coordinate. We use half
    // a lane as heuristic to determine if the road is straight enough.
    if (max_deviation_from_straight < 0.5 * ASSUMED_LANE_WIDTH)
    {
        return coordinates.back();
    }

    /*
     * if a road turns barely in the beginning, it is similar to the first coordinate being
     * sufficiently far ahead.
     * possible negative:
     * http://www.openstreetmap.org/search?query=52.514503%2013.32252#map=19/52.51450/13.32252
     */
    const auto straight_distance_and_index = [&]() {
        auto straight_distance = segment_distances[1];

        std::size_t index;
        for (index = 2; index < coordinates.size(); ++index)
        {
            // check the deviation from a straight line
            if (GetMaxDeviation(coordinates.begin(),
                                coordinates.begin() + index,
                                coordinates.front(),
                                *(coordinates.begin() + index)) < 0.25 * ASSUMED_LANE_WIDTH)
                straight_distance += segment_distances[index];
            else
                break;
        }
        return std::make_pair(index - 1, straight_distance);
    }();
    const auto straight_distance = straight_distance_and_index.second;
    const auto straight_index = straight_distance_and_index.first;

    const bool starts_of_without_turn = [&]() {
        return straight_distance >=
               considered_lanes * 0.5 * ASSUMED_LANE_WIDTH + LOOKAHEAD_DISTANCE_WITHOUT_LANES;
    }();
    if (starts_of_without_turn)
    {
        // skip over repeated coordinates
        return TrimCoordinatesToLength(std::move(coordinates), 5).back();
    }

    // compute the regression vector based on the sum of least squares
    const auto regression_line = RegressionLine(coordinates);

    /*
     * If we can find a line that represents the full set of coordinates within a certain range in
     * relation to ASSUMED_LANE_WIDTH, we use the regression line to express the turn angle.
     * This yields a transformation similar to:
     *
     *         c   d                       d
     *    b              ->             c
     *                               b
     * a                          a
     */
    const double max_deviation_from_regression = GetMaxDeviation(
        coordinates.begin(), coordinates.end(), regression_line.first, regression_line.second);

    if (max_deviation_from_regression < 0.35 * ASSUMED_LANE_WIDTH)
    {
        // We use the locations on the regression line to offset the regression line onto the
        // intersection.
        const auto coord_between_front =
            util::coordinate_calculation::projectPointOnSegment(
                regression_line.first, regression_line.second, coordinates.front())
                .second;
        const auto coord_between_back =
            util::coordinate_calculation::projectPointOnSegment(
                regression_line.first, regression_line.second, coordinates.back())
                .second;
        return GetCorrectedCoordinate(turn_coordinate, coord_between_front, coord_between_back);
    }

    const auto total_distance =
        std::accumulate(segment_distances.begin(), segment_distances.end(), 0.);

    if (IsDirectOffset(coordinates,
                       straight_index,
                       straight_distance,
                       total_distance,
                       segment_distances,
                       considered_lanes))
    {
        // could be too agressive? Depend on lanes to check how far we want to go out?
        // compare
        // http://www.openstreetmap.org/search?query=52.411243%2013.363575#map=19/52.41124/13.36357
        const auto offset_index = std::max<decltype(straight_index)>(1, straight_index);
        return GetCorrectedCoordinate(
            turn_coordinate, coordinates[offset_index], coordinates[offset_index + 1]);
    }

    if (IsCurve(coordinates,
                segment_distances,
                total_distance,
                considered_lanes * 0.5 * ASSUMED_LANE_WIDTH,
                turn_edge_data))
    {
        /*
         * In curves we now have to distinguish between larger curves and tiny curves modelling the
         * actual turn in the beginnig.
         *
         * We distinguish between turns that simply model the initial way of getting onto the
         * destination lanes and the ones that performa a larger turn.
         */
        const double offset = 0.5 * considered_lanes * ASSUMED_LANE_WIDTH;
        coordinates = TrimCoordinatesToLength(std::move(coordinates), offset);
        const auto vector_head = coordinates.back();
        coordinates = TrimCoordinatesToLength(std::move(coordinates), offset);
        BOOST_ASSERT(coordinates.size() >= 2);
        return GetCorrectedCoordinate(turn_coordinate, coordinates.back(), vector_head);
    }

    {
        // skip over the first coordinates, in specific the assumed lane count. We add a small
        // safety factor, to not overshoot on the regression
        const auto trimmed_coordinates = TrimCoordinatesByLengthFront(
            coordinates, 0.8 * (considered_lanes * ASSUMED_LANE_WIDTH));
        if (trimmed_coordinates.size() >= 2)
        {
            // get the regression line
            const auto regression_line_trimmed = RegressionLine(trimmed_coordinates);

            // and compute the maximum deviation from it
            const auto max_deviation_from_trimmed_regression =
                GetMaxDeviation(trimmed_coordinates.begin(),
                                trimmed_coordinates.end(),
                                regression_line_trimmed.first,
                                regression_line_trimmed.second);

            if (max_deviation_from_trimmed_regression < 0.5 * ASSUMED_LANE_WIDTH)
                return GetCorrectedCoordinate(
                    turn_coordinate, regression_line_trimmed.first, regression_line_trimmed.second);
        }
    }

    // We use the locations on the regression line to offset the regression line onto the
    // intersection.
    return TrimCoordinatesToLength(coordinates, LOOKAHEAD_DISTANCE_WITHOUT_LANES).back();
}

std::vector<util::Coordinate>
CoordinateExtractor::GetCoordinatesAlongRoad(const NodeID intersection_node,
                                             const EdgeID turn_edge,
                                             const bool traversed_in_reverse,
                                             const NodeID to_node) const
{
    if (!compressed_geometries.HasEntryForID(turn_edge))
    {
        if (traversed_in_reverse)
            return {{node_coordinates[to_node]}, {node_coordinates[intersection_node]}};
        else
            return {{node_coordinates[intersection_node]}, {node_coordinates[to_node]}};
    }
    else
    {
        // extracts the geometry in coordinates from the compressed edge container
        std::vector<util::Coordinate> result;
        const auto &geometry = compressed_geometries.GetBucketReference(turn_edge);
        result.reserve(geometry.size() + 2);

        // the compressed edges contain node ids, we transfer them to coordinates accessing the
        // node_coordinates array
        const auto compressedGeometryToCoordinate = [this](
            const CompressedEdgeContainer::OnewayCompressedEdge &compressed_edge) -> util::Coordinate {
            return node_coordinates[compressed_edge.node_id];
        };

        // add the coordinates to the result in either normal or reversed order, based on
        // traversed_in_reverse
        if (traversed_in_reverse)
        {
            std::transform(geometry.rbegin(),
                           geometry.rend(),
                           std::back_inserter(result),
                           compressedGeometryToCoordinate);
            result.push_back(node_coordinates[intersection_node]);
        }
        else
        {
            result.push_back(node_coordinates[intersection_node]);
            std::transform(geometry.begin(),
                           geometry.end(),
                           std::back_inserter(result),
                           compressedGeometryToCoordinate);
        }
        return result;
    }
}

double
CoordinateExtractor::GetMaxDeviation(std::vector<util::Coordinate>::const_iterator range_begin,
                                     const std::vector<util::Coordinate>::const_iterator &range_end,
                                     const util::Coordinate &straight_begin,
                                     const util::Coordinate &straight_end) const
{
    double deviation = 0;
    for (; range_begin != range_end; range_begin = std::next(range_begin))
    {
        // find the projected coordinate
        auto coord_between = util::coordinate_calculation::projectPointOnSegment(
                                 straight_begin, straight_end, *range_begin)
                                 .second;
        // and calculate the distance between the intermediate coordinate and the coordinate
        // on the osrm-way
        deviation =
            std::max(deviation,
                     util::coordinate_calculation::haversineDistance(coord_between, *range_begin));
    }
    return deviation;
};

bool CoordinateExtractor::IsCurve(const std::vector<util::Coordinate> &coordinates,
                                  const std::vector<double> &segment_distances,
                                  const double segment_length,
                                  const double considered_lane_width,
                                  const util::NodeBasedEdgeData &edge_data) const
{
    BOOST_ASSERT(coordinates.size() > 2);

    // by default, we treat roundabout as curves
    if (edge_data.roundabout)
        return true;

    // TODO we might have to fix this to better compensate for errors due to repeated coordinates
    const bool takes_an_actual_turn = [&coordinates]() {
        const auto begin_bearing =
            util::coordinate_calculation::bearing(coordinates[0], coordinates[1]);
        const auto end_bearing = util::coordinate_calculation::bearing(
            coordinates[coordinates.size() - 2], coordinates[coordinates.size() - 1]);

        const auto total_angle = angularDeviation(begin_bearing, end_bearing);
        return total_angle > 0.5 * NARROW_TURN_ANGLE;
    }();

    if (!takes_an_actual_turn)
        return false;

    const auto get_deviation = [](const util::Coordinate &line_start,
                                  const util::Coordinate &line_end,
                                  const util::Coordinate &point) {
        // find the projected coordinate
        auto coord_between =
            util::coordinate_calculation::projectPointOnSegment(line_start, line_end, point).second;
        // and calculate the distance between the intermediate coordinate and the coordinate
        return util::coordinate_calculation::haversineDistance(coord_between, point);
    };

    // a curve needs to be on one side of the coordinate array
    const bool all_same_side = [&]() {
        if (coordinates.size() <= 3)
            return true;

        const bool ccw = util::coordinate_calculation::isCCW(
            coordinates.front(), coordinates.back(), coordinates[1]);

        for (std::size_t coordinate_index = 2; coordinate_index + 1 < coordinates.size();
             ++coordinate_index)
        {
            const bool compare_ccw = util::coordinate_calculation::isCCW(
                coordinates.front(), coordinates.back(), coordinates[coordinate_index]);
            if (compare_ccw != ccw)
                return false;
        }
        return true;
    }();

    if (!all_same_side)
        return false;

    // check if the deviation is a sequence that increases up to a maximum deviation and decreses
    // after, following what we would expect from a modelled curve
    std::size_t location_of_max_deviation = 0;
    double maximum_deviation = 0;
    const bool has_up_down_deviation = [&coordinates, get_deviation](std::size_t &maximum_index,
                                                                     double &maximum_deviation) {
        double last_deviation = 0;
        std::size_t current_coordinate_index = 1;

        // proceed to the maximum deviation
        for (; current_coordinate_index < coordinates.size(); ++current_coordinate_index)
        {
            const auto current_deviation = get_deviation(
                coordinates.front(), coordinates.back(), coordinates[current_coordinate_index]);
            if (current_deviation >= last_deviation)
            {
                last_deviation = current_deviation;
                maximum_index = current_coordinate_index;
                maximum_deviation = current_deviation;
            }
            else
            {
                ++current_coordinate_index;
                last_deviation = current_deviation;
                break;
            }
        }

        // check if we are only descending in deviation now
        for (; current_coordinate_index < coordinates.size(); ++current_coordinate_index)
        {
            const auto current_deviation = get_deviation(
                coordinates.front(), coordinates.back(), coordinates[current_coordinate_index]);

            if (current_deviation > last_deviation)
                return false;

            last_deviation = current_deviation;
        }
        return true;
    }(location_of_max_deviation, maximum_deviation);

    // if the maximum deviation is at a quarter of the total curve, we are probably looking at a
    // normal turn
    const auto distance_to_max_deviation = std::accumulate(
        segment_distances.begin(), segment_distances.begin() + location_of_max_deviation, 0);
    if ((distance_to_max_deviation < 0.25 * segment_length ||
         maximum_deviation < std::max(0.2 * considered_lane_width, 0.5 * ASSUMED_LANE_WIDTH)) &&
        segment_length > 10)
    {
        return false;
    }

    // a curve has increasing deviation from its front/back vertices to a certain point and after it
    // only decreases
    if (!has_up_down_deviation)
    {
        return false;
    }

    BOOST_ASSERT(coordinates.size() >= 3);
    // Compute all turn angles along the road
    const auto turn_angles = [coordinates]() {
        std::vector<double> turn_angles;
        turn_angles.reserve(coordinates.size() - 2);
        for (std::size_t index = 0; index + 2 < coordinates.size(); ++index)
        {
            turn_angles.push_back(util::coordinate_calculation::computeAngle(
                coordinates[index], coordinates[index + 1], coordinates[index + 2]));
        }
        return turn_angles;
    }();

    const bool curve_is_valid = [&turn_angles, &segment_distances, &segment_length]() {
        bool last_was_straight = false;
        double straight_distance = segment_distances[1];
        for (std::size_t i = 1; i < turn_angles.size(); ++i)
        {
            if (angularDeviation(turn_angles[i - 1], turn_angles[i]) > FUZZY_ANGLE_DIFFERENCE &&
                angularDeviation(turn_angles[i], turn_angles[i]) > FUZZY_ANGLE_DIFFERENCE &&
                (turn_angles[i] > STRAIGHT_ANGLE == turn_angles[i - 1] < STRAIGHT_ANGLE))
                return false;

            if (angularDeviation(turn_angles[i], STRAIGHT_ANGLE) < 5)
            {
                straight_distance += segment_distances[i];
                if (last_was_straight && straight_distance > 0.3 * segment_length)
                    return false;
                else
                    last_was_straight = true;
            }
            else
            {
                last_was_straight = false;
                straight_distance = segment_distances[i];
            }
        }
        return true;
    }();

    return (segment_length > 2 * considered_lane_width && curve_is_valid);
}

bool CoordinateExtractor::IsDirectOffset(const std::vector<util::Coordinate> &coordinates,
                                         const std::size_t straight_index,
                                         const double straight_distance,
                                         const double segment_length,
                                         const std::vector<double> &segment_distances,
                                         const std::uint8_t considered_lanes) const
{
    // check if a given length is with half a lane of the assumed lane offset
    const auto IsCloseToLaneDistance = [considered_lanes](const double width) {
        // a road usually is connected to the middle of the lanes. So the lane-offset has to
        // consider half to road
        const auto lane_offset = 0.5 * considered_lanes * ASSUMED_LANE_WIDTH;
        return std::abs(width - lane_offset) < 0.5 * ASSUMED_LANE_WIDTH;
    };

    // Check whether the very first coordinate is simply an offset. This is the case if the initial
    // vertex is close to the turn and the remaining coordinates are nearly straight.
    const auto offset_index = std::max<decltype(straight_index)>(1, straight_index);

    // we need at least a single coordinate
    if (offset_index + 1 >= coordinates.size())
        return false;

    // the straight part has to be around the lane distance
    if (!IsCloseToLaneDistance(segment_distances[offset_index]))
        return false;

    // the segment itself cannot be short
    if (segment_length < 0.8 * FAR_LOOKAHEAD_DISTANCE)
        return false;

    // if the remaining segment is short, we don't consider it an offset
    if ((segment_length - std::max(straight_distance, segment_distances[1])) > 0.1 * segment_length)
        return false;

    // finally, we cannot be far off from a straight line for the remaining coordinates
    return 0.5 * ASSUMED_LANE_WIDTH > GetMaxDeviation(coordinates.begin() + offset_index,
                                                      coordinates.end(),
                                                      coordinates[offset_index],
                                                      coordinates.back());
}

std::vector<util::Coordinate>
CoordinateExtractor::TrimCoordinatesToLength(std::vector<util::Coordinate> coordinates,
                                             const double desired_length) const
{
    double distance_to_current_coordinate = 0;

    for (std::size_t coordinate_index = 1; coordinate_index < coordinates.size();
         ++coordinate_index)
    {
        const auto distance_to_next_coordinate =
            distance_to_current_coordinate +
            util::coordinate_calculation::haversineDistance(coordinates[coordinate_index - 1],
                                                            coordinates[coordinate_index]);

        // if we reached the number of coordinates, we can stop here
        if (distance_to_next_coordinate >= desired_length)
        {
            coordinates.resize(coordinate_index + 1);
            coordinates.back() = util::coordinate_calculation::interpolateLinear(
                ComputeInterpolationFactor(
                    desired_length, distance_to_current_coordinate, distance_to_next_coordinate),
                coordinates[coordinate_index - 1],
                coordinates[coordinate_index]);
            break;
        }

        // remember the accumulated distance
        distance_to_current_coordinate = distance_to_next_coordinate;
    }
    if (coordinates.size() > 2 &&
        util::coordinate_calculation::haversineDistance(coordinates[0], coordinates[1]) <= 1)
        coordinates.erase(coordinates.begin() + 1);
    return coordinates;
}

util::Coordinate
CoordinateExtractor::GetCorrectedCoordinate(const util::Coordinate &fixpoint,
                                            const util::Coordinate &vector_base,
                                            const util::Coordinate &vector_head) const
{
    // if the coordinates are close together, we were not able to look far ahead, so
    // we can use the end-coordinate
    if (util::coordinate_calculation::haversineDistance(vector_base, vector_head) <
        DESIRED_COORDINATE_DIFFERENCE)
        return vector_head;
    else
    {
        /* to correct for the initial offset, we move the lookahead coordinate close
         * to the original road. We do so by subtracting the difference between the
         * turn coordinate and the offset coordinate from the lookahead coordinge:
         *
         * a ------ b ------ c
         *          |
         *          d
         *             \
         *                \
         *                   e
         *
         * is converted to:
         *
         * a ------ b ------ c
         *             \
         *                \
         *                   e
         *
         * for turn node `b`, vector_base `d` and vector_head `e`
         */
        const std::int32_t offset_percentage = 90;
        const auto corrected_lon =
            vector_head.lon -
            util::FixedLongitude{offset_percentage * (int)(vector_base.lon - fixpoint.lon) / 100};
        const auto corrected_lat =
            vector_head.lat -
            util::FixedLatitude{offset_percentage * (int)(vector_base.lat - fixpoint.lat) / 100};

        return util::Coordinate(corrected_lon, corrected_lat);
    }
};

std::vector<util::Coordinate>
CoordinateExtractor::SampleCoordinates(const std::vector<util::Coordinate> &coordinates,
                                       const double max_sample_length,
                                       const double rate) const
{
    BOOST_ASSERT(rate > 0 && coordinates.size() >= 2);

    // the return value
    std::vector<util::Coordinate> sampled_coordinates;

    // the very first coordinate is always part of the sample
    sampled_coordinates.push_back(coordinates.front());

    // interpolate coordinates as long as we are not past the desired length
    auto previous_coordinate = coordinates.begin();
    auto current_coordinate = std::next(previous_coordinate);
    for (double current_length = 0., total_length = 0.;
         total_length < max_sample_length && current_coordinate != coordinates.end();
         previous_coordinate = current_coordinate,
                current_coordinate = std::next(current_coordinate))
    {
        BOOST_ASSERT(current_length < rate);
        const auto distance_between = util::coordinate_calculation::haversineDistance(
            *previous_coordinate, *current_coordinate);
        if (current_length + distance_between >= rate)
        {
            // within the current segment, there is at least a single coordinate that we want to
            // sample. We extract all coordinates that are on our sampling intervals and update our
            // local sampling item to reflect the travelled distance
            const auto base_sampling = rate - current_length;

            // the number of samples in the interval is equal to the length of the interval (+ the
            // already traversed part from the previous segment) divided by the sampling rate
            BOOST_ASSERT(max_sample_length > total_length);
            const std::size_t num_samples = std::floor(
                (std::min(max_sample_length - total_length, distance_between) + current_length) /
                rate);

            for (std::size_t sample_value = 0; sample_value < num_samples; ++sample_value)
            {
                const auto interpolation_factor = ComputeInterpolationFactor(
                    base_sampling + sample_value * rate, 0, distance_between);
                auto sampled_coordinate = util::coordinate_calculation::interpolateLinear(
                    interpolation_factor, *previous_coordinate, *current_coordinate);
                sampled_coordinates.emplace_back(sampled_coordinate);
            }

            // current length needs to reflect how much is missing to the next sample. Here we can
            // ignore max sample range, because if we reached it, the loop is done anyhow
            current_length = (distance_between + current_length) - (num_samples * rate);
        }
        else
        {
            // do the necessary bookkeeping and continue
            current_length += distance_between;
        }
        // the total length travelled is always updated by the full distance
        total_length += distance_between;
    }

    return sampled_coordinates;
}

double CoordinateExtractor::ComputeInterpolationFactor(const double desired_distance,
                                                       const double distance_to_first,
                                                       const double distance_to_second) const
{
    BOOST_ASSERT(distance_to_first < desired_distance);
    double segment_length = distance_to_second - distance_to_first;
    BOOST_ASSERT(segment_length > 0);
    BOOST_ASSERT(distance_to_second >= desired_distance);
    double missing_distance = desired_distance - distance_to_first;
    return std::max(0., std::min(missing_distance / segment_length, 1.0));
}

std::vector<util::Coordinate>
CoordinateExtractor::TrimCoordinatesByLengthFront(std::vector<util::Coordinate> coordinates,
                                                  const double desired_length) const
{
    double distance_to_index = 0;
    std::size_t index = 0;
    for (std::size_t next_index = 1; next_index < coordinates.size(); ++next_index)
    {
        const double next_distance =
            distance_to_index + util::coordinate_calculation::haversineDistance(
                                    coordinates[index], coordinates[next_index]);
        if (next_distance >= desired_length)
        {
            const auto factor =
                ComputeInterpolationFactor(desired_length, distance_to_index, next_distance);
            auto interpolated_coordinate = util::coordinate_calculation::interpolateLinear(
                factor, coordinates[index], coordinates[next_index]);
            if (index > 0)
                coordinates.erase(coordinates.begin(), coordinates.begin() + index);
            coordinates.front() = interpolated_coordinate;
            return coordinates;
        }

        distance_to_index = next_distance;
        index = next_index;
    }

    // the coordinates in total are too short in length for the desired length
    coordinates.clear();
    return coordinates;
}

std::pair<util::Coordinate, util::Coordinate>
CoordinateExtractor::RegressionLine(const std::vector<util::Coordinate> &coordinates) const
{
    // create a sample of all coordinates to improve the quality of our regression vector
    // (less dependent on modelling of the data in OSM)
    const auto sampled_coordinates = SampleCoordinates(coordinates, FAR_LOOKAHEAD_DISTANCE, 1);

    /* We use the sum of least squares to calculate a linear regression through our
     * coordinates.
     * This regression gives a good idea of how the road can be perceived and corrects for
     * initial
     * and final corrections
     */
    const auto least_square_regression = [](const std::vector<util::Coordinate> &coordinates)
        -> std::pair<util::Coordinate, util::Coordinate> {
            double sum_lon = 0, sum_lat = 0, sum_lon_lat = 0, sum_lon_lon = 0;
            double min_lon = (double)toFloating(coordinates.front().lon);
            double max_lon = (double)toFloating(coordinates.front().lon);
            for (const auto coord : coordinates)
            {
                min_lon = std::min(min_lon, (double)toFloating(coord.lon));
                max_lon = std::max(max_lon, (double)toFloating(coord.lon));
                sum_lon += (double)toFloating(coord.lon);
                sum_lon_lon += (double)toFloating(coord.lon) * (double)toFloating(coord.lon);
                sum_lat += (double)toFloating(coord.lat);
                sum_lon_lat += (double)toFloating(coord.lon) * (double)toFloating(coord.lat);
            }

            const auto dividend = coordinates.size() * sum_lon_lat - sum_lon * sum_lat;
            const auto divisor = coordinates.size() * sum_lon_lon - sum_lon * sum_lon;
            if (std::abs(divisor) < std::numeric_limits<double>::epsilon())
                return std::make_pair(coordinates.front(), coordinates.back());

            // slope of the regression line
            const auto slope = dividend / divisor;
            const auto intercept = (sum_lat - slope * sum_lon) / coordinates.size();

            const auto GetLatatLon =
                [intercept, slope](const util::FloatLongitude longitude) -> util::FloatLatitude {
                return {intercept + slope * (double)(longitude)};
            };

            const util::Coordinate regression_first = {
                toFixed(util::FloatLongitude{min_lon - 1}),
                toFixed(util::FloatLatitude(GetLatatLon(util::FloatLongitude{min_lon - 1})))};
            const util::Coordinate regression_end = {
                toFixed(util::FloatLongitude{max_lon + 1}),
                toFixed(util::FloatLatitude(GetLatatLon(util::FloatLongitude{max_lon + 1})))};

            return {regression_first, regression_end};
        };

    // compute the regression vector based on the sum of least squares
    const auto regression_line = least_square_regression(sampled_coordinates);
    const auto coord_between_front =
        util::coordinate_calculation::projectPointOnSegment(
            regression_line.first, regression_line.second, coordinates.front())
            .second;
    const auto coord_between_back =
        util::coordinate_calculation::projectPointOnSegment(
            regression_line.first, regression_line.second, coordinates.back())
            .second;

    return {coord_between_front, coord_between_back};
}

} // namespace guidance
} // namespace extractor
} // namespace osrm
