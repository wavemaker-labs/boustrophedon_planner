#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/Quotient.h>
#include <CGAL/number_utils.h>
#include "boustrophedon_server/striping_planner.h"
#include "boustrophedon_server/cgal_utils.h"
#include "spline/Bezier.h"
#include "spline/BSpline.h"
#include "spline/CatmullRom.h"

void StripingPlanner::setParameters(StripingPlanner::Parameters parameters)
{
  params_ = parameters;
}

void StripingPlanner::addToPath(const Polygon& polygon, const Polygon& sub_polygon, Point& robot_position,
                                std::vector<NavPoint>& path)
{
  std::vector<NavPoint> new_path_section;

  fillPolygon(sub_polygon, new_path_section, robot_position);

  if (new_path_section.empty())
  {
    return;
  }

  bool trackBoundary = false;
  if (params_.travel_along_boundary)
  {
    trackBoundary = true;
  }
  else if (path.size() > 0)
  {
    // check if the straight line route to next sub-area goes outside the boundary
    auto intersections = getIntersectionPoints(polygon, Line(path.back().point, new_path_section.front().point));

    if (intersections.size() > 1)
    {
      // Determine all intermediate points between intersections
      auto went_outside = [&polygon](const auto& a, const auto& b) {
        auto midpoint_x = 0.5 * (a.x() + b.x());
        auto midpoint_y = 0.5 * (a.y() + b.y());
        auto result = polygon.bounded_side(Point(midpoint_x, midpoint_y));
        return (CGAL::ON_UNBOUNDED_SIDE == result);
      };
      for (size_t index = 0; index + 1 < intersections.size(); ++index)
      {
        if (went_outside(intersections[index], intersections[index + 1]))
        {
          trackBoundary = true;
          break;
        }
      }
    }
  }

  if (trackBoundary)
  {
    std::vector<NavPoint> start_to_striping =
        getOutlinePathToPoint(polygon, robot_position, new_path_section.front().point);

    path.insert(path.end(), start_to_striping.begin(), start_to_striping.end());
  }
  path.insert(path.end(), new_path_section.begin(), new_path_section.end());

  robot_position = new_path_section.back().point;
}

void StripingPlanner::addReturnToStart(const Polygon& polygon, const Point& start_position, const Point& robot_position,
                                       std::vector<NavPoint>& path)
{
  std::vector<NavPoint> striping_to_start = getOutlinePathToPoint(polygon, robot_position, start_position);
  path.insert(path.end(), striping_to_start.begin(), striping_to_start.end());
}

void StripingPlanner::fillPolygon(const Polygon& polygon, std::vector<NavPoint>& path, const Point& robot_position)
{
  auto left_vertex = *getLeftVertex(polygon);
  auto min_x = left_vertex.x();
  auto max_x = CGAL::right_vertex_2(polygon.container().begin(), polygon.container().end())->x();

  auto stripe_count = static_cast<int>(std::round(CGAL::to_double(max_x - min_x) / params_.stripe_separation)) + 1;

  StripingDirection stripe_dir = StripingDirection::STARTING;

  bool left_closest = isLeftClosest(polygon, robot_position, min_x, max_x);

  for (auto stripe_num = 0; stripe_num < stripe_count; stripe_num++)
  {
    double x;
    if (left_closest)
    {
      x = min_x + (stripe_num * params_.stripe_separation);
    }
    else
    {
      x = max_x - (stripe_num * params_.stripe_separation);
    }

    auto intersections = getIntersectionPoints(polygon, Line(Point(x, 0.0), Point(x, 1.0)));

    if (intersections.size() > 1)
    {
      // lambda for determining the first striping direction naively
      auto compare_current_point_distance = [&robot_position](const auto& a, const auto& b) {
        auto comp = CGAL::compare_distance_to_point(robot_position, a, b);
        return (comp == CGAL::SMALLER);
      };

      // lambda for sorting points based on current striping direction
      auto compare_striping_direction = [&stripe_dir](const auto& a, const auto& b) {
        if (stripe_dir == StripingDirection::UP)
        {
          return a.y() < b.y();
        }
        else if (stripe_dir == StripingDirection::DOWN)
        {
          return a.y() > b.y();
        }
        else
        {
          std::cout << "Comparing based on STARTING striping direction!" << std::endl;
          return true;
        }
      };

      if (stripe_dir == StripingDirection::STARTING)
      {
        // figure out the first striping direction using naive comparison of the two points
        std::sort(intersections.begin(), intersections.end(), compare_current_point_distance);
        // is the first point lower y than the second?
        stripe_dir =
            intersections.front().y() < intersections.back().y() ? StripingDirection::UP : StripingDirection::DOWN;
      }

      // add in intermediaries, not sorted
      addIntermediaryPoints(intersections);

      // remove any potential duplicates
      intersections.erase(std::unique(intersections.begin(), intersections.end()), intersections.end());

      // re-sort based on the striping direction
      std::sort(intersections.begin(), intersections.end(), compare_striping_direction);

      // here's where we add in the behavior to get from the previous stripe to this stripe
      if (params_.enable_half_y_turn)
      {
        // use the half-y-turn behavior
        addHalfYTurnPoints(path, intersections.front(), stripe_dir);

        // the half-y-turn places its own stripe start points, so we don't want to place this one naively, unless there
        // isn't anything in the path
        if (path.empty())
        {
          // we still need to place the start point in this case
          path.emplace_back(PointType::StripeStart, intersections.front());
        }
      }
      else if (params_.enable_full_u_turn || params_.enable_bulb_turn)
      {
        // the full-u-turn places its own stripe start points, so we don't want to place this one naively, unless there
        // isn't anything in the path
        if (path.empty())
        {
          // we still need to place the start point in this case
          path.emplace_back(PointType::StripeStart, intersections.front());
        }
        else
        {
          auto factor = params_.enable_bulb_turn ? 5.0 : 3.0;
          auto edge_delta = factor * (params_.stripe_separation + params_.u_turn_radius);

          // do not make an arc turn if the difference between the two edge points make the plan go over the safety edge
          // instead revert to following the boundary edge to get to the next point
          if ( !isTurnAreaSufficient(path, intersections)
            || definitelyGreaterThan(abs(path.back().point.y() - intersections.front().y()), edge_delta, EPSILON) )
          {
            addBoundaryFollowingPoints(path, intersections.front(), polygon);

            // place the points into the path
            path.emplace_back(PointType::StripeStart, intersections.front());
          }
          else
          {
            // use the full-u-turn behavior
            addFullUTurnPoints(path, intersections.front(), stripe_dir);
          }
        }
      }
      else
      {
        // else use the simple boundary_following behavior
        // follow the boundary to get to the next point
        addBoundaryFollowingPoints(path, intersections.front(), polygon);

        // place the points into the path
        path.emplace_back(PointType::StripeStart, intersections.front());
      }

      for (auto it = intersections.begin() + 1; it < intersections.end() - 1; it++)
      {
        path.emplace_back(PointType::StripeIntermediate, *it);
      }
      path.emplace_back(PointType::StripeEnd, intersections.back());

      // reverse the striping direction for the next stripe
      stripe_dir = stripe_dir == StripingDirection::UP ? StripingDirection::DOWN : StripingDirection::UP;
    }
  }
}

// adds in waypoints between the two waypoints provided in the intersections vector
void StripingPlanner::addIntermediaryPoints(std::vector<Point>& intersections)
{
  Point lower_point = intersections.front();
  Point upper_point = intersections.back();
  double x = lower_point.x();  // this is the x coordinate of all points on this striping line
  double y = lower_point.y();  // this is the base y coordinate

  int intermediary_count =
      static_cast<int>(std::trunc((upper_point.y() - lower_point.y()) / params_.intermediary_separation));

  // start the loop at 1. We don't want to repeat a point on the ends of the line segment.
  for (int i = 1; i < intermediary_count; i++)
  {
    double new_y = i * params_.intermediary_separation + y;
    intersections.emplace_back(x, new_y);
  }
}

void StripingPlanner::addBoundaryFollowingPoints(std::vector<NavPoint>& path, const Point& next_stripe_start,
                                                 Polygon polygon)
{
  // sanity check, don't add a boundary following point if there is no previous stripe
  if (path.empty())
  {
    return;
  }

  // first, insert the last point of the path (the end of last stripe) and the start of the next stripe to the polygon
  insertPointAlongEdge(path.back().point, polygon);
  insertPointAlongEdge(next_stripe_start, polygon);

  // we now have a polygon with all necessary points to find a path between the stripes
  std::vector<NavPoint> boundary_path = getOutlinePathToPoint(polygon, path.back().point, next_stripe_start);

  // check if there's actually a new path we have to follow, instead of just going straight from end to start
  if (boundary_path.size() > 2)
  {
    // if so, add it to the path. the end is already in there, and the start will be added later
    path.insert(path.end(), boundary_path.begin() + 1, boundary_path.end() - 1);

    std::cout << "inserted an extra boundary following path of size: " << boundary_path.size() - 2 << std::endl;
  }
}

bool StripingPlanner::isTurnAreaSufficient(const std::vector<NavPoint>& path, const std::vector<Point> &next_stripe)
{

  if (path.size() < 2 || next_stripe.size() < 2)
  {
    return false;
  }

  double turn_offset{0.0};

  double boundary_angle = atan2(fabs(path.back().point.y() - next_stripe.front().y()), params_.stripe_separation);
  double setback_factor = 1.2 * (1 - sin(boundary_angle)) / cos(boundary_angle);
  if (params_.enable_bulb_turn)
  {
    turn_offset = std::max(4.6 * params_.u_turn_radius, 2.3 * params_.stripe_separation);
    // turn_offset += turn_offset * setback_factor;
  }
  else
    turn_offset = params_.stripe_separation * 2.3 + params_.stripe_separation * setback_factor;

  bool allowed = false;
  if (definitelyGreaterThan(abs(path.back().point.y() - path[path.size() - 2].point.y()),
        turn_offset, EPSILON)
      && definitelyGreaterThan(abs(next_stripe.front().y() - next_stripe[1].y()),
        turn_offset, EPSILON))
  {
    allowed = true;
  }

  return allowed;
}

void StripingPlanner::addHalfYTurnPoints(std::vector<NavPoint>& path, const Point& next_stripe_start,
                                         StripingDirection& stripe_dir)
{
  // sanity check, don't add a boundary following point if there is no previous stripe
  if (path.empty())
  {
    return;
  }

  Point arc_center_point;
  std::vector<NavPoint> arc;

  // we'll need to do different behaviors based on which direction we are striping in
  // the key differences are whether we are striping left-to-right or right-to-left, and whether we are going up or down
  if (stripe_dir == StripingDirection::DOWN && path.back().point.x() < next_stripe_start.x())
  {
    // we are going left to right
    // the current stripe is going down, so we need an arc from an UP stripe to a DOWN stripe

    // figure out which one of path.back() or next_stripe_start is higher y-coordinate
    if (definitelyGreaterThan(path.back().point.y(), next_stripe_start.y(), EPSILON))
    {
      // the previous stripe's end is higher than the current stripe's start.

      // the arc is centered around a point in the previous stripe
      arc_center_point =
          Point(path.back().point.x(), path.back().point.y() + params_.turn_start_offset - params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from PI/2 and ending at 0 (left to right, higher to lower)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, CGAL_PI / 2, 0.0,
                                   params_.points_per_turn);

      // the first point in the arc is the zero-turn point, make it end the stripe
      arc.front().type = PointType::StripeEnd;
      // copy the first point in the arc to start the next stripe
      arc.insert(arc.begin() + 1, NavPoint{ PointType::StripeStart, arc.front().point });
    }
    else
    {
      // the previous stripe's end is lower or equal(ish) to the current stripe's start.

      // the arc is centered around a point in the current stripe
      arc_center_point =
          Point(next_stripe_start.x(), next_stripe_start.y() + params_.turn_start_offset - params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from PI and ending at PI/2 (left to right, lower to higher)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, CGAL_PI, CGAL_PI / 2,
                                   params_.points_per_turn);

      // the last point in the arc is the zero-turn point, make it end the stripe
      arc.back().type = PointType::StripeEnd;
      // copy the last point in the arc to start the next stripe
      arc.insert(arc.end(), NavPoint{ PointType::StripeStart, arc.back().point });
    }
  }
  else if (stripe_dir == StripingDirection::UP && path.back().point.x() < next_stripe_start.x())
  {
    // we are going left to right
    // the current stripe is going up, so we need an arc from a DOWN stripe to an UP stripe.

    // figure out which one of path.back() or next_stripe_start is higher y-coordinate
    if (definitelyLessThan(path.back().point.y(), next_stripe_start.y(), EPSILON))
    {
      // the previous stripe's end is lower than the current stripe's start.

      // the arc is centered around a point in the previous stripe
      arc_center_point =
          Point(path.back().point.x(), path.back().point.y() - params_.turn_start_offset + params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from 3PI/2 and ending at 2PI (left to right, lower to higher)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, 3 * CGAL_PI / 2, CGAL_PI * 2,
                                   params_.points_per_turn);

      // the first point in the arc is the zero-turn point, make it end the stripe
      arc.front().type = PointType::StripeEnd;
      // copy the first point in the arc to start the next stripe
      arc.insert(arc.begin() + 1, NavPoint{ PointType::StripeStart, arc.front().point });
    }
    else
    {
      // the previous stripe's end is higher or equal(ish) to the current stripe's start.

      // the arc is centered around a point in the current stripe
      arc_center_point =
          Point(next_stripe_start.x(), next_stripe_start.y() - params_.turn_start_offset + params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from PI and ending at 3PI/2 (left to right, higher to lower)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, CGAL_PI, 3 * CGAL_PI / 2,
                                   params_.points_per_turn);

      // the last point in the arc is the zero-turn point, make it end the stripe
      arc.back().type = PointType::StripeEnd;
      // copy the last point in the arc to start the next stripe
      arc.insert(arc.end(), NavPoint{ PointType::StripeStart, arc.back().point });
    }
  }
  else if (stripe_dir == StripingDirection::DOWN && path.back().point.x() > next_stripe_start.x())
  {
    // we are going right to left
    // the current stripe is going down, so we need an arc from an UP stripe to a DOWN stripe

    // figure out which one of path.back() or next_stripe_start is higher y-coordinate
    if (definitelyGreaterThan(path.back().point.y(), next_stripe_start.y(), EPSILON))
    {
      // the previous stripe's end is higher than the current stripe's start.

      // the arc is centered around a point in the previous stripe
      arc_center_point =
          Point(path.back().point.x(), path.back().point.y() + params_.turn_start_offset - params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from PI/2 and ending at PI (right to left, higher to lower)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, CGAL_PI / 2, CGAL_PI,
                                   params_.points_per_turn);

      // the first point in the arc is the zero-turn point, make it end the stripe
      arc.front().type = PointType::StripeEnd;
      // copy the first point in the arc to start the next stripe
      arc.insert(arc.begin() + 1, NavPoint{ PointType::StripeStart, arc.front().point });
    }
    else
    {
      // the previous stripe's end is lower or equal(ish) to the current stripe's start.

      // the arc is centered around a point in the current stripe
      arc_center_point =
          Point(next_stripe_start.x(), next_stripe_start.y() + params_.turn_start_offset - params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from 0 and ending at PI/2 (right to left, lower to higher)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, 0.0, CGAL_PI / 2,
                                   params_.points_per_turn);

      // the last point in the arc is the zero-turn point, make it end the stripe
      arc.back().type = PointType::StripeEnd;
      // copy the last point in the arc to start the next stripe
      arc.insert(arc.end(), NavPoint{ PointType::StripeStart, arc.back().point });
    }
  }
  else if (stripe_dir == StripingDirection::UP && path.back().point.x() > next_stripe_start.x())
  {
    // we are going right to left
    // the current stripe is going up, so we need an arc from a DOWN stripe to an UP stripe.

    // figure out which one of path.back() or next_stripe_start is higher y-coordinate
    if (definitelyLessThan(path.back().point.y(), next_stripe_start.y(), EPSILON))
    {
      // the previous stripe's end is lower than the current stripe's start.

      // the arc is centered around a point in the previous stripe
      arc_center_point =
          Point(path.back().point.x(), path.back().point.y() - params_.turn_start_offset + params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from 3PI/2 and ending at PI (right to left, lower to higher)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, 3 * CGAL_PI / 2, CGAL_PI,
                                   params_.points_per_turn);

      // the first point in the arc is the zero-turn point, make it end the stripe
      arc.front().type = PointType::StripeEnd;
      // copy the first point in the arc to start the next stripe
      arc.insert(arc.begin() + 1, NavPoint{ PointType::StripeStart, arc.front().point });
    }
    else
    {
      // the previous stripe's end is higher or equal(ish) to the current stripe's start.

      // the arc is centered around a point in the current stripe
      arc_center_point =
          Point(next_stripe_start.x(), next_stripe_start.y() - params_.turn_start_offset + params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from 2PI and ending at 3PI/2 (right to left, higher to lower)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, CGAL_PI * 2, 3 * CGAL_PI / 2,
                                   params_.points_per_turn);

      // the last point in the arc is the zero-turn point, make it end the stripe
      arc.back().type = PointType::StripeEnd;
      // copy the last point in the arc to start the next stripe
      arc.insert(arc.end(), NavPoint{ PointType::StripeStart, arc.back().point });
    }
  }
  else
  {
    // we shouldn't be able to get here
    std::cout << "addHalfYTurnPoints(): stripe_dir was neither DOWN nor UP!" << std::endl;
    return;
  }
  // remove the previous stripe end point
  path.erase(path.end());

  // insert the arc points onto the end of the path
  path.insert(path.end(), arc.begin(), arc.end());
}


void StripingPlanner::addFullUTurnPoints(std::vector<NavPoint>& path, const Point& next_stripe_start,
                                         StripingDirection& stripe_dir)
{
  // sanity check, don't add a boundary following point if there is no previous stripe
  if (path.empty())
  {
    return;
  }

  Point arc_center_point;
  Point turn_start_point;
  Point turn_end_point;
  Point turn_start_neighbor;
  Point turn_end_neighbor;

  std::vector<NavPoint> arc;
  std::vector<NavPoint> arc_trace;
  double arc_center_offset;
  double rad_offset;

  // Check if the minimum turning radius is set to be larger than the separation, which means adjustments
  if (params_.enable_bulb_turn && params_.u_turn_radius > (params_.stripe_separation / 2.0))
  {
    // distance from the edge
    arc_center_offset = params_.u_turn_radius - (params_.stripe_separation / 2.0);
    // wideness of the turn arc that opens just enough to connect with the stripe width
    rad_offset = acos(params_.stripe_separation / (2.0 * params_.u_turn_radius)) / 2.0;
  }
  else
  {
    arc_center_offset = 0.0;
    rad_offset = 0.0;
  }

  double arc_radius = arc_center_offset + params_.stripe_separation / 2.0;
  double start_rad, start_rad_offset;
  double end_rad, end_rad_offset;

  // for now this edge boundary turning is only guaranteed for non-bulb u-turns
  double boundary_angle = atan2(fabs(path.back().point.y() - next_stripe_start.y()), params_.stripe_separation);
  double turn_setback = arc_radius * (1 - sin(boundary_angle)) / cos(boundary_angle);

  constexpr double k_lookahead_factor = 2.3;

  // EQ: Patterned after Y-turn logic so it's easier to continue the thought process

  // we'll need to do different behaviors based on which direction we are striping in
  // the key differences are whether we are striping left-to-right or right-to-left, and whether we are going up or down
  if (stripe_dir == StripingDirection::DOWN && path.back().point.x() < next_stripe_start.x())
  {
    // we are going left to right
    // the current stripe is going down, so we need an arc from an UP stripe to a DOWN stripe
    // the arc is centered between the two stripes
    arc_center_point =
          Point((path.back().point.x() + next_stripe_start.x()) / 2.0,
                std::min(path.back().point.y(), next_stripe_start.y())
                 + params_.turn_start_offset - turn_setback);

    // Common circular arc, starts from PI and ends at 0 (upper semicircle from left to right)
    start_rad = CGAL_PI;
    end_rad = 0.0;
    start_rad_offset = CGAL_PI + rad_offset;
    end_rad_offset = 0.0 - rad_offset;

    turn_start_point = Point(path.back().point.x(),
                             std::min(path.back().point.y(), next_stripe_start.y())
                             + params_.turn_start_offset - arc_radius * k_lookahead_factor);
    turn_end_point = Point(next_stripe_start.x(),
                           std::min(path.back().point.y(), next_stripe_start.y())
                           + params_.turn_start_offset - arc_radius * k_lookahead_factor);
    turn_start_neighbor = Point(turn_start_point.x(), turn_start_point.y() + 0.1);
    turn_end_neighbor = Point(turn_end_point.x(), turn_end_point.y() - 0.1);
  }
  else if (stripe_dir == StripingDirection::UP && path.back().point.x() < next_stripe_start.x())
  {
    // we are going left to right
    // the current stripe is going up, so we need an arc from a DOWN stripe to an UP stripe.
    // the arc is centered between the two stripes
    arc_center_point =
          Point((path.back().point.x() + next_stripe_start.x()) / 2.0,
                std::max(path.back().point.y(), next_stripe_start.y())
                 - params_.turn_start_offset + turn_setback);

    // Common circular arc, starts from CGAL_PI and ends at CGAL_PI * 2 (lower semicircle, left to right)
    start_rad = CGAL_PI;
    end_rad = CGAL_PI * 2;
    start_rad_offset = CGAL_PI - rad_offset;
    end_rad_offset = CGAL_PI * 2 + rad_offset;

    turn_start_point = Point(path.back().point.x(),
                             std::max(path.back().point.y(), next_stripe_start.y())
                             - params_.turn_start_offset + arc_radius * k_lookahead_factor);
    turn_end_point = Point(next_stripe_start.x(),
                           std::max(path.back().point.y(), next_stripe_start.y())
                           - params_.turn_start_offset + arc_radius * k_lookahead_factor);
    turn_start_neighbor = Point(turn_start_point.x(), turn_start_point.y() - 0.1);
    turn_end_neighbor = Point(turn_end_point.x(), turn_end_point.y() + 0.1);
  }
  else if (stripe_dir == StripingDirection::DOWN && path.back().point.x() > next_stripe_start.x())
  {
    // we are going right to left
    // the current stripe is going down, so we need an arc from an UP stripe to a DOWN stripe
    // the arc is centered between the two stripes
    arc_center_point =
          Point((path.back().point.x() + next_stripe_start.x()) / 2.0,
                std::min(path.back().point.y(), next_stripe_start.y())
                 + params_.turn_start_offset - turn_setback);

    // Common circular arc, starts from 0 and ends at CGAL_PI (upper semicircle, right to left)
    start_rad = 0.0;
    end_rad = CGAL_PI;
    start_rad_offset = 0.0 - rad_offset;
    end_rad_offset = CGAL_PI + rad_offset;

    turn_start_point = Point(path.back().point.x(),
                             std::min(path.back().point.y(), next_stripe_start.y())
                             + params_.turn_start_offset - arc_radius * k_lookahead_factor);
    turn_end_point = Point(next_stripe_start.x(),
                           std::min(path.back().point.y(), next_stripe_start.y())
                             + params_.turn_start_offset - arc_radius * k_lookahead_factor);
    turn_start_neighbor = Point(turn_start_point.x(), turn_start_point.y() + 0.1);
    turn_end_neighbor = Point(turn_end_point.x(), turn_end_point.y() - 0.1);
  }
  else if (stripe_dir == StripingDirection::UP && path.back().point.x() > next_stripe_start.x())
  {
    // we are going right to left
    // the current stripe is going up, so we need an arc from a DOWN stripe to an UP stripe.
    // the arc is centered between the two stripes
    arc_center_point =
          Point((path.back().point.x() + next_stripe_start.x()) / 2.0,
                std::max(path.back().point.y(), next_stripe_start.y())
                 - params_.turn_start_offset + turn_setback);

    // Common circular arc, starts from CGAL_PI * 2 and ends at CGAL_PI (lower semicircle, right to left)
    start_rad = CGAL_PI * 2;
    end_rad = CGAL_PI;
    start_rad_offset = CGAL_PI * 2 + rad_offset;
    end_rad_offset = CGAL_PI - rad_offset;

    turn_start_point = Point(path.back().point.x(),
                             std::max(path.back().point.y(), next_stripe_start.y())
                             - params_.turn_start_offset + arc_radius * k_lookahead_factor);
    turn_end_point = Point(next_stripe_start.x(),
                           std::max(path.back().point.y(), next_stripe_start.y())
                           - params_.turn_start_offset + arc_radius * k_lookahead_factor);
    turn_start_neighbor = Point(turn_start_point.x(), turn_start_point.y() - 0.1);
    turn_end_neighbor = Point(turn_end_point.x(), turn_end_point.y() + 0.1);
  }
  else
  {
    // we shouldn't be able to get here
    std::cout << "addFullUTurnPoints(): stripe_dir was neither DOWN nor UP!" << std::endl;
    return;
  }

  arc = generateDiscretizedArc(arc_center_point,
                               arc_radius,
                               start_rad, end_rad,
                               params_.points_per_turn);

  // The first point in the arc terminates the stripe, and the last point starts the next stripe
  // arc.front().type = PointType::StripeEnd;
  // arc.insert(arc.end(), NavPoint{ PointType::StripeStart, arc.back().point });


  // remove the previous stripe end point
  path.erase(path.end());
  path.push_back(NavPoint(PointType::StripeEnd, turn_start_point));

  BSpline arc_entry_curve;
  BSpline arc_exit_curve;
  arc_entry_curve.set_steps(params_.points_per_turn);
  arc_exit_curve.set_steps(params_.points_per_turn);

  if (params_.enable_bulb_turn)
  {
    arc_trace = generateDiscretizedArc(arc_center_point,
                                      arc_radius,
                                      start_rad_offset, end_rad_offset,
                                      10);
    // curve smoothen start
    arc_entry_curve.add_way_point(Vector(turn_start_point.x(), turn_start_point.y()));
    arc_entry_curve.add_way_point(Vector(turn_start_neighbor.x(), turn_start_neighbor.y()));
    arc_entry_curve.add_way_point(Vector(arc_trace.front().point.x(), arc_trace.front().point.y()));
    arc_entry_curve.add_way_point(Vector(arc_trace[1].point.x(), arc_trace[1].point.y()));

#define USE_ORIGINAL_CIRCULAR_ARC 1
#if USE_ORIGINAL_CIRCULAR_ARC
    arc_entry_curve.add_way_point(Vector(arc.front().point.x(), arc.front().point.y()));
#else
    for (auto const& nav_point : arc_trace)
    {
      arc_entry_curve.add_way_point(Vector(nav_point.point.x(), nav_point.point.y()));
    }
    arc_entry_curve.add_way_point(Vector(turn_end_point.x(), turn_end_point.y()));
    arc_entry_curve.add_way_point(Vector(turn_end_neighbor.x(), turn_end_neighbor.y()));
#endif

    // insert the arc points onto the end of the path
    for (auto const &vector_pt : arc_entry_curve.nodes())
    {
      path.push_back(NavPoint(PointType::StripeIntermediate, Point(vector_pt.x(), vector_pt.y())));
    }
  }

#if USE_ORIGINAL_CIRCULAR_ARC
  // circular turn arc
  path.insert(path.end(), arc.begin(), arc.end());

  if (params_.enable_bulb_turn)
  {
    // curve smoothen end
    arc_exit_curve.add_way_point(Vector(arc.back().point.x(), arc.back().point.y()));
    arc_exit_curve.add_way_point(Vector(arc_trace[arc_trace.size()-2].point.x(), arc_trace[arc_trace.size()-2].point.y()));
    arc_exit_curve.add_way_point(Vector(arc_trace.back().point.x(), arc_trace.back().point.y()));
    arc_exit_curve.add_way_point(Vector(turn_end_point.x(), turn_end_point.y()));
    arc_exit_curve.add_way_point(Vector(turn_end_neighbor.x(), turn_end_neighbor.y()));
    for (auto const &vector_pt : arc_exit_curve.nodes())
    {
      path.push_back(NavPoint(PointType::StripeIntermediate, Point(vector_pt.x(), vector_pt.y())));
    }
  }
#endif

  path.push_back(NavPoint(PointType::StripeStart, turn_end_neighbor));
}

std::vector<NavPoint> StripingPlanner::generateDiscretizedArc(const Point& center_point, const float& radius,
                                                              const float& start_rad, const float& end_rad,
                                                              const int& num_points)
{
  std::vector<NavPoint> points;

  float radian_interval = (end_rad - start_rad) / (num_points - 1);

  for (int i = 0; i < num_points; i++)
  {
    float current_radian = start_rad + (radian_interval * i);
    Point point =
        Point(center_point.x() + (radius * cos(current_radian)), center_point.y() + (radius * sin(current_radian)));
    points.emplace_back(PointType::StripeIntermediate, point);
  }

  return points;
}

bool StripingPlanner::isLeftClosest(const Polygon& polygon, const Point& robot_position, double& min_x, double& max_x)
{
  // lambda for determining the first striping direction naively
  auto compare_current_point_distance = [&robot_position](const auto& a, const auto& b) {
    // EQ: revised to look only for nearer x distance
    return CGAL::abs(robot_position.x() - a.x()) < CGAL::abs(robot_position.x() - b.x());
  };

  std::vector<Point> starting_points = getIntersectionPoints(polygon, Line(Point(min_x, 0.0), Point(min_x, 1.0)));
  std::vector<Point> right_points = getIntersectionPoints(polygon, Line(Point(max_x, 0.0), Point(max_x, 1.0)));
  starting_points.insert(starting_points.end(), right_points.begin(), right_points.end());
  std::sort(starting_points.begin(), starting_points.end(), compare_current_point_distance);

  // if we cannot determine, fix this to false
  if (starting_points.empty())
  {
    std::cout << "WARNING: START POINT NOT FOUND!" << std::endl;
    return false;
  }

  // if the closest potential starting point is on the left, return true. If not, return false.
  return starting_points.front().x() == min_x;
}

std::vector<NavPoint> StripingPlanner::getOutlinePathToPoint(const Polygon& polygon, const Point& start_point,
                                                             const Point& end_point)
{
  NeighborSearch::Tree tree(polygon.vertices_begin(), polygon.vertices_end());  // Initialize the search tree

  NeighborSearch start_search(tree, start_point, 1);  // find the 1 nearest point to start_point
  Point polygon_start_point = start_search.begin()->first;

  NeighborSearch end_search(tree, end_point, 1);  // find the 1 nearest point to end_point
  Point polygon_end_point = end_search.begin()->first;

  // now we need to figure out the shortest path along the polygon from polygon_start_point to polygon_end_point
  // we only need to try two paths: one going clockwise and one going counter-clockwise
  std::vector<NavPoint> clockwise_path = getPolygonPath(polygon, polygon_start_point, polygon_end_point);
  Polygon reversed_polygon = polygon;
  reversed_polygon.reverse_orientation();
  std::vector<NavPoint> counter_clockwise_path =
      getPolygonPath(reversed_polygon, polygon_start_point, polygon_end_point);
  if (getPathLength(clockwise_path) < getPathLength(counter_clockwise_path))
  {
    return clockwise_path;
  }
  else
  {
    return counter_clockwise_path;
  }
}

std::vector<NavPoint> StripingPlanner::getPolygonPath(const Polygon& polygon, const Point& start_point,
                                                      const Point& end_point)
{
  // get a circulator starting at start_point
  Polygon::Vertex_const_circulator start_circulator = polygon.vertices_circulator();

  while (*start_circulator != start_point)  // loop until the circulator is pointing at the start
  {
    start_circulator++;
  }
  // start with the start_point
  std::vector<NavPoint> path;
  path.emplace_back(PointType::Outline, *start_circulator);

  // now we can add points to the path
  while (*start_circulator++ != end_point)
  {
    path.emplace_back(PointType::Outline, *start_circulator);
  }

  return path;
}

float StripingPlanner::getPathLength(const std::vector<NavPoint>& path)
{
  float squared_length = 0.0;
  for (auto it = path.begin(); it != --path.end(); it++)
  {
    Point point_1 = it->point;
    Point point_2 = (it + 1)->point;
    squared_length += CGAL::sqrt(CGAL::squared_distance(point_1, point_2));
  }
  return squared_length;
}
