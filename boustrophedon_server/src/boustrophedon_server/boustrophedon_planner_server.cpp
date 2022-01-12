#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include "boustrophedon_server/boustrophedon_planner_server.h"

BoustrophedonPlannerServer::BoustrophedonPlannerServer()
  : private_node_handle_("~")
  , action_server_(node_handle_, "plan_path", boost::bind(&BoustrophedonPlannerServer::executePlanPathAction, this, _1),
                   false)
  , action_server_with_param_(node_handle_, "config_and_plan_path", boost::bind(&BoustrophedonPlannerServer::configAndExecutePlanPathAction, this, _1),
                   false)
  , conversion_server_{ node_handle_.advertiseService("convert_striping_plan_to_path",
                                                      &BoustrophedonPlannerServer::convertStripingPlanToPath, this) }
{
  std::size_t error = fetchParams();

  // todo make this dynamic_reconfigurable instead of topic-based so parameter server mirrors the changes
  params_subscriber_ = private_node_handle_.subscribe(parameters_topic_, 1, &BoustrophedonPlannerServer::updateParamsInternal, this);
  default_params_publisher_ = private_node_handle_.advertise<boustrophedon_msgs::PlanParameters>("default_config", 1, true);

  // todo: test this out for consistency. If not, then switch to subscriber connection callback
  publishCurrentParameters();

  action_server_.start();
  action_server_with_param_.start();

  if (publish_polygons_)
  {
    initial_polygon_publisher_ = private_node_handle_.advertise<geometry_msgs::PolygonStamped>("initial_polygon", 1);
    preprocessed_polygon_publisher_ =
        private_node_handle_.advertise<geometry_msgs::PolygonStamped>("preprocessed_polygon", 1);
  }
  // mainly for use with plotJuggler, which wants the points to be put one at a time on the same topic
  if (publish_path_points_)
  {
    path_points_publisher_ = private_node_handle_.advertise<geometry_msgs::PointStamped>("path_points", 1000);
    polygon_points_publisher_ = private_node_handle_.advertise<geometry_msgs::PointStamped>("polygon_points", 1000);
  }

}

std::size_t BoustrophedonPlannerServer::fetchParams()
{
  std::size_t error = 0;
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "repeat_boundary", params_.repeat_boundary_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "outline_clockwise", params_.outline_clockwise_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "skip_outlines", params_.skip_outlines_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "outline_layer_count", params_.outline_layer_count_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "stripe_separation", params_.stripe_separation_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "intermediary_separation", params_.intermediary_separation_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "stripe_angle", params_.stripe_angle_));
  error += static_cast<std::size_t>(!rosparam_shortcuts::get("plan_path", private_node_handle_,
                                                             "enable_stripe_angle_orientation", params_.enable_orientation_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "travel_along_boundary", params_.travel_along_boundary_));
  error += static_cast<std::size_t>(!rosparam_shortcuts::get(
      "plan_path", private_node_handle_, "allow_points_outside_boundary", params_.allow_points_outside_boundary_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "enable_half_y_turns", params_.enable_half_y_turns_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "points_per_turn", params_.points_per_turn_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "turn_start_offset", params_.turn_start_offset_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "u_turn_radius", params_.u_turn_radius_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "publish_polygons", publish_polygons_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "publish_path_points", publish_path_points_));

  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "enable_full_u_turns", params_.enable_full_u_turns_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "enable_bulb_turns", params_.enable_bulb_turns_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "stripes_before_outlines", params_.stripes_before_outlines_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "parameters_topic", parameters_topic_));

  rosparam_shortcuts::shutdownIfError("plan_path", error);

  if (params_.intermediary_separation_ <= 0.0)
  {
    // doesn't make sense, or we don't want intermediaries. set it to double max so we can't make any intermediaries
    params_.intermediary_separation_ = std::numeric_limits<double>::max();
  }

  if (params_.enable_half_y_turns_ && params_.outline_layer_count_ < 1)
  {
    if (params_.allow_points_outside_boundary_)
    {
      ROS_WARN_STREAM("Current configuration will result in turns that go outside the boundary, but this has been "
                      "explicitly enabled");
    }
    else
    {
      // we can't do half-y-turns safely without an inner boundary layer, as the arc will stick outside of the boundary
      ROS_ERROR_STREAM("Cannot plan using half-y-turns if the outline_layer_count is less than 1! Boustrophedon "
                       "planner will not start.");
      return error;
    }
  }

  striping_planner_.setParameters({
        params_.stripe_separation_,
        params_.intermediary_separation_,
        params_.travel_along_boundary_,
        params_.enable_half_y_turns_,
        params_.enable_full_u_turns_,
        params_.enable_bulb_turns_,
        params_.points_per_turn_,
        params_.turn_start_offset_,
        params_.u_turn_radius_
      });
  outline_planner_.setParameters({
        params_.repeat_boundary_,
        params_.outline_clockwise_,
        params_.skip_outlines_,
        params_.outline_layer_count_,
        params_.stripe_separation_
      });

  return error;
}


void BoustrophedonPlannerServer::updateParamsInternal(const boustrophedon_msgs::PlanParameters &params)
{
  ROS_INFO_STREAM("Setting new parameters " << params);
  params_.stripe_separation_ = params.cut_spacing;

  // The angle received from UI is in degrees. This must be converted to radians before saving
  params_.stripe_angle_ = params.cut_angle_degrees * 3.1415927 / 180.0;
  params_.stripes_before_outlines_ = params.stripes_before_outlines;
  params_.enable_orientation_ = params.enable_stripe_angle_orientation;
  params_.intermediary_separation_ = params.intermediary_separation;
  params_.travel_along_boundary_ = params.travel_along_boundary;
  params_.points_per_turn_ = params.points_per_turn;
  params_.turn_start_offset_ = params.turn_start_offset;
  params_.repeat_boundary_ = params.repeat_boundary;
  params_.outline_clockwise_ = params.outline_clockwise;
  params_.skip_outlines_ = params.skip_outlines;
  params_.outline_layer_count_ = params.outline_layer_count;
  params_.u_turn_radius_ = params.u_turn_radius;

  switch (params.turn_type)
  {
    case boustrophedon_msgs::PlanParameters::TURN_BULB:
      params_.enable_half_y_turns_ = false;
      params_.enable_full_u_turns_ = false;
      params_.enable_bulb_turns_ = true;
      break;
    case boustrophedon_msgs::PlanParameters::TURN_FULL_U:
      params_.enable_half_y_turns_ = false;
      params_.enable_full_u_turns_ = true;
      params_.enable_bulb_turns_ = false;
      break;
    case boustrophedon_msgs::PlanParameters::TURN_HALF_Y:
      params_.enable_half_y_turns_ = true;
      params_.enable_full_u_turns_ = false;
      params_.enable_bulb_turns_ = false;
      break;
    case boustrophedon_msgs::PlanParameters::TURN_BOUNDARY:
      params_.enable_half_y_turns_ = false;
      params_.enable_full_u_turns_ = false;
      params_.enable_bulb_turns_ = false;
      break;
  }

  striping_planner_.setParameters({
        params_.stripe_separation_,
        params_.intermediary_separation_,
        params_.travel_along_boundary_,
        params_.enable_half_y_turns_,
        params_.enable_full_u_turns_,
        params_.enable_bulb_turns_,
        params_.points_per_turn_,
        params_.turn_start_offset_
      });
  outline_planner_.setParameters({
        params_.repeat_boundary_,
        params_.outline_clockwise_,
        params_.skip_outlines_,
        params_.outline_layer_count_,
        params_.stripe_separation_
      });
}

void BoustrophedonPlannerServer::configAndExecutePlanPathAction(const boustrophedon_msgs::PlanMowingPathParamGoalConstPtr& goalWithParams)
{
  boustrophedon_msgs::PlanMowingPathGoal goal;
  goal.property = goalWithParams->property;
  goal.robot_position = goalWithParams->robot_position;

  updateParamsInternal(goalWithParams->parameters);

  std::string boundary_frame = goalWithParams->property.header.frame_id;

  auto path = executePlanPathInternal(goal, params_);
  auto result = toResult(std::move(path), boundary_frame);

  if (last_status_.empty())
  {
    boustrophedon_msgs::PlanMowingPathParamResult result_with_param;
    result_with_param.plan = result.plan;
    action_server_with_param_.setSucceeded(result_with_param);
  }
  else
  {
    action_server_with_param_.setAborted(ServerWithParam::Result(), last_status_);
  }

  //todo: return-update to rosparam server. For now the action client holds the responsibility (not this callback)
}

void BoustrophedonPlannerServer::executePlanPathAction(const boustrophedon_msgs::PlanMowingPathGoalConstPtr& goal)
{
  std::string boundary_frame = goal->property.header.frame_id;
  auto path = executePlanPathInternal(*goal, params_);
  auto result = toResult(std::move(path), boundary_frame);
  if (last_status_.empty())
  {
    action_server_.setSucceeded(result);
  }
  else
  {
    action_server_.setAborted(Server::Result(), last_status_);
  }
}

std::vector<NavPoint> BoustrophedonPlannerServer::executePlanPathInternal(
                                                    const boustrophedon_msgs::PlanMowingPathGoal& goal,
                                                    Parameters params)
{
  // std::string boundaexecutePlanPathInternalry_frame = goal->property.header.frame_id;
  last_status_.clear();

  ROS_INFO_STREAM("using Angle: " << params.stripe_angle_ * 180 / 3.14159265359 << " and Spacing: " << params.stripe_separation_);
  if (params.enable_orientation_)
  {
    params.stripe_angle_ = getStripeAngleFromOrientation(goal.robot_position);
  }

  Polygon polygon = fromBoundary(goal.property);
  if (!checkPolygonIsValid(polygon))
  {
    last_status_ = std::string("Boustrophedon planner does not work for polygons of "
                               "size "
                               "< 3.");
    return {};
  }
  if (!polygon.is_simple())
  {
    last_status_ = std::string("Boustrophedon planner only works for simple (non "
                               "self-intersecting) polygons.");
    return {};
  }
  Point robot_position;
  try
  {
    robot_position = fromPositionWithFrame(goal.robot_position, goal.property.header.frame_id);
  }
  catch (const tf::TransformException& ex)
  {
    last_status_ = std::string("Boustrophedon planner failed with a tf exception: ") + ex.what();
    return {};
  }
  std::vector<NavPoint> path;
  std::vector<NavPoint> outline_path;

  auto preprocess_transform = preprocessPolygon(polygon, robot_position, params.stripe_angle_);

  if (publish_polygons_)
  {
    initial_polygon_publisher_.publish(goal.property);
    preprocessed_polygon_publisher_.publish(convertCGALPolygonToMsg(polygon));
  }

  Polygon fill_polygon;
  if (!outline_planner_.addToPath(polygon, robot_position, outline_path, fill_polygon))
  {
    last_status_ = std::string("Boustrophedon planner failed because "
                               "outline_layer_count "
                               "was too large for the boundary.");
    return {};
  }

  if (!outline_path.empty())
  {
    if (params.stripes_before_outlines_)
    {
      path.push_back(outline_path[0]);
    }
    else
    {
      path.insert(path.end(), outline_path.begin(), outline_path.end());
    }
  }

  PolygonDecomposer polygon_decomposer{};
  // A print statement MUST be here, see issue #1586
  ROS_INFO_STREAM("Decomposing boundary polygon into sub-polygons...");
  polygon_decomposer.decompose(fill_polygon);
  std::vector<Polygon> sub_polygons = polygon_decomposer.getSubPolygons(robot_position);
  ROS_INFO_STREAM("Broke the boundary up into " << sub_polygons.size() << " sub-polygons");
  Polygon merged_polygon;
  Point start_position = robot_position;
  for (const auto& subpoly : sub_polygons)
  {
    // combine the new subpoly with the merged_polygon. If merged_polygon is empty, it returns the sub polygon
    merged_polygon = mergePolygons(merged_polygon, subpoly);

    // add the stripes to the path, using merged_polygon boundary to travel if necessary.
    striping_planner_.addToPath(merged_polygon, subpoly, robot_position, path);
  }
  if (params.travel_along_boundary_)
  {
    striping_planner_.addReturnToStart(merged_polygon, start_position, robot_position, path);
  }

  if (params.stripes_before_outlines_ && !outline_path.empty())
  {
    path.insert(path.end(), outline_path.begin(), outline_path.end());
  }

  postprocessPolygonAndPath(preprocess_transform, polygon, path);
  if (publish_path_points_)  // if we care about visualizing the planned path in plotJuggler
  {
    publishPathPoints(path);
    publishPolygonPoints(polygon);
  }
  // auto result = toResult(std::move(path), boundary_frame);
  return path;
}

boustrophedon_msgs::PlanMowingPathResult BoustrophedonPlannerServer::toResult(std::vector<NavPoint>&& path,
                                                                         const std::string& frame) const
{
  boustrophedon_msgs::PlanMowingPathResult result;
  result.plan.points.reserve(path.size());
  result.plan.header.stamp = ros::Time::now();
  result.plan.header.frame_id = frame;

  for (const auto& point : path)
  {
    boustrophedon_msgs::StripingPoint stripe_point{};
    stripe_point.point.x = point.point.x();
    stripe_point.point.y = point.point.y();
    stripe_point.point.z = 0;
    stripe_point.type = static_cast<uint8_t>(point.type);
    result.plan.points.emplace_back(stripe_point);
  }
  return result;
}

Polygon BoustrophedonPlannerServer::fromBoundary(const geometry_msgs::PolygonStamped& boundary) const
{
  Polygon polygon;
  for (const auto& point : boundary.polygon.points)
  {
    polygon.push_back(Point(point.x, point.y));
  }
  return polygon;
}

Point BoustrophedonPlannerServer::fromPositionWithFrame(const geometry_msgs::PoseStamped& pose,
                                                        const std::string& target_frame) const
{
  geometry_msgs::PoseStamped transformed_pose;
  transform_listener_.transformPose(target_frame, pose, transformed_pose);
  return { transformed_pose.pose.position.x, transformed_pose.pose.position.y };
}

bool BoustrophedonPlannerServer::convertStripingPlanToPath(boustrophedon_msgs::ConvertPlanToPath::Request& request,
                                                           boustrophedon_msgs::ConvertPlanToPath::Response& response)
{
  response.path.header.frame_id = request.plan.header.frame_id;
  response.path.header.stamp = request.plan.header.stamp;

  std::transform(request.plan.points.begin(), request.plan.points.end(), response.path.poses.begin(),
                 [&](const boustrophedon_msgs::StripingPoint& point) {
                   geometry_msgs::PoseStamped pose;
                   pose.header.frame_id = request.plan.header.frame_id;
                   pose.header.stamp = request.plan.header.stamp;
                   pose.pose.position = point.point;
                   pose.pose.orientation.x = 0.0;
                   pose.pose.orientation.y = 0.0;
                   pose.pose.orientation.z = 0.0;
                   pose.pose.orientation.w = 1.0;
                   return pose;
                 });
  return true;
}

geometry_msgs::PolygonStamped BoustrophedonPlannerServer::convertCGALPolygonToMsg(const Polygon& poly) const
{
  geometry_msgs::PolygonStamped stamped_poly;

  for (auto it = poly.vertices_begin(); it != poly.vertices_end(); it++)
  {
    geometry_msgs::Point32 point;
    point.x = float(it->x());
    point.y = float(it->y());
    point.z = float(0.0);
    stamped_poly.polygon.points.push_back(point);
  }

  stamped_poly.header.frame_id = "map";
  stamped_poly.header.stamp = ros::Time::now();
  return stamped_poly;
}

bool BoustrophedonPlannerServer::checkPolygonIsValid(const Polygon& poly) const
{
  return !(poly.size() < 3);  // expand later if we find more cases of invalid polygons
}

// get the yaw from the robot_position part of the given goal
double BoustrophedonPlannerServer::getStripeAngleFromOrientation(const geometry_msgs::PoseStamped& robot_position)
{
  tf::Quaternion quat(robot_position.pose.orientation.x, robot_position.pose.orientation.y,
                      robot_position.pose.orientation.z, robot_position.pose.orientation.w);
  tf::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO_STREAM("Got Striping Angle from Orientation: " << yaw);
  // TODO: Recalibrate the IMU so that we can get rid of this constant below.
  return yaw + 1.57079632679;  // Adds PI / 2 to account for incorrect IMU calibration / reference vector
}

// publishes the path points one at a time for visualization in plotJuggler
void BoustrophedonPlannerServer::publishPathPoints(const std::vector<NavPoint>& path) const
{
  for (NavPoint point : path)
  {
    geometry_msgs::PointStamped stripe_point{};
    stripe_point.header.stamp = ros::Time::now();
    stripe_point.point.x = point.point.x();
    stripe_point.point.y = point.point.y();
    path_points_publisher_.publish(stripe_point);
    ros::spinOnce();
  }
}

// publishes the polygon points one at a time for visualization in plotJuggler
void BoustrophedonPlannerServer::publishPolygonPoints(const Polygon& poly) const
{
  for (auto it = poly.vertices_begin(); it != poly.vertices_end(); it++)
  {
    geometry_msgs::PointStamped point;
    point.header.stamp = ros::Time::now();
    point.point.x = float(it->x());
    point.point.y = float(it->y());
    point.point.z = float(0.0);
    polygon_points_publisher_.publish(point);
    ros::spinOnce();
  }
}

void BoustrophedonPlannerServer::publishCurrentParameters() const
{
  boustrophedon_msgs::PlanParameters params;

  params.cut_spacing = params_.stripe_separation_;
  params.cut_angle_degrees = params_.stripe_angle_;
  params.stripes_before_outlines = params_.stripes_before_outlines_;
  params.enable_stripe_angle_orientation = params_.enable_orientation_;

  if (params_.intermediary_separation_ != std::numeric_limits<double>::max())
    params.intermediary_separation = params_.intermediary_separation_;
  else
    params.intermediary_separation = 0.0;
  params.travel_along_boundary = params_.travel_along_boundary_;
  params.points_per_turn = params_.points_per_turn_;
  params.turn_start_offset = params_.turn_start_offset_;
  params.repeat_boundary = params_.repeat_boundary_;
  params.outline_clockwise = params_.outline_clockwise_;
  params.skip_outlines = params_.skip_outlines_;
  params.outline_layer_count = params_.outline_layer_count_;

  if (params_.enable_bulb_turns_)
  {
    params.turn_type = boustrophedon_msgs::PlanParameters::TURN_BULB;  // todo replace with named constant
  }
  if (params_.enable_full_u_turns_)
  {
    params.turn_type = boustrophedon_msgs::PlanParameters::TURN_FULL_U;
  }
  else if (params_.enable_half_y_turns_)
  {
    params.turn_type = boustrophedon_msgs::PlanParameters::TURN_HALF_Y;
  }
  else
  {
    params.turn_type = boustrophedon_msgs::PlanParameters::TURN_BOUNDARY;
  }

  default_params_publisher_.publish(params);
}
