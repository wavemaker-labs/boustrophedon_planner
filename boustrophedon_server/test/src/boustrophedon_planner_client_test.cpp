#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <CGAL/create_offset_polygons_2.h>

#define RYML_SINGLE_HDR_DEFINE_NOW
#include "rapidyaml-0.4.1.hpp"

#include <fstream>
#include <sstream>
#include <string>


#include <boustrophedon_msgs/PlanMowingPathAction.h>
#include <boustrophedon_msgs/PlanMowingPathParamAction.h>
#include <boustrophedon_msgs/ConvertPlanToPath.h>
#include <boustrophedon_msgs/PlanParameters.h>
#include "boustrophedon_server/cgal_utils.h"

using namespace ::testing;
using namespace ::boustrophedon_msgs;

namespace testutil {

    inline std::string file_contents_to_string(const std::string &filename) {
        //clunky, obscure, but reliable boilerplate code
        std::ifstream t(filename);
        t.seekg(0, std::ios::end);
        size_t size = t.tellg();
        std::string buffer(size, ' ');
        t.seekg(0);
        t.read(&buffer[0], size);
        return buffer;
    }

    PlanMowingPathActionGoal get_goal_from_file(const std::string &filename) {
        PlanMowingPathActionGoal action_msg;

        std::string contents = file_contents_to_string(filename);
        ryml::Tree tree = ryml::parse_in_arena(ryml::to_csubstr(contents)); // immutable (csubstr) overload

        auto root = tree.rootref();

        // if (!root.is_seq()) throw std::domain_error("unsupported local map file");
        std::cerr << "root: number of siblings = " << root.num_siblings();
        std::cerr << ", num children = " << root.num_children() << std::endl;


        std::stringstream debug_poly;
        for (auto const &node : root.children()) {
            geometry_msgs::Point32 point;
            node["easting"] >> point.x;
            node["northing"] >> point.y;

            action_msg.goal.property.polygon.points.push_back(point);
            debug_poly << "[" << point.x << ", " << point.y << "], ";
        }
        auto poly_str = debug_poly.str();
        poly_str = poly_str.substr(0, poly_str.size() - 2);
        std::cerr << std::endl << "[" << poly_str << "]\n -- end of polygon -- " << std::endl;

        if (action_msg.goal.property.polygon.points.empty()) return {};

        auto const &first_point = action_msg.goal.property.polygon.points[0];
        action_msg.goal.property.header.frame_id = "map";
        action_msg.goal.property.header.stamp = ros::Time::now();
        action_msg.goal.robot_position.pose.position.x = first_point.x;
        action_msg.goal.robot_position.pose.position.y = first_point.y;
        action_msg.goal.robot_position.pose.orientation.w = 1.0;
        action_msg.goal.robot_position.header.frame_id = "map";
        action_msg.goal.property.header.stamp = ros::Time::now();

        return action_msg;
    }

    inline bool pointInPolygon(const Point data_point, const std::vector<Point> &polygon)
    {
        int i, j, nvert = static_cast<int>(polygon.size());
        bool flag = false;

        for(i = 0, j = nvert - 1; i < nvert; j = i++)
        {
            if(((polygon[i].y() >= data_point.y()) != (polygon[j].y() >= data_point.y())) &&
                (data_point.x() <= (polygon[j].x() - polygon[i].x()) * (data_point.y() - polygon[i].y()) / (polygon[j].y() - polygon[i].y()) + polygon[i].x()))
                flag = !flag;
        }

        return flag;
    }
}

namespace testtypes {
    // So that we can test any combination of the commonly used settings
    using TestParamTypes = std::tuple<int /*angles*/, double /*spacings*/, const char * /*maps*/>;

    struct paramNameStringGen {
        std::string operator()(const ::testing::TestParamInfo<TestParamTypes>& info) {
        // Can use info.param here to generate the test suffix
        std::stringstream name;
        auto map_name = std::string(std::get<2>(info.param));
        auto angle_str = std::to_string(std::get<0>(info.param));
        auto spacing_str = std::to_string(std::get<1>(info.param));
        std::replace_if(spacing_str.begin(), spacing_str.end(),
            [](char c) { return !std::isalnum(c); }, '_');
        std::replace_if(map_name.begin(), map_name.end(),
            [](char c) { return !std::isalnum(c); }, '_');
        name << angle_str << "_" << spacing_str << "_"
            << map_name.substr(0, map_name.size() - 5);
        return name.str();
        }
    };
}

class BplannerShould : public testing::TestWithParam<testtypes::TestParamTypes> {
public:
    BplannerShould() : planning_client_("plan_path"), planning_client_with_param_("config_and_plan_path") {
        ;
    }

protected:
    actionlib::SimpleActionClient<PlanMowingPathAction> planning_client_;
    actionlib::SimpleActionClient<PlanMowingPathParamAction> planning_client_with_param_;
    PlanMowingPathActionGoal current_goal_;
    PlanMowingPathParamActionGoal current_goal_with_params_;
    int angle_;
    double spacing_;
    bool initial_simple_goal_succeeded_{false};
    bool initial_bundled_goal_succeeded_{false};

protected:
    inline void SetUp() override {
        ASSERT_TRUE(planning_client_.waitForServer(ros::Duration(5))) << "Action server is down!";
        ASSERT_TRUE(planning_client_with_param_.waitForServer(ros::Duration(5))) << "Action server is down!";
        // pre-load the current test case goal
        auto current_param = GetParam();

        std::string map_dir = ros::package::getPath("boustrophedon_example") + "/cutting/mowing/maps/";
        EXPECT_NO_THROW(current_goal_ = testutil::get_goal_from_file(map_dir + std::string(std::get<2>(current_param))));
        angle_ = std::get<0>(current_param);
        spacing_ = std::get<1>(current_param);
    }

    inline void TearDown() override {

    }

    inline PlanParameters create_test_param() {
        PlanParameters test_params;
        test_params.allow_points_outside_boundary = false;
        test_params.enable_stripe_angle_orientation = false;
        test_params.intermediary_separation = 0.0;
        test_params.outline_clockwise = false;
        test_params.outline_layer_count = 0;
        test_params.points_per_turn = 0;
        test_params.repeat_boundary = false;
        test_params.skip_outlines = false;
        test_params.stripes_before_outlines = false;
        test_params.travel_along_boundary = false;
        test_params.turn_start_offset = 0.0;
        test_params.u_turn_radius = 1.0;
        test_params.turn_type = PlanParameters::TURN_BOUNDARY;
        test_params.cut_angle_degrees = angle_;
        test_params.cut_spacing = spacing_;

        std::cerr << "Current params: angle = " << angle_ << ", spacing = " << spacing_ << std::endl;
        return test_params;
    }

    inline StripingPlan submit_goal_and_get_result() {
        current_goal_with_params_.goal.property = current_goal_.goal.property;
        current_goal_with_params_.goal.robot_position = current_goal_.goal.robot_position;
        current_goal_with_params_.goal.parameters = create_test_param();
        planning_client_with_param_.sendGoalAndWait(current_goal_with_params_.goal, ros::Duration(5));

        auto result = planning_client_with_param_.getResult();
        return result->plan;
    }

    inline void check_plan_is_within_polygon(const StripingPlan &plan) {
        std::stringstream debug_plan;
        Polygon polygon, *safety_polygon{nullptr};

        // Let's try a tight boundary of 10cm
        constexpr double safety_boundary = 0.1;
        for (const auto& point : current_goal_.goal.property.polygon.points)
        {
            polygon.push_back(Point(point.x, point.y));
        }
        if (polygon.is_clockwise_oriented())
        {
            polygon.reverse_orientation();
        }

        auto outer_offset_polygons = CGAL::create_exterior_skeleton_and_offset_polygons_2(safety_boundary, polygon);
        std::cerr << "Offset polygons: " << outer_offset_polygons.size() << std::endl;
        for (auto const offset_polygon : outer_offset_polygons)
        {
            auto container = (*offset_polygon).vertices_circulator().container();
            std::cerr << "Polygon size: " << container->size() << std::endl;
            for (auto const &point : *container)
            {
                debug_plan << "[" << point.x() << ", " << point.y() << "], ";
            }
            std::cerr << debug_plan.str() << std::endl << std::endl;
            std::stringstream().swap(debug_plan);
            if (container->size() == polygon.size()) {
                safety_polygon = offset_polygon.get();
            }
        }

        // asserts
        ASSERT_TRUE(safety_polygon != nullptr) << "Safety_polygon was not found!";
        for (auto const &point : plan.points) {
            Point p(point.point.x, point.point.y);
            debug_plan << "[" << p.x() << ", " << p.y() << "], ";
            EXPECT_NE(safety_polygon->bounded_side(p), CGAL::ON_UNBOUNDED_SIDE) << "Point is outside: " << point.point.x << ", " << point.point.y;
            // EXPECT_TRUE(testutil::pointInPolygon(p, poly)) << "Point is outside: " << point.point.x << ", " << point.point.y;
        }
        auto plan_str = debug_plan.str();
        plan_str = plan_str.substr(0, plan_str.size() - 2);
        // std::cerr << std::endl << "[" << plan_str << "]\n -- end of plan -- " << std::endl;
    }
};


TEST_P(BplannerShould, makeAPlanUsingPreloadedParam) {
    if (initial_simple_goal_succeeded_)
        GTEST_SKIP();
    ASSERT_FALSE(current_goal_.goal.property.polygon.points.empty());
    planning_client_.sendGoalAndWait(current_goal_.goal, ros::Duration(5));
    auto result = planning_client_.getResult();
    ASSERT_NE(result->plan.points.size(), 0);
    initial_simple_goal_succeeded_ = true;
}

TEST_P(BplannerShould, makeAPlanUsingBundledParam) {
    if (initial_bundled_goal_succeeded_)
        GTEST_SKIP();
    ASSERT_FALSE(current_goal_.goal.property.polygon.points.empty());
    auto result = submit_goal_and_get_result();
    ASSERT_NE(result.points.size(), 0);
    initial_bundled_goal_succeeded_ = true;
}

TEST_P(BplannerShould, noPointOutsideSafetyBoundaries) {
    // assemble and activate test
    auto result = submit_goal_and_get_result();
    // do checks
    check_plan_is_within_polygon(result);
}

//**
// Below are the actual input test values
//

// Tweak this with the valid range of angles allowed in UI
constexpr int MINIMUM_ANGLE = 0;
constexpr int MAXIMUM_ANGLE = 180;
constexpr int ANGLE_STEP = 10;

// Tweak this with the valid range of spacings allowed in UI
constexpr double MINIMUM_SPACING = 1.0;
constexpr double MAXIMUM_SPACING = 1.8;
constexpr double SPACING_STEP = 0.2;

INSTANTIATE_TEST_SUITE_P(CommonSettingsTest, BplannerShould,
  testing::Combine(
    testing::Range<int>(MINIMUM_ANGLE, MAXIMUM_ANGLE, ANGLE_STEP),
    testing::Range<double>(MINIMUM_SPACING, MAXIMUM_SPACING, SPACING_STEP),
    // Add maps that might be interesting to check for issues
    testing::Values(
        "scholl_large.yaml"
    )
  ),
  testtypes::paramNameStringGen()
);

INSTANTIATE_TEST_SUITE_P(LocalMapsTest, BplannerShould,
  testing::Combine(
    testing::Values(MINIMUM_ANGLE, MAXIMUM_ANGLE, ANGLE_STEP),
    testing::Values(MINIMUM_SPACING, MAXIMUM_SPACING),
    // Add maps that might be interesting to check for issues
    testing::Values(
        "helipad-L.yaml",
        "helipad_section_B.yaml"
    )
  ),
  testtypes::paramNameStringGen()
);

INSTANTIATE_TEST_SUITE_P(UtmMapsTest, BplannerShould,
  testing::Combine(
    testing::Range<int>(MINIMUM_ANGLE, MAXIMUM_ANGLE, ANGLE_STEP),
    testing::Values(MINIMUM_SPACING),
    testing::Values(
    // Add maps that might be interesting to check for issues
        "utm_very_small_area.yaml"
    )
  ),
  testtypes::paramNameStringGen()
);
