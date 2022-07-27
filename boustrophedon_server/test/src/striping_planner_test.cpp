#include <gtest/gtest.h>

#include "boustrophedon_server/striping_planner.h"
#include "boustrophedon_server/cgal_utils.h"

TEST(StripingPlannerShould, makeStripePlans)
{
    StripingPlanner class_under_test;
    Polygon p;
    p.push_back(Point(0, 0));
    p.push_back(Point(3, 0));
    p.push_back(Point(3, 3));
    p.push_back(Point(0, 3));

    std::vector<NavPoint> new_path_section;
    Point initial_position(0, 0);
    class_under_test.fillPolygon(p, new_path_section, initial_position);
    // There is no sanity check on what the plan actually is. Let's do that on the next test cases
    EXPECT_GT(new_path_section.size(), 0);
}

TEST(StripingPlannerShould, DISABLED_onlyFillInnerArea)
{
    FAIL();
}

TEST(StripingPlannerShould, DISABLED_StoreCorrectParameters)
{
    FAIL();
}

TEST(StripingPlannerShould, DISABLED_CategorizeWaypointTypes)
{
    FAIL();
}

TEST(StripingPlannerShould, DISABLED_AllowConcavePolygons)
{
    FAIL();
}

TEST(StripingPlannerShould, DISABLED_CorrectlyOrientCornRows)
{
    FAIL();
}

TEST(StripingPlannerShould, DISABLED_SpaceCornRowsCorrectly)
{
    FAIL();
}
