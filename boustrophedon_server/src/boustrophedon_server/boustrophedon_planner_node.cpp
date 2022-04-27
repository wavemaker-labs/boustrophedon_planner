#include <ros/ros.h>
#include "boustrophedon_server/boustrophedon_planner_server.h"
#include <sentry.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "boustrophedon_planner_node");
  sentry_options_t *options = sentry_options_new();
  sentry_options_set_dsn(options, "https://1cb1bee385a84f38bd128de9c1d23baf@o1162461.ingest.sentry.io/6301844");
  sentry_init(options);

  BoustrophedonPlannerServer server;

  ros::spin();
  sentry_shutdown();
  return 0;
}
