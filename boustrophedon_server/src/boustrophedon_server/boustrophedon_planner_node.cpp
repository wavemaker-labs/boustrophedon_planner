#include <ros/ros.h>
#include "boustrophedon_server/boustrophedon_planner_server.h"
#include <sentry.h>
#include <cstdlib>

namespace sentry_cpp {
  class Sentry final {
    public:
      Sentry() {
        initialize();
      }

      Sentry(Sentry const &) = delete;
      Sentry(Sentry &&) = delete;
      Sentry &operator=(Sentry const &) = delete;
      Sentry &operator=(Sentry &&) = delete;

      ~Sentry() {
        shutdown();
      }

      // Initialize sentry and ensure it's only initialized once
      inline int initialize() {
        if (is_enabled())
        {
          // this is a custom return value. sentry_init only returns 1 on fail case.
          constexpr int k_sentry_already_initialized{-1};
          return k_sentry_already_initialized;
        }

        sentry_options_t *options = sentry_options_new();
        auto sentry_dsn = getenv("SENTRY_CPP_DSN");
        auto sentry_env = getenv("SENTRY_CPP_ENV");

        if (sentry_env)
        {
          sentry_options_set_environment(options, sentry_env);
        }

        if (sentry_dsn)
        {
          Sentry::sentry_enabled = true;
          sentry_options_set_dsn(options, sentry_dsn);
          sentry_init(options);
        }
      }

      // Check if sentry is initialized
      inline bool is_enabled() const {
        return Sentry::sentry_enabled;
      }

      //Shutdown sentry as needed
      inline void shutdown() {
        if (Sentry::sentry_enabled)
        {
          sentry_shutdown();
        }
      }

    private:
      inline static bool sentry_enabled = false;
  };

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "boustrophedon_planner_node");
  sentry_cpp::Sentry instance;

  BoustrophedonPlannerServer server;

  ros::spin();

  return 0;
}
