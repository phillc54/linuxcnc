#include "motion_planning_config.hh"
#include "usrmotintf.h"
#include <math.h>
#include <algorithm>

static emcmot_config_t emcmotConfig_storage{};
emcmot_config_t* emcmotConfig = &emcmotConfig_storage;

int updateTPConfig()
{
    return usrmotReadEmcmotConfig(&emcmotConfig_storage);
}

double findMaxVelocityWithFilteringActive(double eff_radius)
{
    double window_size_sec = emcmotConfig->joint_filter_cfg.window_size_sec;
    if ( window_size_sec <= emcmotConfig->servoCycleTime || emcmotConfig->joint_filter_cfg.window_size_sec == 0.0) {
        return 1e99;
    }
    // Need to specify a non-zero tolerance with
    double max_vel = sqrt(std::max(emcmotConfig->joint_filter_cfg.path_filtering_tolerance, 1e-6) * 24. * eff_radius)/(emcmotConfig->joint_filter_cfg.window_size_sec);
    return max_vel;
}
