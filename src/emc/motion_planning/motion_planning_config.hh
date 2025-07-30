#ifndef MOTION_PLANNING_CONFIG_HH
#define MOTION_PLANNING_CONFIG_HH

#include "motion.h"

extern emcmot_config_t* emcmotConfig;

int updateTPConfig();

double findMaxVelocityWithFilteringActive(double eff_radius);

#endif // MOTION_PLANNING_CONFIG_HH
