// -*- mode: c++; indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4; -*-
#include "ConfigurationSpace.h"
#include "RRTPathPlanner.h"
#include "RRTRobot.h"
#define _USE_MATH_DEFINES // for MSVC
#include <math.h>

using namespace PathEngine;

inline double theta_limit(double theta)
{
    while (theta >= 2*M_PI) theta -= 2*M_PI;
    while (theta < 0) theta += 2*M_PI;
    return theta;
}

Configuration RRTRobot::interpolate(const Configuration& from, 
                                     const Configuration& to,
                                     double ratio) const
{
    ConfigurationSpace *cspace = planner_->getConfigurationSpace();
    Configuration cfg(cspace->size());
    for (unsigned int i=0; i<cfg.size(); i++){
        if (cspace->unboundedRotation(i)){
            double dth = to.value(i) - from.value(i);
            dth = theta_limit(dth);
            if (dth > M_PI){
                dth = dth - 2*M_PI;
            }
            cfg.value(i) = from.value(i) + ratio*dth;
        }else{
            cfg.value(i) = (1-ratio)*from.value(i) + ratio*to.value(i);
        }
    }
    
    return cfg;
}

double RRTRobot::distance(const Configuration& from, const Configuration& to) const
{
    ConfigurationSpace *cspace = planner_->getConfigurationSpace();
    double v=0, d;
    for (unsigned int i=0; i<cspace->size(); i++){
        if (cspace->unboundedRotation(i)){
            double dth = theta_limit(to.value(i) - from.value(i));
            if (dth > M_PI) dth = 2*M_PI - dth;
            d = cspace->weight(i)*dth;
        }else{
            d = cspace->weight(i)*(to.value(i) - from.value(i));
        }
        v += d*d;
    }
    return sqrt(v);
}



