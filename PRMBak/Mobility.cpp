#include "RRTPathPlanner.h"
#include "Mobility.h"

using namespace PathEngine;

double Mobility::interpolationDistance_ = 0.1;

bool Mobility::isReachable(Configuration& from, Configuration& to,
                           bool checkCollision, CollisionChecker* col_checker,
													 bool docheckstart, bool docheckend) const
{
    std::vector<Configuration> path;
    if (!getPath(from, to, path)) return false;
#if 0
    std::cout << "isReachable(" << from << ", " << to << ")" << std::endl;
    for (unsigned int  i = 0; i<path.size(); i++){
        std::cout << i << ":" << path[i] << std::endl;
    }
#endif
    if (checkCollision){
			if (col_checker == NULL) {
        return !planner_->checkCollision(path, docheckstart);
			} else {
				return !col_checker->checkCollision(path, docheckstart);
			}
    }else{
        return true;
    }
}

bool Mobility::getPath(Configuration &from, Configuration &to,
                       std::vector<Configuration>& o_path) const
{
    o_path.push_back(from);
    
    unsigned int n = (unsigned int)(distance(from, to)/interpolationDistance())+1;
    Configuration pos(planner_->getConfigurationSpace()->size());
    for (unsigned int i=1; i<n; i++){
        pos = interpolate(from, to, ((double)i)/n);
        o_path.push_back(pos);
    }
    o_path.push_back(to);
      
    return true;
}



