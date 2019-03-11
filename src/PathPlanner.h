#ifndef __PATH_PLANNER_H__
#define __PATH_PLANNER_H__

#include <memory>
#include <vector>
#include "helpers.h"
#include "Highway.h"
#include "Vehicle.h"
#include "BehaviorPlanner.h"

class PathPlanner
{
public:
    PathPlanner(
        const DrivingParameters&, 
        const HighwayParameters&, 
        const std::vector<HighwayWaypoint>&);

    struct EgoVehicleState
    {
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
    };

    struct OtherVehicleState
    {
        size_t ID;
        double x;
        double y;
        double vx;
        double vy;
        double s;
        double d;
    };

    std::vector<Vector2D> PlanNextPath(
        const std::vector<Vector2D> &previousPath,
        const EgoVehicleState&,
        const std::vector<OtherVehicleState>&);

private:
    HighwayParameters _highwayParams;
    std::unique_ptr<HighwayCoordinates> _highwayCoords;
    std::unique_ptr<BehaviorPlanner> _behaviorPlanner;
};

#endif // __PATH_PLANNER_H__