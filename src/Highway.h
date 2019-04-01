#ifndef __HIGHWAY_H__
#define __HIGHWAY_H__

#include <vector>
#include <cmath>
#include "helpers.h"

struct HighwayParameters 
{
    size_t LaneCount;
    double LaneWidth;
    double SpeedLimit;
    double WrapAroundPosition;

    size_t WhichLane(double d) const
    {
        return (size_t)std::round(clamp((d/LaneWidth)-0.5, 0, LaneCount-1));
    }
};

struct HighwayWaypoint 
{
    double x;   // Cartesian x coordinate relative to map
    double y;   // Cartesian y coordinate relative to map
    double s;   // Frenet arc-length coordinate
    double dx;  // Frenet unit normal x component
    double dy;  // Frenet unit normal y component
};

class HighwayCoordinates
{
public:
    HighwayCoordinates(HighwayParameters, const std::vector<HighwayWaypoint>&);

    // Frenet2D MapToHighway(Vector2D mapLoc, double heading) const;
    // Vector2D HighwayToMap(Frenet2D hwyLoc) const;
    // std::vector<Vector2D> HighwayToMap(const std::vector<Frenet2D> &hwyLocs) const;

    Frenet2D MapToHighway(Pose2D) const;
    Pose2D HighwayToMap(Frenet2D) const;
    std::vector<Pose2D> HighwayToMap(const std::vector<Frenet2D>&) const;

private:
    size_t NearestWaypoint(Vector2D mapLoc) const;
    size_t NextWaypoint(Pose2D) const;
    size_t NextWaypoint(Frenet2D) const;

    Pose2D WaypointPose(size_t index) const;
    double WaypointPosition(size_t index) const;

    HighwayParameters _hwyParams;
    std::vector<HighwayWaypoint> _waypoints;
    std::vector<double> _cumDist;
};


#endif // __HIGHWAY_H__