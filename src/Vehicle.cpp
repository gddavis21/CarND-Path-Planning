#include "Vehicle.h"

using namespace std;

Vehicle::Vehicle(
    const HighwayParameters &hwyParams, 
    const Kinematics &longKinematics, 
    double lateralPos, 
    double time)
{
    _valid = true;
    _hwyParams = hwyParams;
    _longKinematics = longKinematics;
    _lateralPos = lateralPos;
    _time = time;
}

Vehicle::Vehicle()
{
    _valid = false;
}

Vehicle Vehicle::Predict(double dt) const
{
    return Vehicle(_hwyParams, Kinematics().Predict(dt), LateralPosition(), Time()+dt);
}

double Vehicle::DistanceBehind(const Vehicle &other) const
{
    double dist = other.Position() - this->Position();
    return (dist < 0.0) ? (dist + _hwyParams.WrapAroundPosition) : dist;
}
