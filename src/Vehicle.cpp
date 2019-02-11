#include "Vehicle.h"
#include <limits>

using namespace std;



Vehicle::Vehicle(RoadParameters roadParams, Kinematics longKinematics, double lateralPos) :
    _roadParams(roadParams),
    _longKinematics(longKinematics),
    _lateralPos(lateralPos)
{
}

vector<Vehicle> Vehicle::GeneratePredictions(unsigned int horizon)
{
    vector<Vehicle> predictions(horizon);

    for (unsigned int i=0; i < horizon; i++)
    {
        predictions[i] = Vehicle(_roadParams, _longKinematics.Predict(i+1), _lateralPos);
    }

    return predictions;
}

// ****************************************************************************

static const size_t ID_EGO = numeric_limits<size_t>::max();

EgoVehicle::EgoVehicle(
    RoadParameters roadParams, 
    DrivingParameters drivingParams, 
    Kinematics longKinematics, 
    double lateralPos) : Vehicle(roadParams, longKinematics, lateralPos),
{
    _drivingParams = drivingParams;
    _currentBehavior = KeepLane;
}

void EgoVehicle::UpdateState(Kinematics longKinematics, double lateralPos)
{
    _longKinematics = longKinematics;
    _lateralPos = lateralPos;
}

vector<Behavior> EgoVehicle::SuccessorStates() const
{
    static unordered_map<Behavior, vector<Behavior>> successors;

    if (successors.empty())
    {
        successors[KeepLane] = { 
            KeepLane, 
            PrepareLaneChangeLeft, 
            PrepareLaneChangeRight };

        successors[PrepareLaneChangeLeft] = { 
            KeepLane, 
            PrepareLaneChangeLeft, 
            LaneChangeLeft };

        successors[PrepareLaneChangeRight] = { 
            KeepLane, 
            PrepareLaneChangeRight, 
            LaneChangeRight };

        successors[LaneChangeLeft] = { 
            KeepLane, 
            LaneChangeLeft };

        successors[LaneChangeRight] = { 
            KeepLane, 
            LaneChangeRight };
    }

    return successors[_currentBehavior];
}

bool EgoVehicle::CreateConstantSpeedTrajectory(
    const VehiclePredictions &predictions,
    VehicleTrajectory &trajectory) const
{
    trajectory = {
        Vehicle(_ID, _lane, _kinematics),
        Vehicle(_ID, _lane, _kinematics.Predict(1.0))
    };
    return true;
}

bool EgoVehicle::CreateKeepLaneTrajectory(
    const VehiclePredictions &predictions,
    VehicleTrajectory &trajectory) const
{

}

bool EgoVehicle::CreatePrepareLaneChangeTrajectory(
    LaneChangeDirection direction,
    const VehiclePredictions &predictions,
    VehicleTrajectory &trajectory) const
{

}

bool EgoVehicle::CreateLaneChangeTrajectory(
    LaneChangeDirection direction,
    const VehiclePredictions &predictions,
    VehicleTrajectory &trajectory) const
{

}

bool EgoVehicle::GetVehicleAhead(
    const VehiclePredictions &predictions, 
    size_t lane, 
    Vehicle &nearestAhead) 
{
    for (auto it = predictions.begin(); it != predictions.end(); it++)
    {
        Vehicle other = it->second[0];
        size_t otherLane = other.GetLane();
        double otherPos = other.GetLongitudinalPosition();

        if (otherLane == lane && otherPos > )
    }
}

bool EgoVehicle::GetVehicleBehind(
    const VehiclePredictions &predictions, 
    size_t lane, 
    Vehicle &nearestBehind)
{

}