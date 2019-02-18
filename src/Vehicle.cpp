#include "Vehicle.h"
#include <limits>
#include <algorithm>

using namespace std;



Vehicle::Vehicle(RoadParameters roadParams, Kinematics longKinematics, double lateralPos) :
    _roadParams(roadParams),
    _longKinematics(longKinematics),
    _lateralPos(lateralPos)
{
}

// vector<Vehicle> Vehicle::GeneratePredictions(unsigned int horizon)
// {
//     vector<Vehicle> predictions(horizon);

//     for (unsigned int i=0; i < horizon; i++)
//     {
//         predictions[i] = Vehicle(_roadParams, _longKinematics.Predict(i+1), _lateralPos);
//     }

//     return predictions;
// }

VehiclePtr Vehicle::Predict(double dt) const
{
    return VehiclePtr(new Vehicle(_roadParams, _longKinematics.Predict(dt), _lateralPos));
}

double Vehicle::DistanceAhead(const Vehicle &other) const
{
    double dist = other.GetLongitudinalPosition() - this->GetLongitudinalPosition();
    return (dist < 0.0) ? (dist + _roadParams.WrapAroundPosition) : dist;
}

double Vehicle::DistanceBehind(const Vehicle &other) const
{
    double dist = this->GetLongitudinalPosition() - other.GetLongitudinalPosition();
    return (dist < 0.0) ? (dist + _roadParams.WrapAroundPosition) : dist;
}

// ****************************************************************************

static const size_t ID_EGO = numeric_limits<size_t>::max();

EgoVehicle::EgoVehicle(
    DrivingParameters drivingParams, 
    RoadParameters roadParams, 
    Kinematics longKinematics, 
    double lateralPos) : Vehicle(roadParams, longKinematics, lateralPos)
{
    _drivingParams = drivingParams;
    _currentBehavior = KeepLane;
}

// void EgoVehicle::UpdateState(Kinematics longKinematics, double lateralPos)
// {
//     _longKinematics = longKinematics;
//     _lateralPos = lateralPos;
// }

VehicleTrajectory EgoVehicle::PlanBehavior(const VehiclePredictions &predictions, double dt)
{
    vector<Behavior> behaviors;
    vector<VehicleTrajectory> trajectories;
    vector<double> costs;

    // next trajectory always starts with current state
    VehiclePtr currentState(new Vehicle(*this));

    // compute candidate trajectories & associated costs
    for (Behavior behavior: SuccessorBehaviors())
    {
        VehiclePtr nextState = GenerateTrajectory(behavior, predictions, dt);

        if (nextState)
        {
            VehicleTrajectory traj = { currentState, nextState };

            behaviors.push_back(behavior);
            trajectories.push_back(traj);
            costs.push_back(TrajectoryCost(traj, predictions));
        }
    }

    // find lowest cost
    size_t bestIndex = std::min_element(costs.begin(), costs.end()) - costs.begin();

    // transition to lowest-cost behavior
    _currentBehavior = behaviors[bestIndex];

    // return lowest-cost trajectory
    return trajectories[bestIndex];
}

vector<Behavior> EgoVehicle::SuccessorBehaviors() const
{
    size_t currentLane = GetLane();
    bool inLeftLane = (currentLane == 0);
    bool inRightLane = (currentLane == _roadParams.LaneCount - 1);

    vector<Behavior> successors;

    switch (_currentBehavior)
    {
        case KeepLane:
            successors.push_back(KeepLane);
            if (!InLeftLane())
                successors.push_back(PrepareLaneChangeLeft);
            if (!InRightLane())
                successors.push_back(PrepareLaneChangeRight);
            break;

        case PrepareLaneChangeLeft:
            successors.push_back(KeepLane);
            successors.push_back(PrepareLaneChangeLeft);
            successors.push_back(LaneChangeLeft);
            break;

        case PrepareLaneChangeRight:
            successors.push_back(KeepLane);
            successors.push_back(PrepareLaneChangeRight);
            successors.push_back(LaneChangeRight);
            break;

        case LaneChangeLeft:
            successors.push_back(KeepLane);
            successors.push_back(LaneChangeLeft);
            break;

        case LaneChangeRight:
            successors.push_back(KeepLane);
            successors.push_back(LaneChangeRight);
            break;
    }

    return successors;
}

VehiclePtr EgoVehicle::GenerateTrajectory(
    Behavior behavior, 
    const VehiclePredictions &predictions,
    double dt) const
{
    const bool LEFT = true;
    const bool RIGHT = false;

    switch (behavior)
    {
        case KeepLane:                return KeepLaneTrajectory(predictions, dt);
        case PrepareLaneChangeLeft:   return PrepareLaneChangeTrajectory(LEFT, predictions, dt);
        case PrepareLaneChangeRight:  return PrepareLaneChangeTrajectory(RIGHT, predictions, dt);
        case LaneChangeLeft:          return LaneChangeTrajectory(LEFT, predictions, dt);
        case LaneChangeRight:         return LaneChangeTrajectory(RIGHT, predictions, dt);
    }
    
    return VehiclePtr();
}

VehiclePtr EgoVehicle::ConstantSpeedTrajectory(
    const VehiclePredictions &predictions, 
    double dt) const
{
    return Predict(dt);
}

VehiclePtr EgoVehicle::KeepLaneTrajectory(
    const VehiclePredictions &predictions, 
    double dt) const
{
    Kinematics kin = GetKinematicsForLane(predictions, GetLane(), dt);
    return VehiclePtr(new Vehicle(_roadParams, kin.Predict(dt), _lateralPos));
}

VehiclePtr EgoVehicle::PrepareLaneChangeTrajectory(
    bool moveLeft, 
    const VehiclePredictions &predictions,
    double dt) const
{
    size_t currentLane = GetLane();
    size_t targetLane = moveLeft ? (currentLane - 1) : (currentLane + 1);
    Kinematics currentLaneKinematics = GetKinematicsForLane(predictions, currentLane, dt);
    Kinematics targetLaneKinematics = GetKinematicsForLane(predictions, targetLane, dt);
    Kinematics nextKinematics;
    
    double gapBehind = numeric_limits<double>::max();
    size_t behindID;

    if (TryGetVehicleBehind(predictions, currentLane, behindID))
    {
        VehiclePtr vehBehind = predictions[behindID][0];
        double gapBehind = DistanceBehind(*vehBehind) / GetLongitudinalVelocity();
    }

    if (gapBehind < 5 || currentLaneKinematics.velocity < targetLaneKinematics.velocity)
        nextKinematics = currentLaneNextKinematics;
    else
        nextKinematics = targetLaneNextKinematics;

    return VehiclePtr(new Vehicle(_roadParams, nextKinematics, _lateralPos));
}

VehiclePtr EgoVehicle::LaneChangeTrajectory(
    bool moveLeft, 
    const VehiclePredictions &predictions,
    double dt) const
{
    double currentVelocity = GetLongitudinalVelocity();
    size_t currentLane = GetLane();
    size_t targetLane = moveLeft ? (currentLane - 1) : (currentLane + 1);

    size_t aheadID, behindID;

    // detect collision with car ahead in target lane
    if (TryGetVehicleAhead(predictions, targetLane, aheadID))
    {
        VehiclePtr vehNow = predictions[aheadID][0];
        VehiclePtr vehPred = predictions[aheadID][1];

        double gapNow = this->DistanceAhead(*vehNow) / currentVelocity;
        double gapPred = this->Predict(dt)->DistanceAhead(*vehPred) / currentVelocity;

        if (gapNow < 1.0 || gapPred < 1.0)
            return VehiclePtr();
    }

    // detect collision with car behind in target lane
    if (TryGetVehicleBehind(predictions, targetLane, behindID))
    {
        VehiclePtr vehNow = predictions[behindID][0];
        VehiclePtr vehPred = predictions[behindID][1];

        double gapNow = this->DistanceBehind(*vehNow) / currentVelocity;
        double gapPred = this->Predict(dt)->DistanceBehind(*vehPred) / currentVelocity;

        if (gapNow < 1.0 || gapPred < 1.0)
            return VehiclePtr();
    }

    // no collisions detected --> safe to move into target lane
    double laneWidth = _roadParams.LaneWidth;

    return VehiclePtr(new Vehicle(
        _roadParams, 
        GetKinematicsForLane(predictions, targetLane, dt), 
        moveLeft ? (_lateralPos - laneWidth) : (_lateralPos + laneWidth));
}

Kinematics EgoVehicle::GetKinematicsForLane(
    const VehiclePredictions &predictions, 
    size_t lane,
    double dt) const
{
    const Kinematics &myKin = _longKinematics;
    double currentPosition = myKin.position;
    double currentVelocity = myKin.velocity;
    double currentAccel = myKin.acceleration;

    size_t aheadID, behindID;
    bool haveAhead = TryGetVehicleAhead(predictions, lane, aheadID);
    bool haveBehind = TryGetVehicleBehind(predictions, lane, behindID);
    double nextVelocity = 0.0;

    if (haveAhead && haveBehind)
    {
        // boxed in --> must travel at speed of traffic
        nextVelocity = predictions[aheadID][0]->GetLongitudinalVelocity();
    }
    else
    {
        // compute constraints on next velocity:
        //   1. don't exceed the legal speed limit
        //   2. don't exceed safe acceleration limits of vehicle
        //   3. maintain safe distance from vehicle ahead (if there is a vehicle ahead)
        double legalConstraint = _roadParams.SpeedLimit - _drivingParams.SpeedLimitBuffer;
        double safetyConstraint = currentVelocity + _drivingParams.AccelerationLimit*dt;
        double trafficConstraint = numeric_limits<double>::max();

        if (haveAhead) 
        {
            // maintain buffer behind vehicle ahead of us
            double aheadPos = predictions[aheadID][1]->GetLongitudinalPosition();
            double bufferDist = myKin.PositionAt(_drivingParams.TrailBufferTime) - currentPosition;
            trafficConstraint = (aheadPos - bufferDist - (currentPosition + 0.5*currentAccel*dt*dt)) / dt;
        }

        // next velocity = max velocity satisfying all constraints
        nextVelocity = *std::min_element({legalConstraint, safetyConstraint, trafficConstraint});
    }
    
    Kinematics accelKin(
        currentPosition, 
        currentVelocity, 
        (nextVelocity - currentVelocity) / dt);

    return accelKin.Predict(dt);
}

bool EgoVehicle::TryGetVehicleAhead(
    const VehiclePredictions &predictions,
    size_t lane, 
    size_t &vehicleID) const
{
    const bool LOOK_AHEAD = true;
    return TryGetNearestVehicle(predictions, LOOK_AHEAD, lane, vehicleID);
}

bool EgoVehicle::TryGetVehicleBehind(
    const VehiclePredictions &predictions,
    size_t lane, 
    size_t &vehicleID) const
{
    const bool LOOK_BEHIND = false;
    return TryGetNearestVehicle(predictions, LOOK_BEHIND, lane, vehicleID);
}

bool EgoVehicle::TryGetNearestVehicle(
    const VehiclePredictions &predictions,
    bool lookAhead,
    size_t lane, 
    size_t &vehicleID) const
{
    double nearestDist = numeric_limits<double>::max();
    bool found = false;

    for (auto it = predictions.begin(); it != predictions.end(); it++)
    {
        VehiclePtr veh = it->second[0];  // vehicle current state

        // First check if the vehicle is in the query lane...
        if (veh->GetLane() == lane)
        {
            // ...then check if it's the nearest vehicle seen so far.
            double dist = lookAhead ? DistanceAhead(*veh) : DistanceBehind(*veh);

            if (dist < nearestDist) 
            {
                nearestDist = dist;
                vehicleID = it->first;
                found = true;
            }
        }
    }

    return found;
}

double EgoVehicle::TrajectoryCost(
    const VehicleTrajectory &traj, 
    const VehiclePredictions &predictions) const
{
    return numeric_limits<double>::max();
}