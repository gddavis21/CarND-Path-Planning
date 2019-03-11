#include "BehaviorPlanner.h"
#include <limits>
#include <algorithm>
#include <iostream>

using namespace std;

const static double timeStep = 1.5;  // sec
//static const size_t ID_EGO = numeric_limits<size_t>::max();

BehaviorPlanner::BehaviorPlanner(
    const DrivingParameters &drivingParams,
    const HighwayParameters &highwayParams)
{
    _drivingParams = drivingParams;
    _highwayParams = highwayParams;
    _currentBehavior = KeepLane;
}

BehaviorPlanner::Behavior BehaviorPlanner::CurrentBehavior() const {
    return _currentBehavior;
}

bool BehaviorPlanner::IsChangingLanes() const 
{
    return _currentBehavior == LaneChangeLeft || _currentBehavior == LaneChangeRight;
}

string BehaviorPlanner::BehaviorLabel(Behavior behavior)
{
    switch (behavior)
    {
        case KeepLane:                  return "Keep Lane";
        case PrepareLaneChangeLeft:     return "Prepare Lane-Change Left";
        case PrepareLaneChangeRight:    return "Prepare Lane-Change Right";
        case LaneChangeLeft:            return "Lane-Change Left";
        case LaneChangeRight:           return "Lane-Change Right";
    }
    return "";
}

// Vehicle EgoVehicle::Update(const VehiclePredictions &predictions, const Vehicle &egoStart)
// {
//     // _longKinematics = longKinematics;
//     // _lateralPos = lateralPos;

//     // vector<Behavior> behaviors;
//     // vector<VehicleTrajectory> trajectories;
//     // vector<double> costs;
//     double minCost = numeric_limits<double>::max();
//     Behavior nextBehavior = KeepLane;
//     Vehicle egoNext;

//     // next trajectory always starts with current state
//     // Vehicle currentState(*this);

//     // compute candidate trajectories & associated costs
//     for (Behavior behavior: SuccessorBehaviors())
//     {
//         Vehicle nextCand = GenerateTrajectory(behavior, predictions, egoNow);

//         if (nextCand.IsValid())
//         {
//             double cost = TrajectoryCost(predictions, egoNow, nextCand);

//             if (cost < minCost)
//             {
//                 nextBehavior = behavior;
//                 minCost = cost;
//                 egoNext = nextCand;
//             }
//         }
//     }

//     _currentBehavior = nextBehavior;
//     return egoNext;
// }

Vehicle BehaviorPlanner::Update(const Vehicle &ego, const vector<Vehicle> &others)
{
    double minCost = numeric_limits<double>::max();
    Behavior bestBehavior = KeepLane;
    Vehicle bestTrajectory;

    for (Behavior behavior: SuccessorBehaviors(ego))
    {
        Vehicle traj = GenerateTrajectory(ego, others, behavior);

        if (traj.IsValid())
        {
            double cost = TrajectoryCost(ego, traj, others, behavior);

            if (cost < minCost)
            {
                bestBehavior = behavior;
                bestTrajectory = traj;
                minCost = cost;
            }
        }
    }

    _currentBehavior = bestBehavior;
    return bestTrajectory;
}

vector<BehaviorPlanner::Behavior> BehaviorPlanner::SuccessorBehaviors(const Vehicle &ego) const
{
    bool inLeftLane = (ego.Lane() == 0);
    bool inRightLane = (ego.Lane() == _highwayParams.LaneCount - 1);

    vector<Behavior> successors;

    switch (_currentBehavior)
    {
        case KeepLane:
            successors.push_back(KeepLane);
            if (!inLeftLane)
                successors.push_back(PrepareLaneChangeLeft);
            if (!inRightLane)
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
            break;

        case LaneChangeRight:
            successors.push_back(KeepLane);
            break;
    }

    return successors;
}

// Vehicle EgoVehicle::GenerateTrajectory(
//     Behavior behavior, 
//     const VehiclePredictions &predictions,
//     const Vehicle &ego) const
// {
//     const bool LEFT = true;
//     const bool RIGHT = false;

//     switch (behavior)
//     {
//         case KeepLane:                return KeepLaneTrajectory(predictions, ego);
//         case PrepareLaneChangeLeft:   return PrepareLaneChangeTrajectory(LEFT, predictions, ego);
//         case PrepareLaneChangeRight:  return PrepareLaneChangeTrajectory(RIGHT, predictions, ego);
//         case LaneChangeLeft:          return LaneChangeTrajectory(LEFT, predictions, ego);
//         case LaneChangeRight:         return LaneChangeTrajectory(RIGHT, predictions, ego);
//     }
    
//     return Vehicle();
// }

Vehicle BehaviorPlanner::GenerateTrajectory(
    const Vehicle &ego,
    const vector<Vehicle> &others,
    Behavior behavior) const
{
    const bool LEFT = true;
    const bool RIGHT = false;

    switch (behavior)
    {
        case KeepLane:                return KeepLaneTrajectory(ego, others);
        case PrepareLaneChangeLeft:   return PrepareLaneChangeTrajectory(ego, others, LEFT);
        case PrepareLaneChangeRight:  return PrepareLaneChangeTrajectory(ego, others, RIGHT);
        case LaneChangeLeft:          return LaneChangeTrajectory(ego, others, LEFT);
        case LaneChangeRight:         return LaneChangeTrajectory(ego, others, RIGHT);
    }
    
    return Vehicle();
}

// Vehicle EgoVehicle::ConstantSpeedTrajectory(
//     const VehiclePredictions &predictions, 
//     double dt) const
// {
//     return Predict(dt);
// }

// Vehicle EgoVehicle::KeepLaneTrajectory(
//     const VehiclePredictions &predictions, 
//     const Vehicle &ego) const
// {
//     Kinematics nextKin = GetKinematicsForLane(predictions, ego, ego.GetLane());
//     return Vehicle(_highwayParams, nextKin, ego.GetLateralPosition(), ego.GetTime() + 1.0);
// }

Vehicle BehaviorPlanner::KeepLaneTrajectory(const Vehicle &ego, const vector<Vehicle> &others) const
{
    return Vehicle(
        _highwayParams, 
        GetLaneKinematics(ego, others, ego.Lane(), timeStep), 
        ego.LateralPosition(), 
        ego.Time() + timeStep);
}

// Vehicle EgoVehicle::PrepareLaneChangeTrajectory(
//     bool moveLeft, 
//     const VehiclePredictions &predictions,
//     const Vehicle &ego) const
// {
//     size_t currentLane = ego.GetLane();
//     size_t targetLane = moveLeft ? (currentLane - 1) : (currentLane + 1);
//     Kinematics currentLaneKinematics = GetKinematicsForLane(predictions, ego, currentLane);
//     Kinematics targetLaneKinematics = GetKinematicsForLane(predictions, ego, targetLane);
    
//     double gapBehind = numeric_limits<double>::max();
//     size_t behindID;

//     if (TryGetVehicleBehind(predictions, ego, currentLane, behindID))
//     {
//         Vehicle vehBehind = predictions.at(behindID)[0];
//         gapBehind = ego.DistanceBehind(vehBehind) / ego.GetLongitudinalVelocity();
//     }

//     Kinematics nextKin = currentLaneKinematics;

//     if (gapBehind > 5 && targetLaneKinematics.velocity < currentLaneKinematics.velocity)
//         nextKin = targetLaneKinematics;

//     return Vehicle(_highwayParams, nextKin, ego.GetLateralPosition(), ego.GetTime() + 1.0);
// }

Vehicle BehaviorPlanner::PrepareLaneChangeTrajectory(
    const Vehicle &ego,
    const vector<Vehicle> &others,
    bool moveLeft) const
{
    size_t currentLane = ego.Lane();
    size_t targetLane = moveLeft ? (currentLane - 1) : (currentLane + 1);

    Kinematics currentLaneKinematics = GetLaneKinematics(ego, others, currentLane, timeStep);
    Kinematics targetLaneKinematics = GetLaneKinematics(ego, others, targetLane, timeStep);
    bool slowToTarget = false;

    if (targetLaneKinematics.velocity < currentLaneKinematics.velocity)
    {
        size_t indexBehind;
        double timeBehind;
        slowToTarget = TryGetVehicleBehind(ego, others, currentLane, indexBehind, timeBehind) && (timeBehind > 3);
    }

    return Vehicle(
        _highwayParams,
        slowToTarget ? targetLaneKinematics : currentLaneKinematics,
        ego.LateralPosition(),
        ego.Time() + timeStep);
}

// Vehicle EgoVehicle::LaneChangeTrajectory(
//     bool moveLeft, 
//     const VehiclePredictions &predictions,
//     const Vehicle &ego) const
// {
//     double currentVelocity = ego.GetLongitudinalVelocity();
//     size_t currentLane = ego.GetLane();
//     size_t targetLane = moveLeft ? (currentLane - 1) : (currentLane + 1);

//     size_t aheadID, behindID;

//     // detect collision with car ahead in target lane
//     if (TryGetVehicleAhead(predictions, ego, targetLane, aheadID))
//     {
//         Vehicle vehNow = predictions.at(aheadID)[0];
//         Vehicle vehPred = predictions.at(aheadID)[1];

//         double gapNow = ego.DistanceAhead(vehNow) / currentVelocity;
//         double gapPred = ego.Predict(1.0).DistanceAhead(vehPred) / currentVelocity;

//         if (gapNow < 1.0 || gapPred < 1.0)
//             return Vehicle();
//     }

//     // detect collision with car behind in target lane
//     if (TryGetVehicleBehind(predictions, ego, targetLane, behindID))
//     {
//         Vehicle vehNow = predictions.at(behindID)[0];
//         Vehicle vehPred = predictions.at(behindID)[1];

//         double gapNow = ego.DistanceBehind(vehNow) / currentVelocity;
//         double gapPred = ego.Predict(1.0).DistanceBehind(vehPred) / currentVelocity;

//         if (gapNow < 1.0 || gapPred < 1.0)
//             return Vehicle();
//     }

//     // no collisions detected --> safe to move into target lane
//     Kinematics nextKin = GetKinematicsForLane(predictions, ego, targetLane);
//     double currentLateral = ego.GetLateralPosition();
//     double laneWidth = _highwayParams.LaneWidth;
//     double nextLateral = moveLeft ? (currentLateral - laneWidth) : (currentLateral + laneWidth);

//     return Vehicle(
//         _highwayParams, 
//         GetKinematicsForLane(predictions, targetLane, dt), 
//         moveLeft ? (_lateralPos - laneWidth) : (_lateralPos + laneWidth));
// }

Vehicle BehaviorPlanner::LaneChangeTrajectory(
    const Vehicle &ego,
    const vector<Vehicle> &others,
    bool moveLeft) const
{
    size_t targetLane = moveLeft ? (ego.Lane() - 1) : (ego.Lane() + 1);
    Kinematics targetLaneKinematics = GetLaneKinematics(ego, others, targetLane, timeStep);

    size_t indexAhead, indexBehind;
    double timeAhead, timeBehind;

    // TODO: parameterize "potential collision" time margin
    // TODO: distinguish between acceptable time ahead & time behind?

    // detect collision with car ahead in target lane
    if (TryGetVehicleAhead(ego, others, targetLane, indexAhead, timeAhead) && (timeAhead < 1.5))
    {
        return Vehicle();
    }
    // {
    //     Vehicle vehAheadPred = others[indexAhead].Predict(timeStep);
    //     double dist = fabs(targetLaneKinematics.position - vehAheadPred.GetLongitudinalPosition());
    //     double speed = max(targetLaneKinematics.velocity, vehAheadPred.GetLongitudinalVelocity());
    //     double gapTime = dist / speed;

    //     // TODO: parameterize acceptable lane-change gap
    //     // TODO: distinguish between acceptable gap ahead and gap behind?
    //     if (gapTime < 1.5)
    //         return Vehicle();
    // }

    // detect collision with car behind in target lane
    if (TryGetVehicleBehind(ego, others, targetLane, indexBehind, timeBehind) && (timeBehind < 1.5))
    {
        return Vehicle();
    }
    // {
    //     Vehicle vehBehindPred = others[indexBehind].Predict(timeStep);
    //     double dist = fabs(targetLaneKinematics.position - vehBehindPred.GetLongitudinalPosition());
    //     double speed = max(targetLaneKinematics.velocity, vehBehindPred.GetLongitudinalVelocity());
    //     double gapTime = dist / speed;

    //     // TODO: parameterize acceptable lane-change gap
    //     // TODO: distinguish between acceptable gap ahead and gap behind?
    //     if (gapTime < 1.5)
    //         return Vehicle();
    // }

    // no collisions detected --> safe to move into target lane
    double laneWidth = _highwayParams.LaneWidth;

    return Vehicle(
        _highwayParams, 
        targetLaneKinematics, 
        moveLeft ? (ego.LateralPosition() - laneWidth) : (ego.LateralPosition() + laneWidth),
        ego.Time() + timeStep);
}

// Kinematics EgoVehicle::GetKinematicsForLane(
//     const VehiclePredictions &predictions, 
//     const Vehicle &ego,
//     size_t lane) const
// {
//     const Kinematics &selfKin = ego.GetLongitudinalKinematics();
//     double currentPosition = selfKin.position;
//     double currentVelocity = selfKin.velocity;
//     double currentAccel = selfKin.acceleration;

//     size_t aheadID, behindID;
//     bool haveAhead = TryGetVehicleAhead(predictions, ego, lane, aheadID);
//     bool haveBehind = TryGetVehicleBehind(predictions, ego, lane, behindID);
//     double nextVelocity = 0.0;

//     if (haveAhead && haveBehind)
//     {
//         // boxed in --> must travel at speed of traffic
//         nextVelocity = predictions.at(aheadID)[0].GetLongitudinalVelocity();
//     }
//     else
//     {
//         // compute constraints on next velocity:
//         //   1. don't exceed the legal speed limit
//         //   2. don't exceed safe acceleration limits of vehicle
//         //   3. maintain safe distance from vehicle ahead (if there is a vehicle ahead)
//         double targetVelocity = _highwayParams.SpeedLimit - _drivingParams.SpeedLimitBuffer;
//         double safetyConstraint = currentVelocity + _drivingParams.AccelerationLimit;
//         double trafficConstraint = numeric_limits<double>::max();

//         if (haveAhead) 
//         {
//             // maintain buffer behind vehicle ahead of us
//             double aheadPos = predictions.at(aheadID)[1].GetLongitudinalPosition();
//             double bufferDist = selfKin.PositionAt(_drivingParams.TrailBufferTime) - currentPosition;
//             trafficConstraint = (aheadPos - bufferDist - (currentPosition + 0.5*currentAccel));
//         }

//         // next velocity = max velocity satisfying all constraints
//         nextVelocity = min(min(safetyConstraint, trafficConstraint), targetVelocity);
//     }
    
//     Kinematics accelKin(currentPosition, currentVelocity, nextVelocity - currentVelocity);
//     return accelKin.Predict(1.0);
// }

Kinematics BehaviorPlanner::GetLaneKinematics(
    const Vehicle &ego,
    const vector<Vehicle> &others,
    size_t lane,
    double dt) const
{
    double egoPos = ego.Position();
    double egoVel = ego.Velocity();
    double egoAcc = ego.Acceleration();

    size_t indexAhead, indexBehind;
    double timeAhead, timeBehind;
    bool haveAhead = TryGetVehicleAhead(ego, others, lane, indexAhead, timeAhead);
    bool haveBehind = TryGetVehicleBehind(ego, others, lane, indexBehind, timeBehind);

    double newAccel = 0;

    if (haveAhead && haveBehind && (timeAhead < 2) && (timeBehind < 2))
    {
        // boxed in, accelerate to match speed of vehicle ahead
        newAccel = (others[indexAhead].Velocity() - egoVel) / dt;
        cout << "boxed in" << endl;
    }
    else
    {
        // by default accelerate to reach target velocity
        double targetVelocity = _highwayParams.SpeedLimit - _drivingParams.SpeedLimitBuffer;
        double accelToTargetVelocity = (targetVelocity - egoVel) / dt;
        double accelToSafeDistance = numeric_limits<double>::max();

        // maintain safe distance behind vehicle ahead of us
        if (haveAhead && (timeAhead < 3)) 
        {
            const Vehicle &vehAhead = others[indexAhead];
            double vehPos = vehAhead.Position();
            double vehVel = vehAhead.Velocity();
            double vehAcc = vehAhead.Acceleration();
            double B = _drivingParams.FollowBuffer;
            accelToSafeDistance = ((vehPos + vehVel*dt + 0.5*vehAcc*dt*dt) - (egoPos + egoVel*dt) - B) * 2 / (dt*dt);
            cout << "maintain safe distance" << endl;
        }

        newAccel = min(accelToTargetVelocity, accelToSafeDistance);
    }

    // enforce acceleration & jerk limits
    double Amax = _drivingParams.AccelerationLimit;
    double Jmax = _drivingParams.JerkLimit;
    newAccel = clamp(newAccel, -Amax, Amax);
    //newAccel = clamp(newAccel, egoAcc - Jmax*dt, egoAcc + Jmax*dt);

    Kinematics newKin(egoPos, egoVel, newAccel);
    return newKin.Predict(dt);
}

bool BehaviorPlanner::TryGetVehicleAhead(
    const Vehicle &ego,
    const vector<Vehicle> &others,
    size_t lane, 
    size_t &indexAhead,
    double &timeAhead) const
{
    const bool LOOK_AHEAD = true;

    if (!TryGetNearestVehicle(ego, others, lane, LOOK_AHEAD, indexAhead))
        return false;

    const Vehicle &vehAhead = others[indexAhead];
    timeAhead = ego.DistanceBehind(vehAhead) / ego.Velocity();
    return true;
}

bool BehaviorPlanner::TryGetVehicleBehind(
    const Vehicle &ego,
    const vector<Vehicle> &others,
    size_t lane, 
    size_t &indexBehind,
    double &timeBehind) const
{
    const bool LOOK_BEHIND = false;

    if (!TryGetNearestVehicle(ego, others, lane, LOOK_BEHIND, indexBehind))
        return false;

    const Vehicle &vehBehind = others[indexBehind];
    timeBehind = vehBehind.DistanceBehind(ego) / vehBehind.Velocity();
}

bool BehaviorPlanner::TryGetNearestVehicle(
    const Vehicle &ego,
    const vector<Vehicle> &others,
    size_t lane, 
    bool lookAhead,
    size_t &vehicleIndex) const
{
    double nearestDist = numeric_limits<double>::max();
    bool found = false;

    for (size_t i=0; i < others.size(); i++)
    {
        const Vehicle &other = others[i];

        if (other.Lane() == lane)
        {
            double dist = lookAhead ? ego.DistanceBehind(other) : other.DistanceBehind(ego);

            if (dist < nearestDist)
            {
                found = true;
                vehicleIndex = i;
                nearestDist = dist;
            }
        }
    }

    return found;
}

// double EgoVehicle::TrajectoryCost(
//     const VehicleTrajectory &traj, 
//     const VehiclePredictions &predictions) const
// {
//     return EfficiencyCost(traj[1].GetLongitudinalVelocity());
// }

// double EgoVehicle::EfficiencyCost(double velocity) const
// {
//     const double SPEED_LIMIT = _highwayParams.SpeedLimit;
//     const double BUFFER_VELOCITY = _drivingParams.SpeedLimitBuffer;
//     const double TARGET_VELOCITY = SPEED_LIMIT - BUFFER_VELOCITY;
//     const double STOP_COST = _drivingParams.StopCost;

//     if (velocity < TARGET_VELOCITY)
//     {
//         return STOP_COST * ((TARGET_VELOCITY - velocity) / TARGET_VELOCITY);
//     }
//     else if (velocity < SPEED_LIMIT)
//     {
//         return (velocity - TARGET_VELOCITY) / BUFFER_VELOCITY;
//     }

//     return 1.0;
// }

// double EgoVehicle::TrajectoryCost(double intendedSpeed, double finalSpeed) const
// {
//     double costIntended = SpeedCost(intendedSpeed);
//     double costFinal = SpeedCost(finalSpeed);

//     if (costIntended == 1.0 || costFinal == 1.0)
//         return 1.0;

//     return (costIntended + costFinal) / 2.0;
// }

double BehaviorPlanner::TrajectoryCost(
    const Vehicle &ego, 
    const Vehicle &next, 
    const vector<Vehicle> &others,
    Behavior behavior) const
{
    size_t intendedLane = ego.Lane();
    size_t finalLane = next.Lane();
    double dt = next.Time() - ego.Time();

    if (behavior == PrepareLaneChangeLeft || behavior == LaneChangeLeft) {
        intendedLane--;
    }
    else if (behavior == PrepareLaneChangeRight || behavior == LaneChangeRight) {
        intendedLane++;
    }

    double intendedSpeed = GetLaneKinematics(ego, others, intendedLane, dt).velocity;
    double finalSpeed = GetLaneKinematics(ego, others, finalLane, dt).velocity;

    double costIntended = SpeedCost(intendedSpeed);
    double costFinal = SpeedCost(finalSpeed);

    // if (costIntended == 1.0 || costFinal == 1.0)
    //     return 1.0;

    //return (costIntended + costFinal) / 2.0;
    return (costIntended + costFinal);
}

double BehaviorPlanner::SpeedCost(double speed) const 
{
    const double SPEED_LIMIT = _highwayParams.SpeedLimit;
    const double SPEED_BUFFER = _drivingParams.SpeedLimitBuffer;
    const double TARGET_SPEED = SPEED_LIMIT - SPEED_BUFFER;
    const double STOP_COST = _drivingParams.StopCost;

    if (speed < TARGET_SPEED)
    {
        return STOP_COST * ((TARGET_SPEED - speed) / TARGET_SPEED);
    }
    else if (speed < SPEED_LIMIT)
    {
        return (speed - TARGET_SPEED) / SPEED_BUFFER;
    }

    return 1.0;
}
