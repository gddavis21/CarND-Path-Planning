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

size_t BehaviorPlanner::GetLane(const Vehicle &veh) const {
    return _highwayParams.WhichLane(veh.LateralPosition());
}

double BehaviorPlanner::LaneCenter(size_t lane) const {
    return lane*_highwayParams.LaneWidth + _highwayParams.LaneWidth/2;
}

double BehaviorPlanner::DistanceBehind(const Vehicle &behind, const Vehicle &ahead) const
{
    double dist = ahead.Position() - behind.Position();
    return (dist < 0.0) ? (dist + _highwayParams.WrapAroundPosition) : dist;
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

BehaviorPlanner::Trajectory BehaviorPlanner::Update(const Vehicle &ego, const vector<Vehicle> &others)
{
    double min_cost = numeric_limits<double>::max();
    Behavior best_behavior = KeepLane;
    Trajectory best_trajectory;

    for (Behavior behavior: SuccessorBehaviors(ego))
    {
        Trajectory traj = GenerateTrajectory(ego, others, behavior);

        if (traj.is_valid)
        {
            double cost = TrajectoryCost(ego, others, behavior, traj);

            if (cost < min_cost)
            {
                best_behavior = behavior;
                best_trajectory = traj;
                min_cost = cost;
            }
        }
    }

    _currentBehavior = best_behavior;
    return best_trajectory;
}

vector<BehaviorPlanner::Behavior> BehaviorPlanner::SuccessorBehaviors(const Vehicle &ego) const
{
    bool inLeftLane = (GetLane(ego) == 0);
    bool inRightLane = (GetLane(ego) == _highwayParams.LaneCount - 1);

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

BehaviorPlanner::Trajectory BehaviorPlanner::GenerateTrajectory(
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
    
    return Trajectory { .is_valid = false };
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

// Vehicle BehaviorPlanner::KeepLaneTrajectory(const Vehicle &ego, const vector<Vehicle> &others) const
// {
//     return Vehicle(
//         _highwayParams, 
//         GetLaneKinematics(ego, others, ego.Lane(), timeStep), 
//         ego.LaneCenter(), 
//         ego.Time() + timeStep);
// }

BehaviorPlanner::Trajectory BehaviorPlanner::KeepLaneTrajectory(
    const Vehicle &ego, 
    const vector<Vehicle> &others) const
{
    return {
        .is_valid = true,
        .goal_velocity = GetLaneVelocity(ego, others, GetLane(ego)),
        .goal_lateral_position = LaneCenter(GetLane(ego))
    };
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

// Vehicle BehaviorPlanner::PrepareLaneChangeTrajectory(
//     const Vehicle &ego,
//     const vector<Vehicle> &others,
//     bool moveLeft) const
// {
//     size_t currentLane = ego.Lane();
//     size_t targetLane = moveLeft ? (currentLane - 1) : (currentLane + 1);

//     Kinematics currentLaneKinematics = GetLaneKinematics(ego, others, currentLane, timeStep);
//     Kinematics targetLaneKinematics = GetLaneKinematics(ego, others, targetLane, timeStep);
//     bool slowToTarget = false;

//     if (targetLaneKinematics.velocity < currentLaneKinematics.velocity)
//     {
//         size_t indexBehind;
//         double timeBehind;
//         slowToTarget = TryGetVehicleBehind(ego, others, currentLane, indexBehind, timeBehind) && (timeBehind > 3);
//     }

//     return Vehicle(
//         _highwayParams,
//         slowToTarget ? targetLaneKinematics : currentLaneKinematics,
//         ego.LaneCenter(),
//         ego.Time() + timeStep);
// }

BehaviorPlanner::Trajectory BehaviorPlanner::PrepareLaneChangeTrajectory(
    const Vehicle &ego,
    const vector<Vehicle> &others,
    bool change_left) const
{
    size_t current_lane = GetLane(ego);
    size_t target_lane = change_left ? (current_lane - 1) : (current_lane + 1);

    double current_lane_velocity = GetLaneVelocity(ego, others, current_lane);
    double target_lane_velocity = GetLaneVelocity(ego, others, target_lane);
    bool slow_to_target = false;

    if (target_lane_velocity < current_lane_velocity)
    {
        size_t index;
        double dist_behind, time_behind;
        slow_to_target = TryGetVehicleBehind(ego, others, current_lane, index, dist_behind, time_behind) && (time_behind > 3);
    }

    return {
        .is_valid = true,
        .goal_velocity = slow_to_target ? target_lane_velocity : current_lane_velocity,
        .goal_lateral_position = LaneCenter(GetLane(ego))
    };
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

// Vehicle BehaviorPlanner::LaneChangeTrajectory(
//     const Vehicle &ego,
//     const vector<Vehicle> &others,
//     bool moveLeft) const
// {
//     size_t targetLane = moveLeft ? (ego.Lane() - 1) : (ego.Lane() + 1);
//     Kinematics targetLaneKinematics = GetLaneKinematics(ego, others, targetLane, timeStep);

//     size_t indexAhead, indexBehind;
//     double timeAhead, timeBehind;

//     // TODO: parameterize "potential collision" time margin
//     // TODO: distinguish between acceptable time ahead & time behind?

//     // detect collision with car ahead in target lane
//     if (TryGetVehicleAhead(ego, others, targetLane, indexAhead, timeAhead) && (timeAhead < 1.5))
//     {
//         return Vehicle();
//     }

//     // detect collision with car behind in target lane
//     if (TryGetVehicleBehind(ego, others, targetLane, indexBehind, timeBehind) && (timeBehind < 1.5))
//     {
//         return Vehicle();
//     }

//     // no collisions detected --> safe to move into target lane
//     double laneWidth = _highwayParams.LaneWidth;

//     return Vehicle(
//         _highwayParams, 
//         targetLaneKinematics, 
//         moveLeft ? (ego.LaneCenter() - laneWidth) : (ego.LaneCenter() + laneWidth),
//         ego.Time() + timeStep);
// }

BehaviorPlanner::Trajectory BehaviorPlanner::LaneChangeTrajectory(
    const Vehicle &ego,
    const vector<Vehicle> &others,
    bool change_left) const
{
    size_t current_lane = GetLane(ego);
    size_t target_lane = change_left ? (current_lane - 1) : (current_lane + 1);

    size_t index;
    double dist_ahead, time_ahead, dist_behind, time_behind;

    // TODO: parameterize "potential collision" time margin
    // TODO: distinguish between acceptable time ahead & time behind?

    // detect collision with car ahead in target lane
    if (TryGetVehicleAhead(ego, others, target_lane, index, dist_ahead, time_ahead) && (time_ahead < 1.5))
    {
        return Trajectory { .is_valid = false };
    }

    // detect collision with car behind in target lane
    if (TryGetVehicleBehind(ego, others, target_lane, index, dist_behind, time_behind) && (time_behind < 1.5))
    {
        return Trajectory { .is_valid = false };
    }

    // no collisions detected --> safe to move into target lane
    return {
        .is_valid = true,
        .goal_velocity = GetLaneVelocity(ego, others, target_lane),
        .goal_lateral_position = LaneCenter(target_lane)
    };
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

// Kinematics BehaviorPlanner::GetLaneKinematics(
//     const Vehicle &ego,
//     const vector<Vehicle> &others,
//     size_t lane,
//     double dt) const
// {
//     double egoPos = ego.Position();
//     double egoVel = ego.Velocity();
//     double egoAcc = ego.Acceleration();

//     size_t indexAhead, indexBehind;
//     double timeAhead, timeBehind;
//     bool haveAhead = TryGetVehicleAhead(ego, others, lane, indexAhead, timeAhead);
//     bool haveBehind = TryGetVehicleBehind(ego, others, lane, indexBehind, timeBehind);

//     double newAccel = 0;

//     if (haveAhead && haveBehind && (timeAhead < 2) && (timeBehind < 2))
//     {
//         // boxed in, accelerate to match speed of vehicle ahead
//         newAccel = (others[indexAhead].Velocity() - egoVel) / dt;
//         cout << "boxed in" << endl;
//     }
//     else
//     {
//         // by default accelerate to reach target velocity
//         double targetVelocity = _highwayParams.SpeedLimit - _drivingParams.SpeedLimitBuffer;
//         double accelToTargetVelocity = (targetVelocity - egoVel) / dt;
//         double accelToSafeDistance = numeric_limits<double>::max();

//         // maintain safe distance behind vehicle ahead of us
//         if (haveAhead && (timeAhead < 3)) 
//         {
//             const Vehicle &vehAhead = others[indexAhead];
//             double vehPos = vehAhead.Position();
//             double vehVel = vehAhead.Velocity();
//             double vehAcc = vehAhead.Acceleration();
//             double B = _drivingParams.FollowBuffer;
//             accelToSafeDistance = ((vehPos + vehVel*dt + 0.5*vehAcc*dt*dt) - (egoPos + egoVel*dt) - B) * 2 / (dt*dt);
//             cout << "maintain safe distance" << endl;
//         }

//         newAccel = min(accelToTargetVelocity, accelToSafeDistance);
//     }

//     // enforce acceleration & jerk limits
//     double Amax = _drivingParams.AccelerationLimit;
//     double Jmax = _drivingParams.JerkLimit;
//     newAccel = clamp(newAccel, -Amax, Amax);
//     //newAccel = clamp(newAccel, egoAcc - Jmax*dt, egoAcc + Jmax*dt);

//     Kinematics newKin(egoPos, egoVel, newAccel);
//     return newKin.Predict(dt);
// }

double BehaviorPlanner::GetLaneVelocity(
    const Vehicle &ego,
    const vector<Vehicle> &others,
    size_t lane) const
{
    size_t indexAhead, indexBehind;
    double distAhead, timeAhead, distBehind, timeBehind;
    bool haveAhead = TryGetVehicleAhead(ego, others, lane, indexAhead, distAhead, timeAhead);
    bool haveBehind = TryGetVehicleBehind(ego, others, lane, indexBehind, distBehind, timeBehind);
    double lane_velocity = _highwayParams.SpeedLimit - _drivingParams.SpeedLimitBuffer;

    if (haveAhead && haveBehind && (timeAhead < 2) && (timeBehind < 2))
    {
        // boxed in, accelerate to match speed of vehicle ahead
        lane_velocity = others[indexAhead].Velocity();
        cout << "boxed in" << endl;
    }
    else if (haveAhead && (timeAhead < 3))
    {
        lane_velocity = min(lane_velocity, others[indexAhead].Velocity());
        cout << "maintain safe distance" << endl;
    }

    return lane_velocity;
}

bool BehaviorPlanner::TryGetVehicleAhead(
    const Vehicle &ego,
    const vector<Vehicle> &others,
    size_t lane, 
    size_t &index,
    double &dist_ahead,
    double &time_ahead) const
{
    const bool LOOK_AHEAD = true;

    if (!TryGetNearestVehicle(ego, others, lane, LOOK_AHEAD, index))
        return false;

    dist_ahead = DistanceBehind(ego, others[index]);
    time_ahead = dist_ahead / ego.Velocity();
    return true;
}

bool BehaviorPlanner::TryGetVehicleBehind(
    const Vehicle &ego,
    const vector<Vehicle> &others,
    size_t lane, 
    size_t &index,
    double &dist_behind,
    double &time_behind) const
{
    const bool LOOK_BEHIND = false;

    if (!TryGetNearestVehicle(ego, others, lane, LOOK_BEHIND, index))
        return false;

    dist_behind = DistanceBehind(others[index], ego);
    time_behind = dist_behind / others[index].Velocity();
    return true;
}

bool BehaviorPlanner::TryGetNearestVehicle(
    const Vehicle &ego,
    const vector<Vehicle> &others,
    size_t lane, 
    bool look_ahead,
    size_t &index) const
{
    double nearest_dist = numeric_limits<double>::max();
    bool found = false;

    for (size_t i=0; i < others.size(); i++)
    {
        const Vehicle &other = others[i];

        if (GetLane(other) == lane)
        {
            double dist = look_ahead ? DistanceBehind(ego, other) : DistanceBehind(other, ego);

            if (dist < nearest_dist)
            {
                found = true;
                index = i;
                nearest_dist = dist;
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

// double BehaviorPlanner::TrajectoryCost(
//     const Vehicle &ego, 
//     const Vehicle &next, 
//     const vector<Vehicle> &others,
//     Behavior behavior) const
// {
//     size_t intendedLane = ego.Lane();
//     size_t finalLane = next.Lane();
//     double dt = next.Time() - ego.Time();

//     if (behavior == PrepareLaneChangeLeft || behavior == LaneChangeLeft) {
//         intendedLane--;
//     }
//     else if (behavior == PrepareLaneChangeRight || behavior == LaneChangeRight) {
//         intendedLane++;
//     }

//     double intendedSpeed = GetLaneKinematics(ego, others, intendedLane, dt).velocity;
//     double finalSpeed = GetLaneKinematics(ego, others, finalLane, dt).velocity;

//     double costIntended = SpeedCost(intendedSpeed);
//     double costFinal = SpeedCost(finalSpeed);

//     cout << BehaviorLabel(behavior) << endl;
//     cout << " intended speed: " << intendedSpeed << ", cost: " << costIntended << endl;
//     cout << " final speed: " << finalSpeed << ", cost: " << costFinal << endl;

//     // if (costIntended == 1.0 || costFinal == 1.0)
//     //     return 1.0;

//     //return (costIntended + costFinal) / 2.0;
//     return (costIntended + costFinal);
// }

size_t BehaviorPlanner::IntendedLane(const Vehicle &ego, Behavior behavior) const
{
    bool change_left = (behavior == PrepareLaneChangeLeft || behavior == LaneChangeLeft);
    bool change_right = (behavior == PrepareLaneChangeRight || behavior == LaneChangeRight);
    int lane_change_offs = change_left ? -1 : (change_right ? 1 : 0);
    return _highwayParams.WhichLane(ego.LateralPosition()) + lane_change_offs;
}

size_t BehaviorPlanner::FinalLane(Trajectory traj) const
{
    return _highwayParams.WhichLane(traj.goal_lateral_position);
}

double BehaviorPlanner::TrajectoryCost(
    const Vehicle &ego, 
    const vector<Vehicle> &others,
    Behavior behavior,
    Trajectory traj) const
{
    const double WEIGHT_TRAFFIC_AHEAD = 10.0;
    const double WEIGHT_TRAFFIC_BEHIND = 10.0;
    const double WEIGHT_INTENDED_SPEED = 1.0;
    const double WEIGHT_FINAL_SPEED = 0.5;

    double total_cost = 0.0;

    double traffic_ahead_cost = TrafficAheadCost(ego, others, behavior, traj);
    total_cost += WEIGHT_TRAFFIC_AHEAD * traffic_ahead_cost;

    double traffic_behind_cost = TrafficBehindCost(ego, others, behavior, traj);
    total_cost += WEIGHT_TRAFFIC_BEHIND * traffic_behind_cost;

    double intended_speed_cost = IntendedLaneSpeedCost(ego, others, behavior, traj);
    total_cost += WEIGHT_INTENDED_SPEED * intended_speed_cost;

    double final_speed_cost = FinalLaneSpeedCost(ego, others, behavior, traj);
    total_cost += WEIGHT_FINAL_SPEED * final_speed_cost;

    cout << BehaviorLabel(behavior) << endl;
    cout << " traffic ahead cost: " << traffic_ahead_cost << endl;
    cout << " traffic behind cost: " << traffic_behind_cost << endl;
    cout << " intended lane cost: " << intended_speed_cost << endl;
    cout << " final lane cost: " << final_speed_cost << endl;

    return total_cost;
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

double BehaviorPlanner::IntendedLaneSpeedCost(
    const Vehicle &ego, 
    const vector<Vehicle> &others,
    Behavior behavior,
    Trajectory traj) const
{
    size_t intended_lane = IntendedLane(ego, behavior);
    double intended_speed = GetLaneVelocity(ego, others, intended_lane);
    return SpeedCost(intended_speed);
}

double BehaviorPlanner::FinalLaneSpeedCost(
    const Vehicle &ego, 
    const vector<Vehicle> &others,
    Behavior behavior,
    Trajectory traj) const
{
    size_t final_lane = FinalLane(traj);
    double final_speed = GetLaneVelocity(ego, others, final_lane);
    return SpeedCost(final_speed);
}

double BehaviorPlanner::TrafficAheadCost(
    const Vehicle &ego, 
    const vector<Vehicle> &others,
    Behavior behavior,
    Trajectory traj) const
{
    size_t final_lane = FinalLane(traj);
    size_t index;
    double dist_ahead, time_ahead;

    if (TryGetVehicleAhead(ego, others, final_lane, index, dist_ahead, time_ahead))
    {
        time_ahead = dist_ahead / traj.goal_velocity;

        if (time_ahead < 1)
            return 1.0;

        if (time_ahead < 3)
            return 1.5 - 0.5*time_ahead;
    }

    return 0.0;
}

double BehaviorPlanner::TrafficBehindCost(
    const Vehicle &ego, 
    const vector<Vehicle> &others,
    Behavior behavior,
    Trajectory traj) const
{
    size_t final_lane = FinalLane(traj);
    size_t index;
    double dist_behind, time_behind;

    if (TryGetVehicleBehind(ego, others, final_lane, index, dist_behind, time_behind))
    {
        if (time_behind < 1)
            return 1.0;

        if (time_behind < 3)
            return 1.5 - 0.5*time_behind;
    }

    return 0.0;
}
