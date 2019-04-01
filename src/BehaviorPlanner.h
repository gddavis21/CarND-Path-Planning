#ifndef __BEHAVIOR_PLANNER_H__
#define __BEHAVIOR_PLANNER_H__

#include <vector>
#include "helpers.h"
#include "Highway.h"
#include "Vehicle.h"

struct DrivingParameters
{
    double AccelerationLimit;
    double JerkLimit;
    double FollowBuffer;
    double SpeedLimitBuffer;
    double StopCost;
};

// class BehaviorPlanner
// {
// public:
//     BehaviorPlanner(const DrivingParameters&, const HighwayParameters&);

//     Vehicle Update(const Vehicle &ego, const std::vector<Vehicle> &others);

//     enum Behavior 
//     {
//         KeepLane,
//         PrepareLaneChangeLeft,
//         PrepareLaneChangeRight,
//         LaneChangeLeft,
//         LaneChangeRight
//     };

//     bool IsChangingLanes() const;
//     Behavior CurrentBehavior() const;

//     static std::string BehaviorLabel(Behavior);

// private:
//     std::vector<Behavior> SuccessorBehaviors(const Vehicle &ego) const;

//     Vehicle GenerateTrajectory(
//         const Vehicle &ego, 
//         const std::vector<Vehicle> &others, 
//         Behavior) const;

//     Vehicle KeepLaneTrajectory(
//         const Vehicle &ego, 
//         const std::vector<Vehicle> &others) const;
    
//     Vehicle PrepareLaneChangeTrajectory(
//         const Vehicle &ego, 
//         const std::vector<Vehicle> &others, 
//         bool moveLeft) const;

//     Vehicle LaneChangeTrajectory(
//         const Vehicle &ego, 
//         const std::vector<Vehicle> &others, 
//         bool moveLeft) const;

//     Kinematics GetLaneKinematics(
//         const Vehicle &ego, 
//         const std::vector<Vehicle> &others, 
//         size_t lane, 
//         double dt) const;

//     bool TryGetVehicleAhead(
//         const Vehicle &ego, 
//         const std::vector<Vehicle> &others, 
//         size_t lane, 
//         size_t &indexAhead,
//         double &timeAhead) const;

//     bool TryGetVehicleBehind(
//         const Vehicle &ego, 
//         const std::vector<Vehicle> &others, 
//         size_t lane, 
//         size_t &indexBehind,
//         double &timeBehind) const;

//     bool TryGetNearestVehicle(
//         const Vehicle &ego, 
//         const std::vector<Vehicle> &others, 
//         size_t lane, 
//         bool lookAhead, 
//         size_t &vehicleIndex) const;

//     double TrajectoryCost(
//         const Vehicle &ego,
//         const Vehicle &next,
//         const std::vector<Vehicle> &others,
//         Behavior) const;

//     double SpeedCost(double speed) const;

//     DrivingParameters _drivingParams;
//     HighwayParameters _highwayParams;
//     Behavior _currentBehavior;
// };

class BehaviorPlanner
{
public:
    BehaviorPlanner(const DrivingParameters&, const HighwayParameters&);

    enum Behavior 
    {
        KeepLane,
        PrepareLaneChangeLeft,
        PrepareLaneChangeRight,
        LaneChangeLeft,
        LaneChangeRight
    };

    struct Trajectory
    {
        bool is_valid;
        double goal_velocity;
        double goal_lateral_position;
    };

    Trajectory Update(const Vehicle &ego, const std::vector<Vehicle> &others);

    Behavior CurrentBehavior() const;
    bool IsChangingLanes() const;

    static std::string BehaviorLabel(Behavior);

private:
    std::vector<Behavior> SuccessorBehaviors(const Vehicle &ego) const;

    Trajectory GenerateTrajectory(
        const Vehicle &ego, 
        const std::vector<Vehicle> &others, 
        Behavior) const;

    Trajectory KeepLaneTrajectory(
        const Vehicle &ego, 
        const std::vector<Vehicle> &others) const;
    
    Trajectory PrepareLaneChangeTrajectory(
        const Vehicle &ego, 
        const std::vector<Vehicle> &others, 
        bool moveLeft) const;

    Trajectory LaneChangeTrajectory(
        const Vehicle &ego, 
        const std::vector<Vehicle> &others, 
        bool moveLeft) const;

    Kinematics GetLaneKinematics(
        const Vehicle &ego, 
        const std::vector<Vehicle> &others, 
        size_t lane, 
        double dt) const;

    double GetLaneVelocity(
        const Vehicle &ego, 
        const std::vector<Vehicle> &others, 
        size_t lane) const;

    bool TryGetVehicleAhead(
        const Vehicle &ego, 
        const std::vector<Vehicle> &others, 
        size_t lane, 
        size_t &indexAhead,
        double &timeAhead) const;

    bool TryGetVehicleBehind(
        const Vehicle &ego, 
        const std::vector<Vehicle> &others, 
        size_t lane, 
        size_t &indexBehind,
        double &timeBehind) const;

    bool TryGetNearestVehicle(
        const Vehicle &ego, 
        const std::vector<Vehicle> &others, 
        size_t lane, 
        bool lookAhead, 
        size_t &vehicleIndex) const;

    double TrajectoryCost(
        const Vehicle &ego,
        const std::vector<Vehicle> &others,
        Behavior,
        Trajectory) const;

    double SpeedCost(double speed) const;
    size_t GetLane(const Vehicle&) const;
    double LaneCenter(size_t lane) const;
    
    // how far behind other vehicle am I? (normalized for track wrap-around)
    double DistanceBehind(const Vehicle &behind, const Vehicle &ahead) const;

    DrivingParameters _drivingParams;
    HighwayParameters _highwayParams;
    Behavior _currentBehavior;
};

#endif // __BEHAVIOR_PLANNER_H__
 