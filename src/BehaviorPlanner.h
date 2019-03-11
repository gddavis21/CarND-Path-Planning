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

class BehaviorPlanner
{
public:
    BehaviorPlanner(const DrivingParameters&, const HighwayParameters&);

    // void Update(
    //     const VehiclePredictions&, 
    //     const Vehicle &egoStart,
    //     Vehicle &egoFinish,
    //     double &trajTime);
    Vehicle Update(const Vehicle &ego, const std::vector<Vehicle> &others);

    enum Behavior 
    {
        KeepLane,
        PrepareLaneChangeLeft,
        PrepareLaneChangeRight,
        LaneChangeLeft,
        LaneChangeRight
    };

    bool IsChangingLanes() const;
    Behavior CurrentBehavior() const;

    static std::string BehaviorLabel(Behavior);

private:
    std::vector<Behavior> SuccessorBehaviors(const Vehicle &ego) const;
    //Vehicle GenerateTrajectory(Behavior, const VehiclePredictions&, double dt) const;
    Vehicle GenerateTrajectory(
        const Vehicle &ego, 
        const std::vector<Vehicle> &others, 
        Behavior) const;

    // Vehicle ConstantSpeedTrajectory(const VehiclePredictions&, double dt) const;
    // Vehicle KeepLaneTrajectory(const VehiclePredictions&, double dt) const;
    // Vehicle PrepareLaneChangeTrajectory(bool left, const VehiclePredictions&, double dt) const;
    // Vehicle LaneChangeTrajectory(bool left, const VehiclePredictions&, double dt) const;
    Vehicle KeepLaneTrajectory(
        const Vehicle &ego, 
        const std::vector<Vehicle> &others) const;
    
    Vehicle PrepareLaneChangeTrajectory(
        const Vehicle &ego, 
        const std::vector<Vehicle> &others, 
        bool moveLeft) const;

    Vehicle LaneChangeTrajectory(
        const Vehicle &ego, 
        const std::vector<Vehicle> &others, 
        bool moveLeft) const;

    //Kinematics GetKinematicsForLane(const VehiclePredictions&, size_t lane, double dt) const;
    Kinematics GetLaneKinematics(
        const Vehicle &ego, 
        const std::vector<Vehicle> &others, 
        size_t lane, 
        double dt) const;
    //Vehicle GetNearestVehicleInLane(size_t lane, bool lookAhead, const VehiclePredictions&) const;

    // bool TryGetVehicleAhead(
    //     const VehiclePredictions&, 
    //     const Vehicle &ego,
    //     size_t lane, 
    //     size_t &ID) const;

    // bool TryGetVehicleBehind(
    //     const VehiclePredictions&, 
    //     const Vehicle &ego,
    //     size_t lane, 
    //     size_t &ID) const;

    // bool TryGetNearestVehicle(
    //     const VehiclePredictions&,
    //     const Vehicle &ego,
    //     bool lookAhead,
    //     size_t lane,
    //     size_t &ID) const;

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

    //double TrajectoryCost(const VehicleTrajectory&, const VehiclePredictions&) const;
    //double EfficiencyCost(double velocity) const;
    double TrajectoryCost(
        const Vehicle &ego,
        const Vehicle &next,
        const std::vector<Vehicle> &others,
        Behavior) const;

    double SpeedCost(double speed) const;

    DrivingParameters _drivingParams;
    HighwayParameters _highwayParams;
    Behavior _currentBehavior;
};

#endif // __BEHAVIOR_PLANNER_H__
 