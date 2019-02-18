#ifndef OTHER_VEHICLE_H
#define OTHER_VEHICLE_H

#include "helpers.h"
#include <memory>
#include <array>
#include <vector>
#include <unordered_map>

class Vehicle;
typedef std::shared_ptr<Vehicle> VehiclePtr;

class Vehicle
{
public:
    Vehicle(RoadParameters roadParams, Kinematics longKinematics, double lateralPos);

    Kinematics GetLongitudinalKinematics() const;

    double GetLongitudinalPosition() const;
    double GetLongitudinalVelocity() const;
    double GetLongitudinalAcceleration() const;

    double GetLateralPosition() const;
    size_t GetLane() const;

    VehiclePtr Predict(double dt) const;

    // how far behind other vehicle am I?
    double DistanceAhead(const Vehicle &other) const;
    double DistanceBehind(const Vehicle &other) const;

protected:
    RoadParameters _roadParams;
    Kinematics _longKinematics;
    double _lateralPos;
};

typedef std::array<VehiclePtr, 2> VehicleTrajectory;
typedef std::unordered_map<size_t, VehicleTrajectory> VehiclePredictions;

class EgoVehicle : public Vehicle 
{
public:
    EgoVehicle(
        DrivingParameters drivingParams, 
        RoadParameters roadParams, 
        Kinematics longKinematics, 
        double lateralPos);

    //void UpdateState(Kinematics longKinematics, double lateralPos);
    //VehicleTrajectory GenerateNextTrajectory(const VehiclePredictions&);

    VehicleTrajectory PlanBehavior(const VehiclePredictions&, double dt);

private:
    enum Behavior 
    {
        KeepLane,
        PrepareLaneChangeLeft,
        PrepareLaneChangeRight,
        LaneChangeLeft,
        LaneChangeRight
    };

    std::vector<Behavior> SuccessorBehaviors() const;
    VehiclePtr GenerateTrajectory(Behavior, const VehiclePredictions&, double dt) const;

    VehiclePtr ConstantSpeedTrajectory(const VehiclePredictions&, double dt) const;
    VehiclePtr KeepLaneTrajectory(const VehiclePredictions&, double dt) const;
    VehiclePtr PrepareLaneChangeTrajectory(bool left, const VehiclePredictions&, double dt) const;
    VehiclePtr LaneChangeTrajectory(bool left, const VehiclePredictions&, double dt) const;

    Kinematics GetKinematicsForLane(size_t lane, const VehiclePredictions&, double dt) const;
    VehiclePtr GetNearestVehicleInLane(size_t lane, bool lookAhead, const VehiclePredictions&) const;

    bool TryGetVehicleAhead(const VehiclePredictions&, size_t lane, size_t &ID) const;
    bool TryGetVehicleBehind(const VehiclePredictions&, size_t lane, size_t &ID) const;

    bool TryGetNearestVehicle(
        const VehiclePredictions&,
        bool lookAhead,
        size_t lane,
        size_t &ID) const;

    double TrajectoryCost(const VehicleTrajectory&, const VehiclePredictions&) const;

    DrivingParameters _drivingParams;
    Behavior _currentBehavior;
};

inline Kinematics Vehicle::GetLongKinematics() const {
    return _longKinematics;
}

inline double Vehicle::GetLongitudinalPosition() const {
    return _longKinematics.position;
}

inline double Vehicle::GetLongitudinalVelocity() const {
    return _longKinematics.velocity;
}

inline double Vehicle::GetLongitudinalAcceleration() const {
    return _longKinematics.acceleration;
}

inline double Vehicle::GetLateralPosition() const {
    return _lateralPos;
}

inline size_t Vehicle::GetLane() const {
    return _roadParams.WhichLane(_lateralPos);
}

#endif // OTHER_VEHICLE_H