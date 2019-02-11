#ifndef OTHER_VEHICLE_H
#define OTHER_VEHICLE_H

#include "helpers.h"
#include <array>
#include <vector>
#include <unordered_map>

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

    std::vector<Vehicle> GeneratePredictions(unsigned int horizon);

private:
    RoadParameters _roadParams;
    Kinematics _longKinematics;
    double _lateralPos;
};

typedef std::unordered_map<size_t, std::vector<Vehicle>> VehiclePredictions;
typedef std::array<Vehicle, 2> VehicleTrajectory;

class EgoVehicle : public Vehicle 
{
public:
    EgoVehicle(
        RoadParameters roadParams, 
        DrivingParameters drivingParams, 
        Kinematics longKinematics, 
        double lateralPos);

    void UpdateState(Kinematics longKinematics, double lateralPos);
    VehicleTrajectory GenerateNextTrajectory(const VehiclePredictions&);

private:
    enum Behavior 
    {
        KeepLane,
        PrepareLaneChangeLeft,
        PrepareLaneChangeRight,
        LaneChangeLeft,
        LaneChangeRight
    };

    enum LaneChangeDirection { Left, Right };

    std::vector<Behavior> SuccessorStates() const;

    bool CreateConstantSpeedTrajectory(const VehiclePredictions&, VehicleTrajectory&) const;
    bool CreateKeepLaneTrajectory(const VehiclePredictions&, VehicleTrajectory&) const;

    bool CreatePrepareLaneChangeTrajectory(
        LaneChangeDirection direction,
        const VehiclePredictions &predictions,
        VehicleTrajectory &trajectory) const;

    bool CreateLaneChangeTrajectory(
        LaneChangeDirection direction,
        const VehiclePredictions &predictions,
        VehicleTrajectory &trajectory) const;

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