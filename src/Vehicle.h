#ifndef __VEHICLE_H__
#define __VEHICLE_H__

#include "helpers.h"
#include "Highway.h"
// #include <memory>
// #include <array>
// #include <vector>
// #include <unordered_map>

// class Vehicle;
// typedef std::shared_ptr<Vehicle> VehiclePtr;

class Vehicle
{
public:
    Vehicle(
        const HighwayParameters &hwyParams, 
        const Kinematics &longKinematics, 
        double lateralPos, 
        double time);

    Vehicle();

    bool IsValid() const;
    double Time() const;

    Kinematics GetKinematics() const;

    double Position() const;
    double Velocity() const;
    double Acceleration() const;

    double LateralPosition() const;
    size_t Lane() const;

    Vehicle Predict(double dt) const;

    // how far behind other vehicle am I? (normalized for track wrap-around)
    double DistanceBehind(const Vehicle &other) const;

protected:
    bool _valid;
    HighwayParameters _hwyParams;
    Kinematics _longKinematics;
    double _lateralPos;
    double _time;
};

// typedef std::array<Vehicle, 2> VehicleTrajectory;
// typedef std::unordered_map<size_t, VehicleTrajectory> VehiclePredictions;

// struct DrivingParameters
// {
//     double AccelerationLimit;
//     double JerkLimit;
//     double FollowBuffer;
//     double SpeedLimitBuffer;
//     double StopCost;
// };

// class EgoVehicle
// {
// public:
//     EgoVehicle(HighwayParameters hwyParams, DrivingParameters drivingParams);

//     //void UpdateState(Kinematics longKinematics, double lateralPos);
//     //VehicleTrajectory GenerateNextTrajectory(const VehiclePredictions&);

//     bool IsChangingLanes() const;

//     // void Update(
//     //     const VehiclePredictions&, 
//     //     const Vehicle &egoStart,
//     //     Vehicle &egoFinish,
//     //     double &trajTime);
//     Vehicle Update(const Vehicle &self, const std::vector<Vehicle> &others);

// private:
//     enum Behavior 
//     {
//         KeepLane,
//         PrepareLaneChangeLeft,
//         PrepareLaneChangeRight,
//         LaneChangeLeft,
//         LaneChangeRight
//     };

//     std::vector<Behavior> SuccessorBehaviors() const;
//     //Vehicle GenerateTrajectory(Behavior, const VehiclePredictions&, double dt) const;
//     Vehicle GenerateTrajectory(
//         const Vehicle &self, 
//         const std::vector<Vehicle> &others, 
//         Behavior) const;

//     // Vehicle ConstantSpeedTrajectory(const VehiclePredictions&, double dt) const;
//     // Vehicle KeepLaneTrajectory(const VehiclePredictions&, double dt) const;
//     // Vehicle PrepareLaneChangeTrajectory(bool left, const VehiclePredictions&, double dt) const;
//     // Vehicle LaneChangeTrajectory(bool left, const VehiclePredictions&, double dt) const;
//     Vehicle KeepLaneTrajectory(
//         const Vehicle &self, 
//         const std::vector<Vehicle> &others) const;
    
//     Vehicle PrepareLaneChangeTrajectory(
//         const Vehicle &self, 
//         const std::vector<Vehicle> &others, 
//         bool moveLeft) const;

//     Vehicle LaneChangeTrajectory(
//         const Vehicle &self, 
//         const std::vector<Vehicle> &others, 
//         bool moveLeft) const;

//     //Kinematics GetKinematicsForLane(const VehiclePredictions&, size_t lane, double dt) const;
//     Kinematics GetLaneKinematics(
//         const Vehicle &self, 
//         const std::vector<Vehicle> &others, 
//         size_t lane, 
//         double dt) const;
//     //Vehicle GetNearestVehicleInLane(size_t lane, bool lookAhead, const VehiclePredictions&) const;

//     // bool TryGetVehicleAhead(
//     //     const VehiclePredictions&, 
//     //     const Vehicle &self,
//     //     size_t lane, 
//     //     size_t &ID) const;

//     // bool TryGetVehicleBehind(
//     //     const VehiclePredictions&, 
//     //     const Vehicle &self,
//     //     size_t lane, 
//     //     size_t &ID) const;

//     // bool TryGetNearestVehicle(
//     //     const VehiclePredictions&,
//     //     const Vehicle &self,
//     //     bool lookAhead,
//     //     size_t lane,
//     //     size_t &ID) const;

//     bool TryGetVehicleAhead(
//         const Vehicle &self, 
//         const std::vector<Vehicle> &others, 
//         size_t lane, 
//         size_t &indexAhead,
//         double &timeAhead) const;

//     bool TryGetVehicleBehind(
//         const Vehicle &self, 
//         const std::vector<Vehicle> &others, 
//         size_t lane, 
//         size_t &indexBehind,
//         double &timeBehind) const;

//     bool TryGetNearestVehicle(
//         const Vehicle &self, 
//         const std::vector<Vehicle> &others, 
//         size_t lane, 
//         bool lookAhead, 
//         size_t &vehicleIndex) const;

//     //double TrajectoryCost(const VehicleTrajectory&, const VehiclePredictions&) const;
//     //double EfficiencyCost(double velocity) const;
//     double TrajectoryCost(
//         const Vehicle &self,
//         const Vehicle &next,
//         const std::vector<Vehicle> &others,
//         Behavior) const;

//     double SpeedCost(double speed) const;

//     DrivingParameters _drivingParams;
//     Behavior _currentBehavior;
// };

inline bool Vehicle::IsValid() const {
    return _valid;
}

inline double Vehicle::Time() const {
    return _time;
}

inline Kinematics Vehicle::GetKinematics() const {
    return _longKinematics;
}

inline double Vehicle::Position() const {
    return _longKinematics.position;
}

inline double Vehicle::Velocity() const {
    return _longKinematics.velocity;
}

inline double Vehicle::Acceleration() const {
    return _longKinematics.acceleration;
}

inline double Vehicle::LateralPosition() const {
    return _lateralPos;
}

inline size_t Vehicle::Lane() const {
    return _hwyParams.WhichLane(_lateralPos);
}

#endif // __VEHICLE_H__