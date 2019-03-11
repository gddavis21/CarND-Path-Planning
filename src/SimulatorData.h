#ifndef __SIMULATOR_DATA_H__
#define __SIMULATOR_DATA_H__

#include <vector>
#include <string>
#include "helpers.h"
#include "PathPlanner.h"

enum class SimulatorInputType
{
    None,
    ManualDriving,
    Telemetry
};

class SimulatorData
{
public:
    SimulatorData(const char *data, size_t length);

    SimulatorInputType GetInputType() const;

    PathPlanner::EgoVehicleState GetEgoVehicle() const;
    std::vector<PathPlanner::OtherVehicleState> GetOtherVehicles() const;
    std::vector<Vector2D> GetPreviousPath() const;
    void SetNextPath(const std::vector<Vector2D>&);

    std::string FormatOutputMsg() const;

private:
    SimulatorInputType _inputType;
    PathPlanner::EgoVehicleState _egoVehicle;
    std::vector<PathPlanner::OtherVehicleState> _otherVehicles;
    std::vector<Vector2D> _previousPath;
    std::vector<Vector2D> _nextPath;
};

inline SimulatorInputType SimulatorData::GetInputType() const {
    return _inputType;
}

inline PathPlanner::EgoVehicleState SimulatorData::GetEgoVehicle() const {
    return _egoVehicle;
}

inline std::vector<PathPlanner::OtherVehicleState> SimulatorData::GetOtherVehicles() const {
    return _otherVehicles;
}

inline std::vector<Vector2D> SimulatorData::GetPreviousPath() const {
    return _previousPath;
}

inline void SimulatorData::SetNextPath(const std::vector<Vector2D> &path) {
    _nextPath = path;
}

#endif // __SIMULATOR_DATA_H__