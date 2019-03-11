#include "SimulatorData.h"
#include "json.hpp"

using namespace std;
using nlohmann::json;

namespace
{
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    bool IsWebsocketEvent(const char *data, size_t length)
    {
        return length && length > 2 && data[0] == '4' && data[1] == '2';
    }

    // Checks if the SocketIO event has JSON data.
    // If there is data the JSON object in string format will be returned,
    //   else the empty string will be returned.
    string ReadWebsocketMessage(const string &data) 
    {
        if (data.find("null") != string::npos)
            return "";

        string::size_type b1 = data.find_first_of("[");
        string::size_type b2 = data.find_first_of("}");

        if (b1 != string::npos && b2 != string::npos) 
            return data.substr(b1, b2 - b1 + 2);

        return "";
    }
}

SimulatorData::SimulatorData(const char *data, size_t length)
{
    _inputType = SimulatorInputType::None;
    string inputMsg;

    if (IsWebsocketEvent(data, length))
    {
        inputMsg = ReadWebsocketMessage(string(data));

        if (inputMsg.empty())
        {
            _inputType = SimulatorInputType::ManualDriving;
        }
        else
        {
            json j = json::parse(inputMsg);
            string event = j[0].get<string>();

            if (event == "telemetry")
            {
                _inputType = SimulatorInputType::Telemetry;

                // Main car's localization Data
                _egoVehicle = {
                    .x = j[1]["x"],
                    .y = j[1]["y"],
                    .s = j[1]["s"],
                    .d = j[1]["d"],
                    .yaw = deg_to_rad(j[1]["yaw"]),
                    .speed = mph_to_mps(j[1]["speed"])
                };

                // Sensor Fusion Data, a list of all other cars on the same side of the road.
                auto sensorFusion = j[1]["sensor_fusion"];
                _otherVehicles.reserve(sensorFusion.size());

                for (size_t i=0; i < sensorFusion.size(); i++)
                {
                    _otherVehicles.push_back({
                        .ID = (size_t)sensorFusion[i][0],
                        .x = sensorFusion[i][1],
                        .y = sensorFusion[i][2],
                        .vx = sensorFusion[i][3],
                        .vy = sensorFusion[i][4],
                        .s = sensorFusion[i][5],
                        .d = sensorFusion[i][6]
                    });
                }

                // Previous path data given to the Planner
                vector<double> prev_path_x = j[1]["previous_path_x"];
                vector<double> prev_path_y = j[1]["previous_path_y"];
                _previousPath.reserve(prev_path_x.size());

                for (size_t i=0; i < prev_path_x.size(); i++)
                {
                    _previousPath.push_back(Vector2D(prev_path_x[i], prev_path_y[i]));
                }

                // Previous path's end s and d values (currently unused)
                double end_path_s = j[1]["end_path_s"];
                double end_path_d = j[1]["end_path_d"];
            }
        }
    }
}

string SimulatorData::FormatOutputMsg() const
{
    string output;

    if (_inputType == SimulatorInputType::ManualDriving) 
    {
        output = "42[\"manual\",{}]";
    }
    else if (_inputType == SimulatorInputType::Telemetry)
    {
        vector<double> next_x_vals(_nextPath.size());
        vector<double> next_y_vals(_nextPath.size());

        for (size_t i=0; i < _nextPath.size(); i++)
        {
            next_x_vals[i] = _nextPath[i].x;
            next_y_vals[i] = _nextPath[i].y;
        }

        json j;
        j["next_x"] = next_x_vals;
        j["next_y"] = next_y_vals;
        output = "42[\"control\"," + j.dump() + "]";
    }

    return output;
}

