#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <array>
#include <vector>
#include <map>
#include "helpers.h"
#include "SimulatorData.h"
#include "json.hpp"
#include "Highway.h"
#include "Vehicle.h"
#include "PathPlanner.h"

// for convenience
using nlohmann::json;
using namespace std;
using std::string;
using std::array;
using std::vector;

// // Checks if the SocketIO event has JSON data.
// // If there is data the JSON object in string format will be returned,
// //   else the empty string "" will be returned.
// string ReadInputMessage(const string &data) 
// {
//     if (data.find("null") != string::npos)
//         return "";

//     string::size_type b1 = data.find_first_of("[");
//     string::size_type b2 = data.find_first_of("}");

//     if (b1 != string::npos && b2 != string::npos) 
//         return data.substr(b1, b2 - b1 + 2);

//     return "";
// }

// struct EgoVehicleState
// {
//     double x;
//     double y;
//     double s;
//     double d;
//     double yaw;
//     double speed;
// };

// EgoVehicleState ReadEgoVehicle(json &j)
// {
//     // Main car's localization Data
//     return {
//         .x = j[1]["x"],
//         .y = j[1]["y"],
//         .s = j[1]["s"],
//         .d = j[1]["d"],
//         .yaw = deg_to_rad(j[1]["yaw"]),
//         .speed = mph_to_mps(j[1]["speed"])
//     };
// }

// struct OtherVehicleState
// {
//     size_t ID;
//     double x;
//     double y;
//     double vx;
//     double vy;
//     double s;
//     double d;
// };

// vector<OtherVehicleState> ReadOtherVehicles(json &j)
// {
//     // Sensor Fusion Data, a list of all other cars on the same side of the road.
//     auto sensor_fusion = j[1]["sensor_fusion"];
//     vector<OtherVehicleState> otherVehicles;

//     for (size_t i=0; i < sensor_fusion.size(); i++)
//     {
//         otherVehicles.push_back({
//             .ID = (size_t)sensor_fusion[i][0],
//             .x = sensor_fusion[i][1],
//             .y = sensor_fusion[i][2],
//             .vx = sensor_fusion[i][3],
//             .vy = sensor_fusion[i][4],
//             .s = sensor_fusion[i][5],
//             .d = sensor_fusion[i][6]
//         });
//     }

//     return otherVehicles;
// }

// string hasData(string s) 
// {
//     auto found_null = s.find("null");
//     auto b1 = s.find_first_of("[");
//     auto b2 = s.find_first_of("}");

//     if (found_null != string::npos) {
//         return "";
//     } 
//     else if (b1 != string::npos && b2 != string::npos) {
//         return s.substr(b1, b2 - b1 + 2);
//     }
//     return "";
// }

// Load up map values for waypoint's x,y,s and d normalized normal vectors
vector<HighwayWaypoint> LoadWaypoints(const string &filePath)
{
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<HighwayWaypoint> waypoints;

    // // Waypoint map to read from
    // string map_file_ = "../data/highway_map.csv";

    ifstream f(filePath.c_str(), ifstream::in);
    string line;

    while (getline(f, line)) 
    {
        istringstream iss(line);
        HighwayWaypoint wp;
        iss >> wp.x >> wp.y >> wp.s >> wp.dx >> wp.dy;
        waypoints.push_back(wp);
    }

    return waypoints;
}

// // const double TARGET_SPEED = 49; // MPH
// const double SPEED_LIMIT = mph_to_mps(50);  // 50 MPH => m/s
// const double SPEED_LIMIT_BUFFER = mph_to_mps(2);
// const double ACCEL_LIMIT = 10;  // m/s^2
// const double JERK_LIMIT = 10;   // m/s^3

// // The max s value before wrapping around the track back to 0
// const double WRAP_AROUND = 6945.554;

const HighwayParameters HIGHWAY_PARAMS = {
    .LaneCount = 3,
    .LaneWidth = 4.0,
    .SpeedLimit = mph_to_mps(50),
    .WrapAroundPosition = 6945.554
};

const DrivingParameters DRIVING_PARAMS = {
    .AccelerationLimit = 5,     // m/s^2
    .JerkLimit = 9,             // m/s^3
    .FollowBuffer = 1.5,
    .SpeedLimitBuffer = mph_to_mps(5),
    .StopCost = 0.8
};

const string WAYPOINTS_PATH = "../data/highway_map.csv";

int main() 
{
    PathPlanner pathPlanner(DRIVING_PARAMS, HIGHWAY_PARAMS, LoadWaypoints(WAYPOINTS_PATH));
    uWS::Hub h;

    // h.onMessage([&hwyParams, &drivingParams, &hwyCoords, &egoVehicle](
    h.onMessage([&pathPlanner](
        uWS::WebSocket<uWS::SERVER> ws, 
        char *data, 
        size_t length,
        uWS::OpCode opCode) 
    {
        SimulatorData simData(data, length);

        if (simData.GetInputType() == SimulatorInputType::Telemetry)
        {
            vector<Vector2D> previousPath = simData.GetPreviousPath();
            PathPlanner::EgoVehicleState egoVehicle = simData.GetEgoVehicle();
            vector<PathPlanner::OtherVehicleState> otherVehicles = simData.GetOtherVehicles();

            // cout << "previous path count: " << previousPath.size() << endl;

            vector<Vector2D> nextPath = pathPlanner.PlanNextPath(previousPath, egoVehicle, otherVehicles);

            // cout << "next path count: " << nextPath.size() << endl << endl;

            cout << endl;

            simData.SetNextPath(nextPath);
        }

        string outputMsg = simData.FormatOutputMsg();

        if (!outputMsg.empty()) {
            ws.send(outputMsg.data(), outputMsg.length(), uWS::OpCode::TEXT);
        }
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) 
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](
        uWS::WebSocket<uWS::SERVER> ws, 
        int code, 
        char *message, 
        size_t length) 
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;

    if (h.listen(port)) 
    {
        std::cout << "Listening to port " << port << std::endl;
    } 
    else 
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}
