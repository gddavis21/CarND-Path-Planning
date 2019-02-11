#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

struct Waypoint 
{
    double x;   // Cartesian x coordinate relative to map
    double y;   // Cartesian y coordinate relative to map
    double s;   // Frenet arc-length coordinate
    double dx;  // Frenet unit normal x component
    double dy;  // Frenet unit normal y component
};

struct OtherVehicleState
{
    size_t ID;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
};

const double TARGET_SPEED = 49; // MPH

// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

int main() 
{
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<Waypoint> hwy_waypoints;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";

    ifstream in_map_(map_file_.c_str(), ifstream::in);
    string line;

    while (getline(in_map_, line)) 
    {
        istringstream iss(line);
        Waypoint wp;
        iss >> wp.x >> wp.y >> wp.s >> wp.dx >> wp.dy;
        hwy_waypoints.push_back(wp);
    }

    h.onMessage([&hwy_waypoints](
        uWS::WebSocket<uWS::SERVER> ws, 
        char *data, 
        size_t length,
        uWS::OpCode opCode) 
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') 
        {
            string s = hasData(data);

            if (s != "") 
            {
                json j = json::parse(s);
                string event = j[0].get<string>();

                if (event == "telemetry") 
                {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    vector<double> prev_path_x = j[1]["previous_path_x"];
                    vector<double> prev_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values 
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    json sensor_fusion = j[1]["sensor_fusion"];
                    vector<OtherVehicle> otherVehicles;

                    for (size_t i=0; i < sensor_fusion.size(); i++) 
                    {
                        size_t veh_ID = (size_t)sensor_fusion[i][0];
                        double veh_x = sensor_fusion[i][1];
                        double veh_y = sensor_fusion[i][2];
                        double veh_vx = sensor_fusion[i][3];
                        double veh_vy = sensor_fusion[i][4];
                        double veh_s = sensor_fusion[i][5];
                        double veh_d = sensor_fusion[i][6];

                        otherVehicles.push_back(OtherVehicle(
                            veh_ID,
                            WhichLane(veh_d),
                            KinematicState(veh_s, sqrt(veh_vx*veh_vx + veh_vy*veh_vy), 0.0)));
                    }

                    std::map<size_t, vector<OtherVehicle>> predictions;

                    for (size_t i=0; i < otherVehicles.size(); i++) {
                        predictions[otherVehicles[i].GetID()] = otherVehicles[i].GeneratePredictions(2);
                    }


                    // TODO: define a path made up of (x,y) points that the car will visit
                    // sequentially every .02 seconds
                    vector<double> next_x_vals, next_y_vals;


                    json msgJson;
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } 
            else 
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
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
