#ifndef __HELPERS_H__
#define __HELPERS_H__

#include <math.h>
#include <string>
#include <vector>
#include <array>
#include <utility>
#include <cmath>

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
// constexpr double pi() { return M_PI; }
// double deg2rad(double x) { return x * pi() / 180; }
// double rad2deg(double x) { return x * 180 / pi(); }

inline double sign(double x) {
    return (x > 0) ? 1.0 : ((x < 0.0) ? -1.0 : x);
}

// conversions between miles/hour and meters/sec
double mph_to_mps(double mph);
double mps_to_mph(double mps);

double deg_to_rad(double deg);
double rad_to_deg(double rad);

// Calculate distance between two points
inline double distance(double x1, double y1, double x2, double y2) 
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// // Calculate closest waypoint to current x, y position
// int ClosestWaypoint(
//     double x, 
//     double y, 
//     const vector<double> &maps_x, 
//     const vector<double> &maps_y) 
// {
//     double closestLen = 100000; //large number
//     int closestWaypoint = 0;

//     for (int i = 0; i < maps_x.size(); ++i) 
//     {
//         double map_x = maps_x[i];
//         double map_y = maps_y[i];
//         double dist = distance(x,y,map_x,map_y);

//         if (dist < closestLen) 
//         {
//             closestLen = dist;
//             closestWaypoint = i;
//         }
//     }

//     return closestWaypoint;
// }

// // Returns next waypoint of the closest waypoint
// int NextWaypoint(
//     double x, 
//     double y, 
//     double theta, 
//     const vector<double> &maps_x, 
//     const vector<double> &maps_y) 
// {
//     int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

//     double map_x = maps_x[closestWaypoint];
//     double map_y = maps_y[closestWaypoint];

//     double heading = atan2((map_y-y),(map_x-x));

//     double angle = fabs(theta-heading);
//     angle = std::min(2*pi() - angle, angle);

//     if (angle > pi()/2) 
//     {
//         ++closestWaypoint;
//         if (closestWaypoint == maps_x.size()) 
//         {
//             closestWaypoint = 0;
//         }
//     }

//     return closestWaypoint;
// }

// // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
// vector<double> getFrenet(
//     double x, 
//     double y, 
//     double theta, 
//     const vector<double> &maps_x, 
//     const vector<double> &maps_y) 
// {
//     int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

//     int prev_wp;
//     prev_wp = next_wp-1;

//     if (next_wp == 0) {
//         prev_wp  = maps_x.size()-1;
//     }

//     double n_x = maps_x[next_wp]-maps_x[prev_wp];
//     double n_y = maps_y[next_wp]-maps_y[prev_wp];
//     double x_x = x - maps_x[prev_wp];
//     double x_y = y - maps_y[prev_wp];

//     // find the projection of x onto n
//     double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
//     double proj_x = proj_norm*n_x;
//     double proj_y = proj_norm*n_y;

//     double frenet_d = distance(x_x,x_y,proj_x,proj_y);

//     //see if d value is positive or negative by comparing it to a center point
//     double center_x = 1000-maps_x[prev_wp];
//     double center_y = 2000-maps_y[prev_wp];
//     double centerToPos = distance(center_x,center_y,x_x,x_y);
//     double centerToRef = distance(center_x,center_y,proj_x,proj_y);

//     if (centerToPos <= centerToRef) {
//         frenet_d *= -1;
//     }

//     // calculate s value
//     double frenet_s = 0;
//     for (int i = 0; i < prev_wp; ++i) {
//         frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
//     }

//     frenet_s += distance(0,0,proj_x,proj_y);
//     return {frenet_s,frenet_d};
// }

// // Transform from Frenet s,d coordinates to Cartesian x,y
// vector<double> getXY(
//     double s, double d, 
//     const vector<double> &maps_s, 
//     const vector<double> &maps_x, 
//     const vector<double> &maps_y) 
// {
//     int prev_wp = -1;

//     while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
//         ++prev_wp;
//     }

//     int wp2 = (prev_wp+1)%maps_x.size();

//     double heading = atan2(
//         (maps_y[wp2]-maps_y[prev_wp]), 
//         (maps_x[wp2]-maps_x[prev_wp]));

//     // the x,y,s along the segment
//     double seg_s = (s-maps_s[prev_wp]);

//     double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
//     double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

//     double perp_heading = heading-pi()/2;

//     double x = seg_x + d*cos(perp_heading);
//     double y = seg_y + d*sin(perp_heading);

//     return {x,y};
// }

inline double clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(x, hi));
}


struct Vector2D
{
    double x;
    double y;

    Vector2D(double x=0, double y=0);

    double Length() const;
    double DistanceTo(Vector2D other) const;
    static double Distance(Vector2D u, Vector2D v);
    Vector2D Unit() const;
    static Vector2D Polar(double angle);
    static double Dot(Vector2D u, Vector2D v);
    static double PerpDot(Vector2D u, Vector2D v);
    Vector2D Perp() const;
};

// vector negation
inline Vector2D operator- (Vector2D v) {
    return Vector2D(-v.x, -v.y);
}

// vector addition
inline Vector2D operator+ (Vector2D u, Vector2D v) {
    return Vector2D(u.x + v.x, u.y + v.y);
}

// vector subtraction
inline Vector2D operator- (Vector2D u, Vector2D v) {
    return Vector2D(u.x - v.x, u.y - v.y);
}

// vector scaling
inline Vector2D operator* (Vector2D v, double s) {
    return Vector2D(s*v.x, s*v.y);
}

// vector scaling
inline Vector2D operator* (double s, Vector2D v) {
    return v*s;
}

// inline Vector2D operator/ (Vector2D v, double d) {
//     return Vector2D(v.x / d, v.y / d);
// }

struct Kinematics 
{
    double position;
    double velocity;
    double acceleration;

    Kinematics(double pos=0, double vel=0, double acc=0) {
        position = pos;
        velocity = vel;
        acceleration = acc;
    }

    double PositionAt(double t) const {
        return position + velocity*t + 0.5*acceleration*t*t;
    }

    double VelocityAt(double t) const {
        return velocity + acceleration*t;
    }

    Kinematics Predict(double t) const {
        return Kinematics(PositionAt(t), VelocityAt(t), acceleration);
    }
};

// struct MapCoord {
//     double x;
//     double y;
// };

struct Frenet2D {
    double s;
    double d;
};

std::array<double, 6> ComputeJerkMinimizingTrajectory(
    Kinematics initialState, 
    Kinematics finalState, 
    double elapsedTime);



#endif  // __HELPERS_H__