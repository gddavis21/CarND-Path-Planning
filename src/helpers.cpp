#include "helpers.h"
#include "Eigen/Dense"
#include <limits>
#include <utility>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

const double PI = M_PI;
const double TWO_PI = 2*PI;
const double HALF_PI = PI/2;

const static double m_per_mm = 1e-3;
const static double mm_per_inch = 25.4;
const static double inch_per_mile = 12 * 5280;
const static double m_per_mile = m_per_mm * mm_per_inch * inch_per_mile;
const static double sec_per_hr = 60 * 60;

double mph_to_mps(double mph) {
    return mph * m_per_mile / sec_per_hr;
}

double mps_to_mph(double mps) {
    return mps * sec_per_hr / m_per_mile;
}

double deg_to_rad(double deg) { 
    return deg * PI / 180; 
}

double rad_to_deg(double rad) { 
    return rad * 180 / PI; 
}


array<double, 6> ComputeJerkMinimizingTrajectory(
    Kinematics initialState,
    Kinematics finalState,
    double elapsedTime)
{
    double si = initialState.position;
    double vi = initialState.velocity;
    double ai = initialState.acceleration;

    double sf = finalState.position;
    double vf = finalState.velocity;
    double af = finalState.acceleration;

    double T = elapsedTime;
    double T2 = T*T;
    double T3 = T2*T;
    double T4 = T3*T;
    double T5 = T4*T;

    Eigen::Matrix3d A;
    A << T3, T4, T5,
        3*T2, 4*T3, 5*T4,
        6*T, 12*T2, 20*T3;

    Eigen::Vector3d b;
    b << sf - (si + vi*T + 0.5*ai*T2),
        vf - (vi + ai*T),
        af - ai;

    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
    return { si, vi, 0.5*ai, x[0], x[1], x[2] };
}

Vector2D::Vector2D(double x, double y) 
{
    this->x = x;
    this->y = y;
}

double Vector2D::Length() const {
    return sqrt(x*x + y*y);
}

double Vector2D::DistanceTo(Vector2D other) const {
    return (*this - other).Length();
}

double Vector2D::Distance(Vector2D u, Vector2D v) {
    return u.DistanceTo(v);
}

Vector2D Vector2D::Unit() const {
    return (*this) * (1.0/Length());
}

Vector2D Vector2D::Polar(double angle) {
    return Vector2D(cos(angle), sin(angle));
}

double Vector2D::Dot(Vector2D u, Vector2D v) {
    return u.x*v.x + u.y*v.y;
}

double Vector2D::PerpDot(Vector2D u, Vector2D v) {
    return u.x*v.y - u.y*v.x;
}

Vector2D Vector2D::Perp() const {
    return Vector2D(-y, x);
}
