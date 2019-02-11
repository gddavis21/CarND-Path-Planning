#include "helpers.h"
#include "Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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

    Matrix3d A;
    A << T3, T4, T5,
        3*T2, 4*T3, 5*T4,
        6*T, 12*T2, 20*T3;

    Vector3d b;
    b << sf - (si + vi*T + 0.5*ai*T2),
        vf - (vi + ai*T),
        af - ai;

    Vector3d x = A.colPivHouseholderQr().solve(b);
    return { si, vi, 0.5*ai, x[0], x[1], x[2] };
}
