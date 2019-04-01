#include "Bezier.h"

using namespace std;

BezierCurve::BezierCurve(Vector2D C1, Vector2D C2, Vector2D C3, Vector2D C4)
{
    this->C1 = C1;
    this->C2 = C2;
    this->C3 = C3;
    this->C4 = C4;
}

BezierCurve BezierCurve::FromControlPoints(Vector2D C1, Vector2D C2, Vector2D C3, Vector2D C4)
{
    return BezierCurve(C1, C2, C3, C4);
}

// BezierCurve BezierCurve::FromEndpointsAndTangents(
//     Vector2D P1, Vector2D P2,
//     Vector2D T1, Vector2D T2,
//     double alpha)
// {
//     alpha = clamp(alpha, 0.01, 0.99);
//     double f = P1.DistanceTo(P2) * 0.5 * alpha;
//     return BezierCurve(P1, P1 + f*T1.Unit(), P2 - f*T2.Unit(), P2);
// }

BezierCurve BezierCurve::FromKeyframes(Pose2D start, Pose2D finish)
{
    double f = start.location.DistanceTo(finish.location) / 6;

    return BezierCurve(
        start.location,
        start.location + f*start.direction,
        finish.location - f*finish.direction,
        finish.location);
}

BezierCurve BezierCurve::Interpolate(Vector2D P1, Vector2D P2, Vector2D P3, Vector2D P4)
{
    double f = 1.0/6;
    return BezierCurve(P2, P2 + f*(P3-P1), P3 + f*(P2-P4), P3);
}

Vector2D BezierCurve::Evaluate(double t) const
{
    // Vector2D P11 = Vector2D::Lerp(C1, C2, t);
    // Vector2D P12 = Vector2D::Lerp(C2, C3, t);
    // Vector2D P13 = Vector2D::Lerp(C3, C4, t);
    // Vector2D P21 = Vector2D::Lerp(P11, P12, t);
    // Vector2D P22 = Vector2D::Lerp(P12, P13, t);
    // return Vector2D::Lerp(P21, P22, t);

    double b1 = (1-t)*(1-t)*(1-t);
    double b2 = 3*t*(1-t)*(1-t);
    double b3 = 3*t*t*(1-t);
    double b4 = t*t*t;
    return b1*C1 + b2*C2 + b3*C3 + b4*C4;
}

Vector2D BezierCurve::FirstDeriv(double t) const
{
    // Vector2D Q1 = 3*(C2-C1);
    // Vector2D Q2 = 3*(C3-C2);
    // Vector2D Q3 = 3*(C4-C3);

    // Vector2D P11 = Vector2D::Lerp(Q1, Q2, t);
    // Vector2D P12 = Vector2D::Lerp(Q2, Q3, t);
    // return Vector2D::Lerp(P11, P12, t);

    double b1 = 3*(1-t)*(1-t);
    double b2 = 6*t*(1-t);
    double b3 = 3*t*t;
    return b1*(C2-C1) + b2*(C3-C2) + b3*(C4-C3);
}

vector<Vector2D> BezierCurve::Render(
    double initSpeed,
    double goalSpeed,
    double maxAccel) const
{
    double t_prev = 0;
    double ds = initSpeed;
    vector<Vector2D> curve;

    for (;;)
    {
        ds = clamp(goalSpeed, ds - maxAccel, ds + maxAccel);
        Vector2D v = FirstDeriv(t_prev);
        double t = t_prev + ds/v.Length();

        if (t > 1.0)
            break;

        curve.push_back(Evaluate(t));
        t_prev = t;
    }

    return curve;
}