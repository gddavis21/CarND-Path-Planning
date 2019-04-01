#ifndef __BEZIER_H__
#define __BEZIER_H__

#include "helpers.h"

class BezierCurve
{
public:
    static BezierCurve FromControlPoints(
        Vector2D C1, Vector2D C2,
        Vector2D C3, Vector2D C4);

    // static BezierCurve FromEndpointsAndTangents(
    //     Vector2D P1, Vector2D P2,   // start and end points
    //     Vector2D T1, Vector2D T2,   // start and end tangents
    //     double alpha);

    static BezierCurve Interpolate(
        Vector2D P1, Vector2D P2, Vector2D P3, Vector2D P4);

    static BezierCurve FromKeyframes(Pose2D start, Pose2D finish);

    Vector2D Evaluate(double t) const;
    Vector2D FirstDeriv(double t) const;

    std::vector<Vector2D> Render(
        double initSpeed,
        double goalSpeed,
        double maxAccel) const;

private:
    BezierCurve(Vector2D C1, Vector2D C2, Vector2D C3, Vector2D C4);
    Vector2D C1, C2, C3, C4;
};

#endif // __BEZIER_H__