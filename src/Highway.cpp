#include "Highway.h"
#include <limits>
#include <iostream>
#include "Bezier.h"

using namespace std;

const double PI = M_PI;
const double TWO_PI = 2*PI;
const double HALF_PI = PI/2;

HighwayCoordinates::HighwayCoordinates(
    HighwayParameters hwyParams, 
    const vector<HighwayWaypoint> &waypoints)
{
    _hwyParams = hwyParams;
    _waypoints = waypoints;
    
    _cumDist.resize(waypoints.size());
    _cumDist[0] = 0.0;

    for (size_t i=1; i < waypoints.size(); i++)
    {
        _cumDist[i] = _cumDist[i-1] + distance(
            waypoints[i-1].x, waypoints[i-1].y, 
            waypoints[i].x, waypoints[i].y);
    }
}

// WP_citer Highway::NearestWaypoint(Vector2D mapCoord) const
// {
//     WP_citer nearest = _waypoints.end();
//     double minSqrDist = numeric_limits<double>::max();

//     for (WP_citer wp = _waypoints.begin(); wp != _waypoints.end(); wp++)
//     {
//         double dx = mapCoord.x - wp->x;
//         double dy = mapCoord.y - wp->y;
//         double sqrDist = dx*dx + dy*dy;

//         if (sqrDist < minSqrDist)
//         {
//             minSqrDist = sqrDist;
//             nearest = wp;
//         }
//     }

//     return nearest;
// }

size_t HighwayCoordinates::NearestWaypoint(Vector2D mapLoc) const
{
    // NOTE: for better performance, the following linear search could be replaced
    // with a (log time) query on a 2D spatial structure (k-d tree).
    size_t nearest = numeric_limits<size_t>::max();
    double minSqrDist = numeric_limits<double>::max();

    for (size_t i=0; i < _waypoints.size(); i++)
    {
        double dx = mapLoc.x - _waypoints[i].x;
        double dy = mapLoc.y - _waypoints[i].y;
        double sqrDist = dx*dx + dy*dy;

        if (sqrDist < minSqrDist)
        {
            minSqrDist = sqrDist;
            nearest = i;
        }
    }

    return nearest;
}

// WP_citer Highway::NextWaypoint(Vector2D mapCoord, double heading) const
// {
//     WP_citer wp = NearestWaypoint(mapCoord);
//     double angle = fabs(atan2(wp->y - mapCoord.y, wp->x - mapCoord.x) - heading);

//     if (min(angle, TWO_PI-angle) > HALF_PI)
//     {
//         wp++;

//         if (wp == _waypoints.end()) {
//             wp = _waypoints.begin();
//         }
//     }

//     return wp;
// }

// size_t HighwayCoordinates::NextWaypoint(Vector2D mapLoc, double heading) const
// {
//     size_t indexNear = NearestWaypoint(mapLoc);
//     size_t indexNext = indexNear;

//     HighwayWaypoint nearWP = _waypoints[indexNear];
//     double angle = fabs(atan2(nearWP.y - mapLoc.y, nearWP.x - mapLoc.x) - heading);

//     if (min(angle, TWO_PI-angle) > HALF_PI)
//         indexNext++;

//     return (indexNext == 0) ? _waypoints.size() : indexNext;
// }

size_t HighwayCoordinates::NextWaypoint(Pose2D pose) const
{
    size_t index_near = NearestWaypoint(pose.location);
    size_t index_next = index_near;
    Vector2D dir_wp = WaypointPose(index_near).location - pose.location;

    if (dir_wp.Dot(pose.direction) < 0.0)
        index_next++;

    return (index_next == 0) ? _waypoints.size() : index_next;
}

size_t HighwayCoordinates::NextWaypoint(Frenet2D hwy) const
{
    double s = fmod(hwy.s, _hwyParams.WrapAroundPosition);
    size_t index_next = 1;

    while (index_next < _waypoints.size() && _waypoints[index_next].s < s)
        index_next++;

    return index_next;
}

// pair<size_t, size_t> Highway::FindInterval(Vector2D mapCoord, double heading) const
// {
//     size_t index = NearestWaypoint(mapCoord);
//     Waypoint near = _waypoints[index];
//     double angle = fabs(atan2(near.y - mapCoord.y, near.x - mapCoord.x) - heading);
//     size_t next = (min(angle, TWO_PI-angle) < HALF_PI) ? index : (index + 1);
//     size_t prev = (next == 0) ? (_waypoints.size() - 1) : (next - 1);
//     return make_pair(prev, next);
// }

// pair<size_t, size_t> Highway::FindInterval(HwyCoord hwyCoord) const
// {
//     size_t next = 1;

//     while (next < _waypoints.size() && _waypoints[next].s < hwyCoord.s) {
//         next++;
//     }

//     return make_pair(next - 1, next);
// }

// WP_citer Highway::NextWaypoint(Vector2D mapCoord, double heading) const
// {
//     size_t index = NearestWaypoint(mapCoord);
//     Waypoint near = _waypoints[index];
//     double angle = fabs(atan2(near.y - mapCoord.y, near.x - mapCoord.x) - heading);
//     return (min(angle, TWO_PI-angle) < HALF_PI) ? index : (index + 1);
// }

// pair<WP_citer, WP_citer> Highway::BoundingWaypoints(HwyCoord hwyCoord) const
// {
//     WP_citer next_wp = _waypoints.begin() + 1;

//     while (next_wp != _waypoints.end() && next_wp->s < hwyCoord.s) {
//         next_wp++;
//     }

//     WP_citer prev_wp = next_wp - 1;

//     if (next_wp == _waypoints.end()) {
//         next_wp = _waypoints.begin();
//     }

//     return make_pair(prev_wp, next_wp);
// }

// HwyCoord Highway::MapToHighway(Vector2D mapCoord, double heading) const
// {
//     WP_citer next_wp = NextWaypoint(mapCoord, heading);
//     WP_citer prev_wp = (next_wp == _waypoints.begin()) ? (_waypoints.end() - 1) : (next_wp - 1);

//     double n_x = next_wp->x - prev_wp->x;
//     double n_y = next_wp->y - prev_wp->y;
//     double x_x = mapCoord.x - prev_wp->x;
//     double x_y = mapCoord.y - prev_wp->y;

//     // find the projection of x onto n
//     double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
//     double proj_x = proj_norm*n_x;
//     double proj_y = proj_norm*n_y;

//     double frenet_d = distance(x_x, x_y, proj_x, proj_y);

//     //see if d value is positive or negative by comparing it to a center point
//     double center_x = 1000 - prev_wp->x;
//     double center_y = 2000 - prev_wp->y;
//     double centerToPos = distance(center_x, center_y, x_x, x_y);
//     double centerToRef = distance(center_x, center_y, proj_x, proj_y);

//     if (centerToPos <= centerToRef) {
//         frenet_d = -frenet_d;
//     }

//     // calculate s value
//     // TODO: replace loop with pre-calculated cumulative distance lookup
//     double frenet_s = 0;
    
//     for (WP_citer wp = _waypoints.begin(); wp != prev_wp; wp++) {
//         frenet_s += distance(wp->x, wp->y, (wp+1)->x, (wp+1)->y);
//     }

//     frenet_s += distance(0, 0, proj_x, proj_y);
//     return { .s = frenet_s, .d = frenet_d };
// }

// Frenet2D HighwayCoordinates::MapToHighway(Vector2D mapLoc, double heading) const
// {
//     size_t next = NextWaypoint(mapLoc, heading);
//     size_t prev = next - 1;

//     HighwayWaypoint prev_wp = _waypoints[prev];
//     HighwayWaypoint next_wp = _waypoints[next % _waypoints.size()];

//     double n_x = next_wp.x - prev_wp.x;
//     double n_y = next_wp.y - prev_wp.y;
//     double x_x = mapLoc.x - prev_wp.x;
//     double x_y = mapLoc.y - prev_wp.y;

//     // find the projection of x onto n
//     double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
//     double proj_x = proj_norm*n_x;
//     double proj_y = proj_norm*n_y;

//     Frenet2D hwyLoc = {
//         .s = _cumDist[prev] + distance(0, 0, proj_x, proj_y),
//         .d = distance(x_x, x_y, proj_x, proj_y)
//     };

//     //see if d value is positive or negative by comparing it to a center point
//     double center_x = 1000 - prev_wp.x;
//     double center_y = 2000 - prev_wp.y;
//     double centerToPos = distance(center_x, center_y, x_x, x_y);
//     double centerToRef = distance(center_x, center_y, proj_x, proj_y);

//     if (centerToPos <= centerToRef) {
//         hwyLoc.d = -hwyLoc.d;
//     }

//     return hwyLoc;
// }

Frenet2D HighwayCoordinates::MapToHighway(Pose2D pose) const
{
    size_t next = NextWaypoint(pose);
    size_t prev = next - 1;

    Vector2D P = WaypointPose(prev).location;
    Vector2D N = WaypointPose(next).location;
    Vector2D L = pose.location;

    double f = (L-P).Dot(N-P) / (N-P).Dot(N-P);
    Vector2D q = f*(N-P);

    return {
        .s = _cumDist[prev] + q.Length(),
        .d = L.DistanceTo(P+q) * sign((L-P).PerpDot(N-P))
    };
}

// Vector2D Highway::HighwayToMap(HwyCoord hwyCoord) const
// {
//     // find bounding waypoints
//     pair<WP_citer, WP_citer> boundWPs = BoundingWaypoints(hwyCoord, waypoints);
//     WP_citer prev_wp = boundWPs.first;
//     WP_citer next_wp = boundWPs.second;

//     // the x,y,s along the segment
//     double heading = atan2((next_wp->y - prev_wp->y), (next_wp->x - prev_wp->x));
//     double seg_s = hwyCoord.s - prev_wp->s;
//     double seg_x = prev_wp->x + seg_s*cos(heading);
//     double seg_y = prev_wp->y + seg_s*sin(heading);

//     double perp_heading = heading - HALF_PI;
//     double x = seg_x + hwyCoord.d*cos(perp_heading);
//     double y = seg_y + hwyCoord.d*sin(perp_heading);

//     return Vector2D(x,y);
// }

Pose2D HighwayCoordinates::WaypointPose(size_t index) const
{
    const HighwayWaypoint &wp = _waypoints[index % _waypoints.size()];

    return {
        .location = Vector2D(wp.x, wp.y),
        .direction = Vector2D(wp.dx, wp.dy)
    };
}

double HighwayCoordinates::WaypointPosition(size_t index) const
{
    return _waypoints[index % _waypoints.size()].s;
}

// Vector2D HighwayCoordinates::HighwayToMap(Frenet2D hwyLoc) const
// {
//     size_t next = NextWaypoint(hwyLoc);
//     size_t prev = next - 1;

//     HighwayWaypoint prev_wp = _waypoints[prev];
//     HighwayWaypoint next_wp = _waypoints[next % _waypoints.size()];

//     // the x,y,s along the segment
//     double heading = atan2((next_wp.y - prev_wp.y), (next_wp.x - prev_wp.x));
//     double s = fmod(hwyLoc.s, _hwyParams.WrapAroundPosition);
//     double seg_s = s - prev_wp.s;
//     double seg_x = prev_wp.x + seg_s*cos(heading);
//     double seg_y = prev_wp.y + seg_s*sin(heading);

//     double perp_heading = heading - HALF_PI;
//     double x = seg_x + hwyLoc.d*cos(perp_heading);
//     double y = seg_y + hwyLoc.d*sin(perp_heading);

//     return Vector2D(x,y);
// }

Pose2D HighwayCoordinates::HighwayToMap(Frenet2D hwy) const
{
    size_t next = NextWaypoint(hwy);
    size_t prev = next - 1;

    size_t next_next = next + 1;
    size_t prev_prev = (prev > 0) ? (prev - 1) : (_waypoints.size() - 1);

    Pose2D WP0 = WaypointPose(prev_prev);
    Pose2D WP1 = WaypointPose(prev);
    Pose2D WP2 = WaypointPose(next);
    Pose2D WP3 = WaypointPose(next_next);

    BezierCurve bez = BezierCurve::Interpolate(
        WP0.location + WP0.direction*hwy.d,
        WP1.location + WP1.direction*hwy.d,
        WP2.location + WP2.direction*hwy.d,
        WP3.location + WP3.direction*hwy.d);

    double s = fmod(hwy.s, _hwyParams.WrapAroundPosition);
    double sP = WaypointPosition(prev);
    double sN = WaypointPosition(next);

    if (sN < sP)
        sN += _hwyParams.WrapAroundPosition;

    double t = (s-sP) / (sN-sP);

    return {
        .location = bez.Evaluate(t),
        .direction = bez.FirstDeriv(t).Unit()
    };
}

vector<Pose2D> HighwayCoordinates::HighwayToMap(const vector<Frenet2D> &hwy) const
{
    vector<Pose2D> path(hwy.size());

    for (size_t i=0; i < hwy.size(); i++) {
        path[i] = HighwayToMap(hwy[i]);
    }

    return path;
}

// vector<Vector2D> HighwayCoordinates::HighwayToMap(const vector<Frenet2D> &hwyLocs) const
// {
//     //cout << "HighwayToMap find next waypoint" << endl;
//     size_t next = NextWaypoint(hwyLocs[0]);
//     size_t prev = next - 1;

//     //cout << "HighwayToMap compute angles" << endl;
//     HighwayWaypoint prev_wp = _waypoints[prev];
//     HighwayWaypoint next_wp = _waypoints[next % _waypoints.size()];

//     double heading = atan2((next_wp.y - prev_wp.y), (next_wp.x - prev_wp.x));
//     double sin_heading = sin(heading);
//     double cos_heading = cos(heading);
//     double sin_perp = sin(heading - HALF_PI);
//     double cos_perp = cos(heading - HALF_PI);
            
//     //cout << "HighwayToMap convert hwy locations to map locations" << endl;
//     //vector<Vector2D> mapLocs(hwyLocs.size());
//     vector<Vector2D> mapLocs;

//     for (size_t i=0; i < hwyLocs.size(); i++)
//     {
//         //cout << "convert location #" << (i+1) << endl;
//         //cout << "hwy: s=" << hwyLocs[i].s << ", d=" << hwyLocs[i].d << endl;

//         double next_s = (next_wp.s > 0) ? next_wp.s : _hwyParams.WrapAroundPosition;

//         if (hwyLocs[i].s > next_s)
//         {
//             //cout << "passed waypoint, recompute angles" << endl;
//             prev++;
//             next++;
//             prev_wp = _waypoints[prev % _waypoints.size()];
//             next_wp = _waypoints[next % _waypoints.size()];
//             heading = atan2((next_wp.y - prev_wp.y), (next_wp.x - prev_wp.x));
//             sin_heading = sin(heading);
//             cos_heading = cos(heading);
//             sin_perp = sin(heading - HALF_PI);
//             cos_perp = cos(heading - HALF_PI);
//         }

//         double s = fmod(hwyLocs[i].s, _hwyParams.WrapAroundPosition);
//         double seg_s = s - prev_wp.s;
//         double seg_x = prev_wp.x + seg_s*cos_heading;
//         double seg_y = prev_wp.y + seg_s*sin_heading;

//         double perp_heading = heading - HALF_PI;
//         double x = seg_x + hwyLocs[i].d*cos_perp;
//         double y = seg_y + hwyLocs[i].d*sin_perp;

//         //cout << "map: x=" << x << ", y=" << y << endl;

//         //mapLocs[i] = Vector2D(x,y);
//         mapLocs.push_back(Vector2D(x,y));
//     }

//     return mapLocs;
// }

