#include "PathPlanner.h"
#include <iostream>
#include "Bezier.h"

using namespace std;

PathPlanner::PathPlanner(
    const DrivingParameters &drivingParams,
    const HighwayParameters &highwayParams,
    const vector<HighwayWaypoint> &waypoints)
{
    _drivingParams = drivingParams;
    _highwayParams = highwayParams;
    _highwayCoords.reset(new HighwayCoordinates(highwayParams, waypoints));
    _behaviorPlanner.reset(new BehaviorPlanner(drivingParams, highwayParams));
}

// vector<Vector2D> PathPlanner::PlanNextPath(
//     const vector<Vector2D> &previousPath,
//     const EgoVehicleState &egoVehicleState,
//     const vector<OtherVehicleState> &otherVehicleStates)
// {
//     cout << "previous path" << endl;
//     for (size_t i=0; i < previousPath.size(); i++) {
//         cout << "  x: " << previousPath[i].x << ", y: " << previousPath[i].y << endl;
//     }

//     vector<Vector2D> nextPath;
//     nextPath.reserve(100);

//     if (_behaviorPlanner->IsChangingLanes())
//     {
//         nextPath.insert(nextPath.end(), previousPath.begin(), previousPath.end());
//     }
//     else
//     {
//         nextPath.insert(
//             nextPath.end(), 
//             previousPath.begin(), 
//             previousPath.begin() + min<size_t>(previousPath.size(), 5));
//     }
    
//     if (nextPath.size() > 20) {
//         return nextPath;
//     }

//     size_t N = nextPath.size();
//     double t0 = N*0.02;

//     Kinematics kin_ego;
//     double d_ego;

//     if (N < 3)
//     {
//         kin_ego = Kinematics(egoVehicleState.s, egoVehicleState.speed, 0.0);
//         d_ego = egoVehicleState.d;
//     }
//     // else if (N == 1)
//     // {
//     //     Vector2D map0(prev_path_x[N-1], prev_path_y[N-1]);
//     //     Vector2D map1(ego_state.x, ego_state.y);

//     //     double yaw0 = atan2(map0.y - map1.y, map0.x - map1.x);

//     //     Frenet2D hwy0 = _highwayCoords->MapToHighway(map0, yaw0);
//     //     Frenet2D hwy1 = { .s = ego_state.s, .d = ego_state.d };

//     //     double vel0 = (hwy0.s - hwy1.s) * 50;
//     //     double vel1 = ego_state.speed;

//     //     kin_ego = Kinematics(hwy0.s, vel0, (vel0 - vel1) * 50);
//     //     d_ego = hwy0.d;
//     // }
//     // else if (N == 2)
//     // {
//     //     Vector2D map0(prev_path_x[N-1], prev_path_y[N-1]);
//     //     Vector2D map1(prev_path_x[N-2], prev_path_y[N-2]);
//     //     Vector2D map2(ego_state.x, ego_state.y);

//     //     double yaw0 = atan2(map0.y - map1.y, map0.x - map1.x);
//     //     double yaw1 = atan2(map1.y - map2.y, map1.x - map2.x);

//     //     Frenet2D hwy0 = _highwayCoords->MapToHighway(map0, yaw0);
//     //     Frenet2D hwy1 = _highwayCoords->MapToHighway(map1, yaw1);
//     //     Frenet2D hwy2 = { .s = ego_state.s, .d = ego_state.d };

//     //     double vel0 = (hwy0.s - hwy1.s) * 50;
//     //     double vel1 = (hwy1.s - hwy2.s) * 50;

//     //     kin_ego = Kinematics(hwy0.s, vel0, (vel0 - vel1) * 50);
//     //     d_ego = hwy0.d;
//     // }
//     // else if (N == 3)
//     // {
//     //     Vector2D map0(prev_path_x[N-1], prev_path_y[N-1]);
//     //     Vector2D map1(prev_path_x[N-2], prev_path_y[N-2]);
//     //     Vector2D map2(prev_path_x[N-3], prev_path_y[N-3]);
//     //     Vector2D map3(ego_state.x, ego_state.y);

//     //     double yaw0 = atan2(map0.y - map1.y, map0.x - map1.x);
//     //     double yaw1 = atan2(map1.y - map2.y, map1.x - map2.x);
//     //     double yaw2 = atan2(map2.y - map3.y, map2.x - map3.x);

//     //     Frenet2D hwy0 = _highwayCoords->MapToHighway(map0, yaw0);
//     //     Frenet2D hwy1 = _highwayCoords->MapToHighway(map1, yaw1);
//     //     Frenet2D hwy2 = _highwayCoords->MapToHighway(map2, yaw2);

//     //     double vel0 = (hwy0.s - hwy1.s) * 50;
//     //     double vel1 = (hwy1.s - hwy2.s) * 50;

//     //     kin_ego = Kinematics(hwy0.s, vel0, (vel0 - vel1) * 50);
//     //     d_ego = hwy0.d;
//     // }
//     else
//     {
//         cout << "N = " << N << endl;

//         Vector2D map0 = nextPath[N-1];
//         Vector2D map1 = nextPath[N-2];
//         Vector2D map2 = nextPath[N-3];
//         // Vector2D map3 = nextPath[N-4];

//         double yaw0 = atan2(map0.y - map1.y, map0.x - map1.x);
//         // double yaw1 = atan2(map1.y - map2.y, map1.x - map2.x);
//         // double yaw2 = atan2(map2.y - map3.y, map2.x - map3.x);

//         Frenet2D hwy0 = _highwayCoords->MapToHighway(map0, yaw0);
//         // Frenet2D hwy1 = _highwayCoords->MapToHighway(map1, yaw1);
//         // Frenet2D hwy2 = _highwayCoords->MapToHighway(map2, yaw2);

//         cout << "previous path end:" << endl;
//         // cout << "  s: " << hwy2.s << ", d: " << hwy2.d << ", x: " << map2.x << ", y: " << map2.y << endl;
//         // cout << "  s: " << hwy1.s << ", d: " << hwy1.d << ", x: " << map1.x << ", y: " << map1.y << endl;
//         cout << "  x: " << map1.x << ", y: " << map1.y << endl;
//         cout << "  x: " << map0.x << ", y: " << map0.y << endl;

//         // double vel0 = (hwy0.s - hwy1.s) * 50;
//         // double vel1 = (hwy1.s - hwy2.s) * 50;
//         double vel0 = map0.DistanceTo(map1) * 50;
//         double vel1 = map1.DistanceTo(map2) * 50;

//         // kin_ego = Kinematics(hwy0.s, vel0, (vel0 - vel1) * 50);
//         kin_ego = Kinematics(hwy0.s, vel0, vel0-vel1);
//         d_ego = hwy0.d;
//     }

//     Vehicle egoVehicle(_highwayParams, kin_ego, d_ego, t0);
    
//     vector<Vehicle> otherVehicles;
//     otherVehicles.reserve(otherVehicleStates.size());

//     for (size_t i=0; i < otherVehicleStates.size(); i++)
//     {
//         const OtherVehicleState &state = otherVehicleStates[i];
//         Kinematics other_kin(state.s, sqrt(state.vx*state.vx + state.vy*state.vy), 0.0);
//         Vehicle otherVehicle(_highwayParams, other_kin, state.d, 0.0);
//         otherVehicles.push_back(otherVehicle.Predict(t0));
//     }

//     Vehicle egoNext = _behaviorPlanner->Update(egoVehicle, otherVehicles);
//     if (!egoNext.IsValid())
//         cout << "INVALID TRAJECTORY!!!" << endl;
//     cout << "behavior: " << BehaviorPlanner::BehaviorLabel(_behaviorPlanner->CurrentBehavior()) << endl;

//     Kinematics s_ini_kin = egoVehicle.GetKinematics();
//     Kinematics s_fin_kin = egoNext.GetKinematics();
//     cout << "initial: " << s_ini_kin.position << ", " << s_ini_kin.velocity << ", " << s_ini_kin.acceleration << endl;
//     cout << "final: " << s_fin_kin.position << ", " << s_fin_kin.velocity << ", " << s_fin_kin.acceleration << endl;

//     // cout << "planned initial/final s kinematics" << endl;
//     // cout << "initial: " << s_ini_kin.position << " " << s_ini_kin.velocity << " " << s_ini_kin.acceleration << endl;
//     // cout << "final: " << s_fin_kin.position << " " << s_fin_kin.velocity << " " << s_fin_kin.acceleration << endl;

//     Kinematics d_ini_kin(egoVehicle.LateralPosition(), 0, 0);
//     Kinematics d_fin_kin(egoNext.LateralPosition(), 0, 0);

//     // cout << "planned initial/final d kinematics" << endl;
//     // cout << "initial: " << d_ini_kin.position << " " << d_ini_kin.velocity << " " << d_ini_kin.acceleration << endl;
//     // cout << "final: " << d_fin_kin.position << " " << d_fin_kin.velocity << " " << d_fin_kin.acceleration << endl;

//     double dt = egoNext.Time() - egoVehicle.Time();
//     //cout << "time step: " << dt << endl;

//     // cout << "computing jerk minimizing s/d trajectories" << endl;
//     array<double, 6> s_JMT = ComputeJerkMinimizingTrajectory(s_ini_kin, s_fin_kin, dt);
//     array<double, 6> d_JMT = ComputeJerkMinimizingTrajectory(d_ini_kin, d_fin_kin, dt);

//     // cout << "s polynomial: " << s_JMT[0] << " " << s_JMT[1] << " " << s_JMT[2] << " " << s_JMT[3] << " " << s_JMT[4] << " " << s_JMT[5] << endl;
//     // cout << "d polynomial: " << d_JMT[0] << " " << d_JMT[1] << " " << d_JMT[2] << " " << d_JMT[3] << " " << d_JMT[4] << " " << d_JMT[5] << endl;

//     size_t M = (size_t)round(dt / 0.02);
//     vector<Frenet2D> pathHwy(M);

//     for (size_t i=0; i < M; i++)
//     {
//         double t = (i+1)*0.02;
//         double t2 = t*t;
//         double t3 = t2*t;
//         double t4 = t3*t;
//         double t5 = t4*t;

//         pathHwy[i].s = s_JMT[0] + s_JMT[1]*t + s_JMT[2]*t2 + s_JMT[3]*t3 + s_JMT[4]*t4 + s_JMT[5]*t5;
//         pathHwy[i].d = d_JMT[0] + d_JMT[1]*t + d_JMT[2]*t2 + d_JMT[3]*t3 + d_JMT[4]*t4 + d_JMT[5]*t5;
//     }

//     // cout << "s/d path:" << endl;

//     // for (size_t i=0; i < 100; i++){
//     //     cout << "s=" << pathHwy[i].s << ", d=" << pathHwy[i].d << endl;
//     // }

//     // cout << "computing next path from trajectory" << endl;
//     vector<Vector2D> pathMap = _highwayCoords->HighwayToMap(pathHwy);
//     size_t K = _behaviorPlanner->IsChangingLanes() ? pathMap.size() : 30;
//     nextPath.insert(nextPath.end(), pathMap.begin(), pathMap.begin() + K);

//     cout << "next path" << endl;
//     for (size_t i=0; i < nextPath.size(); i++) {
//         cout << "  x: " << nextPath[i].x << ", y: " << nextPath[i].y << endl;
//     }

//     return nextPath;
// }

vector<Vector2D> PathPlanner::PlanNextPath(
    const vector<Vector2D> &previousPath,
    const EgoVehicleState &ego_state,
    const vector<OtherVehicleState> &other_states)
{
    // cout << "previous path" << endl;
    // for (size_t i=0; i < previousPath.size(); i++) {
    //     cout << "  x: " << previousPath[i].x << ", y: " << previousPath[i].y << endl;
    // }

    vector<Vector2D> nextPath;
    //nextPath.reserve(100);

    // if (_behaviorPlanner->IsChangingLanes())
    // {
    //     nextPath.insert(nextPath.end(), previousPath.begin(), previousPath.end());
    // }
    // else
    // {
    //     nextPath.insert(
    //         nextPath.end(), 
    //         previousPath.begin(), 
    //         previousPath.begin() + min<size_t>(previousPath.size(), 50));
    // }
    
    nextPath.insert(nextPath.end(), previousPath.begin(), previousPath.end());

    if (nextPath.size() >= 35) {
        return nextPath;
    }

    size_t N = nextPath.size();
    double t0 = N*0.02;

    // Kinematics kin_ego;
    // double d_ego;
    // Pose2D ego_pose_start, ego_pose_next;
    //double ego_speed_start, ego_speed_next;

    // double x_ini, vx_ini, y_ini, vy_ini;
    // double s_ini, vs_ini, d_ini;
    Kinematics x_kin_ini, y_kin_ini;
    Kinematics s_kin_ini, s_kin_fin;
    double d_ini;

    if (N == 0)
    {
        Vector2D ego_dir = Vector2D::Polar(ego_state.yaw);
        // x_ini = ego_state.x;
        // y_ini = ego_state.y;
        // vx_ini = ego_state.speed * ego_dir.x;
        // vy_ini = ego_state.speed * ego_dir.y;
        x_kin_ini = Kinematics(ego_state.x, ego_state.speed * ego_dir.x, 0.0);
        y_kin_ini = Kinematics(ego_state.y, ego_state.speed * ego_dir.y, 0.0);
        s_kin_ini = Kinematics(ego_state.s, ego_state.speed, 0.0);
        // s_ini = ego_state.s;
        // vs_ini = ego_state.speed;
        d_ini = ego_state.d;

        // ego_speed_start = egoVehicleState.speed;
    }
    else if (N < 4)
    {
        cout << "Error: previous path too short!" << endl;
        throw "error";
    }
    // else if (N == 1)
    // {
    //     Vector2D map0(prev_path_x[N-1], prev_path_y[N-1]);
    //     Vector2D map1(ego_state.x, ego_state.y);

    //     double yaw0 = atan2(map0.y - map1.y, map0.x - map1.x);

    //     Frenet2D hwy0 = _highwayCoords->MapToHighway(map0, yaw0);
    //     Frenet2D hwy1 = { .s = ego_state.s, .d = ego_state.d };

    //     double vel0 = (hwy0.s - hwy1.s) * 50;
    //     double vel1 = ego_state.speed;

    //     kin_ego = Kinematics(hwy0.s, vel0, (vel0 - vel1) * 50);
    //     d_ego = hwy0.d;
    // }
    // else if (N == 2)
    // {
    //     Vector2D map0(prev_path_x[N-1], prev_path_y[N-1]);
    //     Vector2D map1(prev_path_x[N-2], prev_path_y[N-2]);
    //     Vector2D map2(ego_state.x, ego_state.y);

    //     double yaw0 = atan2(map0.y - map1.y, map0.x - map1.x);
    //     double yaw1 = atan2(map1.y - map2.y, map1.x - map2.x);

    //     Frenet2D hwy0 = _highwayCoords->MapToHighway(map0, yaw0);
    //     Frenet2D hwy1 = _highwayCoords->MapToHighway(map1, yaw1);
    //     Frenet2D hwy2 = { .s = ego_state.s, .d = ego_state.d };

    //     double vel0 = (hwy0.s - hwy1.s) * 50;
    //     double vel1 = (hwy1.s - hwy2.s) * 50;

    //     kin_ego = Kinematics(hwy0.s, vel0, (vel0 - vel1) * 50);
    //     d_ego = hwy0.d;
    // }
    // else if (N == 3)
    // {
    //     Vector2D map0(prev_path_x[N-1], prev_path_y[N-1]);
    //     Vector2D map1(prev_path_x[N-2], prev_path_y[N-2]);
    //     Vector2D map2(prev_path_x[N-3], prev_path_y[N-3]);
    //     Vector2D map3(ego_state.x, ego_state.y);

    //     double yaw0 = atan2(map0.y - map1.y, map0.x - map1.x);
    //     double yaw1 = atan2(map1.y - map2.y, map1.x - map2.x);
    //     double yaw2 = atan2(map2.y - map3.y, map2.x - map3.x);

    //     Frenet2D hwy0 = _highwayCoords->MapToHighway(map0, yaw0);
    //     Frenet2D hwy1 = _highwayCoords->MapToHighway(map1, yaw1);
    //     Frenet2D hwy2 = _highwayCoords->MapToHighway(map2, yaw2);

    //     double vel0 = (hwy0.s - hwy1.s) * 50;
    //     double vel1 = (hwy1.s - hwy2.s) * 50;

    //     kin_ego = Kinematics(hwy0.s, vel0, (vel0 - vel1) * 50);
    //     d_ego = hwy0.d;
    // }
    else
    {
        cout << "N = " << N << endl;

        Vector2D p0 = nextPath[N-1];
        Vector2D p1 = nextPath[N-2];
        Vector2D p2 = nextPath[N-3];
        Vector2D p3 = nextPath[N-4];
        Vector2D vel0 = 50*(p0-p1);
        Vector2D vel1 = 50*(p1-p2);
        Vector2D vel2 = 50*(p2-p3);
        Vector2D acc0 = 50*(vel0-vel1);
        Vector2D acc1 = 50*(vel1-vel2);
        // Vector2D map3 = nextPath[N-4];

        x_kin_ini = Kinematics(p0.x, vel0.x, acc0.x);
        y_kin_ini = Kinematics(p0.y, vel0.y, acc0.y);

        Pose2D pose0 = { .location = p0, .direction = vel0.Unit() };
        Pose2D pose1 = { .location = p1, .direction = vel1.Unit() };
        Pose2D pose2 = { .location = p2, .direction = vel2.Unit() };

        Frenet2D hwy0 = _highwayCoords->MapToHighway(pose0);
        Frenet2D hwy1 = _highwayCoords->MapToHighway(pose1);
        Frenet2D hwy2 = _highwayCoords->MapToHighway(pose2);

        //ego_pose_start = pose0;

        // cout << "previous path end:" << endl;
        // // cout << "  s: " << hwy2.s << ", d: " << hwy2.d << ", x: " << map2.x << ", y: " << map2.y << endl;
        // // cout << "  s: " << hwy1.s << ", d: " << hwy1.d << ", x: " << map1.x << ", y: " << map1.y << endl;
        // cout << "  x: " << p1.x << ", y: " << p1.y << endl;
        // cout << "  x: " << p0.x << ", y: " << p0.y << endl;

        // double vel0 = (hwy0.s - hwy1.s) * 50;
        // double vel1 = (hwy1.s - hwy2.s) * 50;
        // double vel0 = map0.DistanceTo(map1) * 50;
        // double vel1 = map1.DistanceTo(map2) * 50;
        double s0 = hwy0.s;
        double s1 = hwy1.s;
        double s2 = hwy2.s;
        double vs0 = 50*(s0-s1);
        double vs1 = 50*(s1-s2);
        double as0 = 50*(vs0-vs1);

        // kin_ego = Kinematics(hwy0.s, vel0, (vel0 - vel1) * 50);
        s_kin_ini = Kinematics(s0, vs0, as0);
        d_ini = hwy0.d;

        //ego_speed_start = vel0;
    }
    // else
    // {
    //     cout << "N = " << N << endl;

    //     Vector2D p0 = nextPath[N-1];
    //     Vector2D p1 = nextPath[N-2];
    //     Vector2D p2 = nextPath[N-3];
    //     // Vector2D p3 = nextPath[N-4];
    //     Vector2D vel0 = 50*(p0-p1);
    //     Vector2D vel1 = 50*(p1-p2);
    //     // Vector2D vel2 = 50*(p2-p3);
    //     // Vector2D acc0 = 50*(vel0-vel1);
    //     // Vector2D acc1 = 50*(vel1-vel2);
    //     // Vector2D map3 = nextPath[N-4];


    //     x_ini = p0.x;
    //     y_ini = p0.y;
    //     vx_ini = vel0.x;
    //     vy_ini = vel0.y;

    //     // x_kin_ini = Kinematics(p0.x, vel0.x, acc0.x);
    //     // y_kin_ini = Kinematics(p0.y, vel0.y, acc0.y);

    //     Pose2D pose0 = { .location = p0, .direction = vel0.Unit() };
    //     Pose2D pose1 = { .location = p1, .direction = vel1.Unit() };
    //     // Pose2D pose2 = { .location = p2, .direction = vel2.Unit() };

    //     Frenet2D hwy0 = _highwayCoords->MapToHighway(pose0);
    //     Frenet2D hwy1 = _highwayCoords->MapToHighway(pose1);
    //     // Frenet2D hwy2 = _highwayCoords->MapToHighway(pose2);

    //     //ego_pose_start = pose0;

    //     // cout << "previous path end:" << endl;
    //     // // cout << "  s: " << hwy2.s << ", d: " << hwy2.d << ", x: " << map2.x << ", y: " << map2.y << endl;
    //     // // cout << "  s: " << hwy1.s << ", d: " << hwy1.d << ", x: " << map1.x << ", y: " << map1.y << endl;
    //     // cout << "  x: " << p1.x << ", y: " << p1.y << endl;
    //     // cout << "  x: " << p0.x << ", y: " << p0.y << endl;

    //     // double vel0 = (hwy0.s - hwy1.s) * 50;
    //     // double vel1 = (hwy1.s - hwy2.s) * 50;
    //     // double vel0 = map0.DistanceTo(map1) * 50;
    //     // double vel1 = map1.DistanceTo(map2) * 50;
    //     double s0 = hwy0.s;
    //     double s1 = hwy1.s;
    //     // double s2 = hwy2.s;
    //     double vs0 = 50*(s0-s1);
    //     // double vs1 = 50*(s1-s2);
    //     // double as0 = 50*(vs0-vs1);

    //     // kin_ego = Kinematics(hwy0.s, vel0, (vel0 - vel1) * 50);
    //     // s_kin_ini = Kinematics(s0, vs0, as0);
    //     s_ini = s0;
    //     vs_ini = vs0;
    //     d_ini = hwy0.d;

    //     //ego_speed_start = vel0;
    // }

    cout << "start x: " << x_kin_ini.position << ", vx: " << x_kin_ini.velocity << ", ax: " << x_kin_ini.acceleration << endl;
    cout << "start y: " << y_kin_ini.position << ", vy: " << y_kin_ini.velocity << ", ay: " << y_kin_ini.acceleration << endl;
    cout << "start s: " << s_kin_ini.position << ", vs: " << s_kin_ini.velocity << ", as: " << s_kin_ini.acceleration << endl;
    cout << "start d: " << d_ini << endl;

    //Vehicle ego_vehicle(_highwayParams, s_kin_ini, d_ini, t0);
    Vehicle ego_vehicle(s_kin_ini.position, s_kin_ini.velocity, d_ini);
    
    vector<Vehicle> other_vehicles;
    other_vehicles.reserve(other_states.size());

    for (size_t i=0; i < other_states.size(); i++)
    {
        const OtherVehicleState &state = other_states[i];
        // Kinematics other_kin(state.s, sqrt(state.vx*state.vx + state.vy*state.vy), 0.0);
        // Vehicle other(_highwayParams, other_kin, state.d, 0.0);
        Vehicle other(
            state.s,
            sqrt(state.vx*state.vx + state.vy*state.vy),
            state.d);

        other_vehicles.push_back(other.Predict(t0));
    }

    // Vehicle ego_next = _behaviorPlanner->Update(ego_vehicle, other_vehicles);

    // if (!ego_next.IsValid())
    // {
    //     cout << "INVALID TRAJECTORY!!!" << endl;
    //     return nextPath;
    // }

    BehaviorPlanner::Trajectory traj = _behaviorPlanner->Update(ego_vehicle, other_vehicles);

    if (!traj.is_valid)
    {
        cout << "INVALID TRAJECTORY!!!" << endl;
        return nextPath;
    }

    string behavior_label = BehaviorPlanner::BehaviorLabel(_behaviorPlanner->CurrentBehavior());
    //double dt = ego_next.Time() - ego_vehicle.Time();
    // cout << "behavior: " << behavior_label << ", duration: " << dt << endl;
    cout << "behavior: " << behavior_label << endl;

    double vs_fin = traj.goal_velocity;
    double d_fin = traj.goal_lateral_position;
    double dur = 1.0;
    double dt = 0.02;

    vector<Vector2D> path;
    path.reserve(100);

    for (;;)
    {
        double s_fin = s_kin_ini.position + 0.5*(s_kin_ini.velocity + vs_fin)*dur;
        Frenet2D hwy_fin = { .s = s_fin, .d = d_fin};
        Pose2D pose_fin = _highwayCoords->HighwayToMap(hwy_fin);

        Kinematics x_kin_fin(pose_fin.location.x, vs_fin * pose_fin.direction.x, 0.0);
        Kinematics y_kin_fin(pose_fin.location.y, vs_fin * pose_fin.direction.y, 0.0);

        array<double, 6> x_JMT = ComputeJerkMinimizingTrajectory(x_kin_ini, x_kin_fin, dur);
        array<double, 6> y_JMT = ComputeJerkMinimizingTrajectory(y_kin_ini, y_kin_fin, dur);

        // cout << "x polynomial: " << x_JMT[0] << " " << x_JMT[1] << " " << x_JMT[2] << " " << x_JMT[3] << " " << x_JMT[4] << " " << x_JMT[5] << endl;
        // cout << "y polynomial: " << y_JMT[0] << " " << y_JMT[1] << " " << y_JMT[2] << " " << y_JMT[3] << " " << y_JMT[4] << " " << y_JMT[5] << endl;

        Vector2D p0(x_kin_ini.position, y_kin_ini.position);
        Vector2D v0(x_kin_ini.velocity, y_kin_ini.velocity);
        size_t M = (size_t)round(dur / 0.02);
        bool violation = false;

        for (size_t i=0; i < M; i++)
        {
            double t = (i+1)*dt;
            double t2 = t*t;
            double t3 = t2*t;
            double t4 = t3*t;
            double t5 = t4*t;

            Vector2D p(
                x_JMT[0] + x_JMT[1]*t + x_JMT[2]*t2 + x_JMT[3]*t3 + x_JMT[4]*t4 + x_JMT[5]*t5,
                y_JMT[0] + y_JMT[1]*t + y_JMT[2]*t2 + y_JMT[3]*t3 + y_JMT[4]*t4 + y_JMT[5]*t5);
            
            Vector2D v = (1/dt)*(p - p0);
            Vector2D a = (1/dt)*(v - v0);

            // if (v.Length() > _highwayParams.SpeedLimit)
            // {
            //     violation = true;
            //     break;
            // }

            if (a.Length() > _drivingParams.AccelerationLimit)
            {
                violation = true;
                break;
            }

            path.push_back(p);
            v0 = v;
            p0 = p;
        }

        if (!violation)
        {
            cout << "path duration: " << dur << endl;
            cout << "final s: " << s_fin << ", vs: " << vs_fin << ", as: " << 0.0 << endl;
            cout << "final d: " << d_fin << endl;
            cout << "final x: " << x_kin_fin.position << ", vx: " << x_kin_fin.velocity << ", ax: " << x_kin_fin.acceleration << endl;
            cout << "final y: " << y_kin_fin.position << ", vy: " << y_kin_fin.velocity << ", ay: " << y_kin_fin.acceleration << endl;
            break;
        }

        dur += 0.25;
        path.clear();
    }

    // double A = (vs_fin - vs_ini) / M_PI;

    // if (fabs(A) >= _drivingParams.AccelerationLimit)
    // {
    //     A = _drivingParams.AccelerationLimit * sign(A);
    //     vs_fin = A*M_PI + vs_ini;
    // }

    // double dur = 1.5;
    // double s_fin = s_ini + 0.5*(vs_ini + vs_fin)*dur;
    // double K = 0.5*(vs_fin - vs_ini)/(A*M_PI);

    // s_kin_fin = ego_next.GetKinematics();
    // d_fin = ego_next.LateralPosition();

    // Frenet2D hwy_fin = { .s = s_kin_fin.position, .d = d_fin };
    // Pose2D pose_fin = _highwayCoords->HighwayToMap(hwy_fin);

    // x_kin_fin = Kinematics(pose_fin.location.x, s_kin_fin.velocity * pose_fin.direction.x, s_kin_fin.acceleration * pose_fin.direction.x);
    // y_kin_fin = Kinematics(pose_fin.location.y, s_kin_fin.velocity * pose_fin.direction.y, s_kin_fin.acceleration * pose_fin.direction.y);

    // Frenet2D hwy_fin = { .s = s_fin, .d = d_fin};
    // Pose2D pose_fin = _highwayCoords->HighwayToMap(hwy_fin);

    // cout << "final x: " << x_kin_fin.position << ", vx: " << x_kin_fin.velocity << ", ax: " << x_kin_fin.acceleration << endl;
    // cout << "final y: " << y_kin_fin.position << ", vy: " << y_kin_fin.velocity << ", ay: " << y_kin_fin.acceleration << endl;
    // cout << "final s: " << s_fin << ", vs: " << vs_fin << ", d: " << d_fin << endl;

    // Frenet2D hwy_next = {
    //     .s = egoNext.Position(),
    //     .d = egoNext.LateralPosition()
    // };

    // ego_speed_next = egoNext.Velocity();
    // ego_pose_next = _highwayCoords->HighwayToMap(hwy_next);

    // cout << "next x: " << ego_pose_next.location.x << ", y: " << ego_pose_next.location.y << endl;

    // double dt = egoNext.Time() - egoVehicle.Time();
    // cout << "time step: " << dt << endl;
    // cout << "avg vel: " << (ego_pose_next.location - ego_pose_start.location).Length() / dt << endl;
    // cout << "start vel: " << ego_speed_start << endl;
    // cout << "next vel: " << ego_speed_next << endl;

    // BezierCurve traj = BezierCurve::FromKeyframes(ego_pose_start, ego_pose_next);

    // // size_t M = (size_t)round(dt / 0.02);
    // // vector<Vector2D> path(M);

    // // for (size_t i=0; i < M; i++)
    // // {
    // //     path[i] = traj.Evaluate(double(i+1) / M);
    // // }

    // vector<Vector2D> path = traj.Render(
    //     ego_speed_start/50, 
    //     ego_speed_next/50, 
    //     _drivingParams.AccelerationLimit/50/50/2);

    // cout << "rendered " << path.size() << " steps" << endl;
    // double vel_final = (path[path.size()-1] - path[path.size()-2]).Length() * 50;
    // cout << "final vel: " << vel_final << endl;

    // cout << "planned initial/final d kinematics" << endl;
    // cout << "initial: " << d_ini_kin.position << " " << d_ini_kin.velocity << " " << d_ini_kin.acceleration << endl;
    // cout << "final: " << d_fin_kin.position << " " << d_fin_kin.velocity << " " << d_fin_kin.acceleration << endl;


    // // cout << "computing jerk minimizing s/d trajectories" << endl;
    // array<double, 6> x_JMT = ComputeJerkMinimizingTrajectory(x_kin_ini, x_kin_fin, dt);
    // array<double, 6> y_JMT = ComputeJerkMinimizingTrajectory(y_kin_ini, y_kin_fin, dt);

    // cout << "x polynomial: " << x_JMT[0] << " " << x_JMT[1] << " " << x_JMT[2] << " " << x_JMT[3] << " " << x_JMT[4] << " " << x_JMT[5] << endl;
    // cout << "y polynomial: " << y_JMT[0] << " " << y_JMT[1] << " " << y_JMT[2] << " " << y_JMT[3] << " " << y_JMT[4] << " " << y_JMT[5] << endl;

    // size_t M = (size_t)round(dt / 0.02);
    // vector<Vector2D> path(M);

    // for (size_t i=0; i < M; i++)
    // {
    //     double t = (i+1)*0.02;
    //     double t2 = t*t;
    //     double t3 = t2*t;
    //     double t4 = t3*t;
    //     double t5 = t4*t;

    //     path[i].x = x_JMT[0] + x_JMT[1]*t + x_JMT[2]*t2 + x_JMT[3]*t3 + x_JMT[4]*t4 + x_JMT[5]*t5;
    //     path[i].y = y_JMT[0] + y_JMT[1]*t + y_JMT[2]*t2 + y_JMT[3]*t3 + y_JMT[4]*t4 + y_JMT[5]*t5;
    // }

    // double dt = 0.02;
    // size_t M = (size_t)round(dur / dt);
    // vector<Vector2D> path(M);

    // double x0 = x_ini;
    // double y0 = y_ini;
    // double vx0 = vx_ini;
    // double vy0 = vy_ini;
    // double ax0 = 0;
    // double ay0 = 0;

    // for (size_t i=0; i < M; i++)
    // {
    //     double t = 2*M_PI*(i+1)/M;
    //     double a = K*(1 - cos(t));
    //     double x = x0 + vx0*dt + 0.5*a*dt*dt;
    //     double y = y0 + vy0*dt + 0.5*a*dt*dt;
        
    //     path[i].x = x;
    //     path[i].y = y;

    //     vx0 = 50*(x - x0);
    //     vy0 = 50*(y - y0);
    //     x0 = x;
    //     y0 = y;
    // }

    // cout << "s/d path:" << endl;

    // for (size_t i=0; i < 100; i++){
    //     cout << "s=" << pathHwy[i].s << ", d=" << pathHwy[i].d << endl;
    // }

    // vector<Pose2D> path = _highwayCoords->HighwayToMap(pathHwy);

    // cout << "computing next path from trajectory" << endl;
    // vector<Vector2D> pathMap = _highwayCoords->HighwayToMap(pathHwy);
    //size_t K = _behaviorPlanner->IsChangingLanes() ? path.size() : 50;
    nextPath.insert(nextPath.end(), path.begin(), path.end());

    // cout << "next path" << endl;
    // for (size_t i=0; i < nextPath.size(); i++) {
    //     cout << "  x: " << nextPath[i].x << ", y: " << nextPath[i].y << endl;
    // }

    return nextPath;
}
