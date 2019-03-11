#include "PathPlanner.h"
#include <iostream>

using namespace std;

PathPlanner::PathPlanner(
    const DrivingParameters &drivingParams,
    const HighwayParameters &highwayParams,
    const vector<HighwayWaypoint> &waypoints)
{
    _highwayParams = highwayParams;
    _highwayCoords.reset(new HighwayCoordinates(highwayParams, waypoints));
    _behaviorPlanner.reset(new BehaviorPlanner(drivingParams, highwayParams));
}

vector<Vector2D> PathPlanner::PlanNextPath(
    const vector<Vector2D> &previousPath,
    const EgoVehicleState &egoVehicleState,
    const vector<OtherVehicleState> &otherVehicleStates)
{
    cout << "previous path" << endl;
    for (size_t i=0; i < previousPath.size(); i++) {
        cout << "  x: " << previousPath[i].x << ", y: " << previousPath[i].y << endl;
    }

    vector<Vector2D> nextPath;
    nextPath.reserve(100);

    if (_behaviorPlanner->IsChangingLanes())
    {
        nextPath.insert(nextPath.end(), previousPath.begin(), previousPath.end());
    }
    else
    {
        nextPath.insert(
            nextPath.end(), 
            previousPath.begin(), 
            previousPath.begin() + min<size_t>(previousPath.size(), 5));
    }
    
    if (nextPath.size() > 20) {
        return nextPath;
    }

    size_t N = nextPath.size();
    double t0 = N*0.02;

    Kinematics kin_ego;
    double d_ego;

    if (N < 3)
    {
        kin_ego = Kinematics(egoVehicleState.s, egoVehicleState.speed, 0.0);
        d_ego = egoVehicleState.d;
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

        Vector2D map0 = nextPath[N-1];
        Vector2D map1 = nextPath[N-2];
        Vector2D map2 = nextPath[N-3];
        // Vector2D map3 = nextPath[N-4];

        double yaw0 = atan2(map0.y - map1.y, map0.x - map1.x);
        // double yaw1 = atan2(map1.y - map2.y, map1.x - map2.x);
        // double yaw2 = atan2(map2.y - map3.y, map2.x - map3.x);

        Frenet2D hwy0 = _highwayCoords->MapToHighway(map0, yaw0);
        // Frenet2D hwy1 = _highwayCoords->MapToHighway(map1, yaw1);
        // Frenet2D hwy2 = _highwayCoords->MapToHighway(map2, yaw2);

        cout << "previous path end:" << endl;
        // cout << "  s: " << hwy2.s << ", d: " << hwy2.d << ", x: " << map2.x << ", y: " << map2.y << endl;
        // cout << "  s: " << hwy1.s << ", d: " << hwy1.d << ", x: " << map1.x << ", y: " << map1.y << endl;
        cout << "  x: " << map1.x << ", y: " << map1.y << endl;
        cout << "  x: " << map0.x << ", y: " << map0.y << endl;

        // double vel0 = (hwy0.s - hwy1.s) * 50;
        // double vel1 = (hwy1.s - hwy2.s) * 50;
        double vel0 = map0.DistanceTo(map1) * 50;
        double vel1 = map1.DistanceTo(map2) * 50;

        // kin_ego = Kinematics(hwy0.s, vel0, (vel0 - vel1) * 50);
        kin_ego = Kinematics(hwy0.s, vel0, vel0-vel1);
        d_ego = hwy0.d;
    }

    Vehicle egoVehicle(_highwayParams, kin_ego, d_ego, t0);
    
    vector<Vehicle> otherVehicles;
    otherVehicles.reserve(otherVehicleStates.size());

    for (size_t i=0; i < otherVehicleStates.size(); i++)
    {
        const OtherVehicleState &state = otherVehicleStates[i];
        Kinematics other_kin(state.s, sqrt(state.vx*state.vx + state.vy*state.vy), 0.0);
        Vehicle otherVehicle(_highwayParams, other_kin, state.d, 0.0);
        otherVehicles.push_back(otherVehicle.Predict(t0));
    }

    Vehicle egoNext = _behaviorPlanner->Update(egoVehicle, otherVehicles);
    if (!egoNext.IsValid())
        cout << "INVALID TRAJECTORY!!!" << endl;
    cout << "behavior: " << BehaviorPlanner::BehaviorLabel(_behaviorPlanner->CurrentBehavior()) << endl;

    Kinematics s_ini_kin = egoVehicle.GetKinematics();
    Kinematics s_fin_kin = egoNext.GetKinematics();
    cout << "initial: " << s_ini_kin.position << ", " << s_ini_kin.velocity << ", " << s_ini_kin.acceleration << endl;
    cout << "final: " << s_fin_kin.position << ", " << s_fin_kin.velocity << ", " << s_fin_kin.acceleration << endl;

    // cout << "planned initial/final s kinematics" << endl;
    // cout << "initial: " << s_ini_kin.position << " " << s_ini_kin.velocity << " " << s_ini_kin.acceleration << endl;
    // cout << "final: " << s_fin_kin.position << " " << s_fin_kin.velocity << " " << s_fin_kin.acceleration << endl;

    Kinematics d_ini_kin(egoVehicle.LateralPosition(), 0, 0);
    Kinematics d_fin_kin(egoNext.LateralPosition(), 0, 0);

    // cout << "planned initial/final d kinematics" << endl;
    // cout << "initial: " << d_ini_kin.position << " " << d_ini_kin.velocity << " " << d_ini_kin.acceleration << endl;
    // cout << "final: " << d_fin_kin.position << " " << d_fin_kin.velocity << " " << d_fin_kin.acceleration << endl;

    double dt = egoNext.Time() - egoVehicle.Time();
    //cout << "time step: " << dt << endl;

    // cout << "computing jerk minimizing s/d trajectories" << endl;
    array<double, 6> s_JMT = ComputeJerkMinimizingTrajectory(s_ini_kin, s_fin_kin, dt);
    array<double, 6> d_JMT = ComputeJerkMinimizingTrajectory(d_ini_kin, d_fin_kin, dt);

    // cout << "s polynomial: " << s_JMT[0] << " " << s_JMT[1] << " " << s_JMT[2] << " " << s_JMT[3] << " " << s_JMT[4] << " " << s_JMT[5] << endl;
    // cout << "d polynomial: " << d_JMT[0] << " " << d_JMT[1] << " " << d_JMT[2] << " " << d_JMT[3] << " " << d_JMT[4] << " " << d_JMT[5] << endl;

    size_t M = (size_t)round(dt / 0.02);
    vector<Frenet2D> pathHwy(M);

    for (size_t i=0; i < M; i++)
    {
        double t = (i+1)*0.02;
        double t2 = t*t;
        double t3 = t2*t;
        double t4 = t3*t;
        double t5 = t4*t;

        pathHwy[i].s = s_JMT[0] + s_JMT[1]*t + s_JMT[2]*t2 + s_JMT[3]*t3 + s_JMT[4]*t4 + s_JMT[5]*t5;
        pathHwy[i].d = d_JMT[0] + d_JMT[1]*t + d_JMT[2]*t2 + d_JMT[3]*t3 + d_JMT[4]*t4 + d_JMT[5]*t5;
    }

    // cout << "s/d path:" << endl;

    // for (size_t i=0; i < 100; i++){
    //     cout << "s=" << pathHwy[i].s << ", d=" << pathHwy[i].d << endl;
    // }

    // cout << "computing next path from trajectory" << endl;
    vector<Vector2D> pathMap = _highwayCoords->HighwayToMap(pathHwy);
    size_t K = _behaviorPlanner->IsChangingLanes() ? pathMap.size() : 30;
    nextPath.insert(nextPath.end(), pathMap.begin(), pathMap.begin() + K);

    cout << "next path" << endl;
    for (size_t i=0; i < nextPath.size(); i++) {
        cout << "  x: " << nextPath[i].x << ", y: " << nextPath[i].y << endl;
    }

    return nextPath;
}
