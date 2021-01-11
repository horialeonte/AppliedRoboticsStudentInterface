// ********** Strategy for the mission **********
// All obstacles, victims, and goals, are considered obstacles; except the current goal.
// The current goal might be one of the numbered victims or the gate.
// The rescued victims will be permanently removed from the obstacles list.
// Once the current goal is reached, the next numbered victim or the gate is set as the new goal.
// Keep going until the gate is reached, when the mission is considered successful.

#include "mission.h"

// See: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperOffset/_Body.htm
std::vector<Polygon> inflate(std::vector<Polygon> obs, float R){
    // The Clipper library only works with int, therefore our floats are scaled to int trying not to lose much precision.
    float scale_int = 10000;
    int R_int = int ( R * scale_int );
    ClipperLib::Paths solution;
    ClipperLib::ClipperOffset co;
    for ( unsigned int i = 0; i < obs.size(); i++ ){
        ClipperLib::Path temp_path;
        for ( unsigned int j = 0; j < obs[i].size(); j++ ){
            temp_path << ClipperLib::IntPoint( int( obs[i][j].x * scale_int ), int( obs[i][j].y * scale_int) );
        }
        // NOTE: jtSquare is selected in order to have simple polygons which are fast to compute
        // Other slower, more precise solutions are jtRound, jtMiter
        // See: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Types/JoinType.htm
        co.AddPath(temp_path, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
    }
    co.Execute(solution, R_int);

    // Scale back to float and return
    std::vector<Polygon> obs_infl;
    for ( unsigned i = 0; i < solution.size(); i++ ){
        Polygon temp_pol;
        for ( unsigned j = 0; j < solution[i].size(); j++ ){
            temp_pol.push_back(Point{solution[i][j].X/scale_int, solution[i][j].Y/scale_int});
        }
        obs_infl.push_back(temp_pol);
    }
    return obs_infl;
}

// Compute the centroid of a polygon. See: https://bell0bytes.eu/centroid-convex/
Point centroid(const Polygon pol){
	float centroidX = 0, centroidY = 0;
	float det = 0, tempDet = 0;
	unsigned int j = 0;
	unsigned int nVertices = (unsigned int)pol.size();

	for (unsigned int i = 0; i < nVertices; i++){

		// closed polygon - last vertex connects with the first one
		if ( i + 1 == nVertices ){ j = 0; }
		else { j = i + 1; }

		// compute the determinant
		tempDet = pol[i].x * pol[j].y - pol[j].x*pol[i].y;
		det += tempDet;

		centroidX += (pol[i].x + pol[j].x)*tempDet;
		centroidY += (pol[i].y + pol[j].y)*tempDet;
	}

	// divide by the total mass of the polygon
	centroidX /= 3*det;
	centroidY /= 3*det;

    return Point{centroidX, centroidY};
}

Path mission1(const struct RRTS_params RRTS_params, const struct Dubins_params Dubins_params, const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta){

    // Variables to measure computation time
    struct timeval tv0, tvf;
    unsigned long long t0;
    unsigned long long tf;

    // Define radius of the circle containing the robot, to later inflate the obstacles.
    const float R = 0.11;

    // ********** HIGH-LEVEL PLANNING ********** //
    // **********        RRT*         ********** //

    // Measure execution time (start)
    gettimeofday(&tv0, NULL);

    // Initialize starting point of the robot and global plan
    Point p_start{x, y};
    Plan plan_global;

    // The number of goals is the number of victims plus the gate
    const int numGoals = victim_list.size() + 1;

    for ( unsigned int currGoal = 1; currGoal <= numGoals; currGoal++ ){

        std::vector<Polygon> mission_obs = obstacle_list;
        Point current_goal;

        // As long as the next goal is not the gate, add the gate as obstacle.
        if ( currGoal < numGoals ){
            mission_obs.push_back(gate);
            // Choose current goal as the next victim ordered by number. Add the remaining victims as obstacles.
            for ( size_t i = 0; i < victim_list.size(); i++ ){
                if ( victim_list[i].first == currGoal ){ current_goal = centroid(victim_list[i].second); }
                else if ( victim_list[i].first > currGoal ) { mission_obs.push_back( victim_list[i].second ); }
            }
        // If last goal, then goal is the gate and no further victims are to be added as obstacles.
        } else if ( currGoal == numGoals ){
            current_goal = centroid(gate);
        }

        // Inflate obstacles
        mission_obs = inflate(mission_obs, R);

        // Computations: 1) Compute RRT* graph, 2) Create plan from graph.
        Graph graph = RRTstar(p_start, current_goal, borders, mission_obs, RRTS_params.maxIt, RRTS_params.tol, RRTS_params.d_lim, RRTS_params.b);
        Plan plan_local = fetchPlan(graph, graph[0], graph[graph.size()-1]);

        std::cout << "Size graph: " << graph.size() << ". Size plan: " << plan_local.size() << std::endl;

        // Update new starting point as old last point.
        // Avoid overlapping between starting and goal points by removing the last point, except for the final local plan.
        if ( currGoal < numGoals ){
            p_start = plan_local.back();
            plan_local.pop_back();
        }

        // Add the local plan to the global plan.
        for ( unsigned int i = 0; i < plan_local.size(); i++ ){
            plan_global.push_back(plan_local[i]);
        }
    }

    // Measure execution time (end)
    gettimeofday(&tvf, NULL);
    t0 = (unsigned long long)(tv0.tv_sec) * 1000 + (unsigned long long)(tv0.tv_usec) / 1000;
    tf = (unsigned long long)(tvf.tv_sec) * 1000 + (unsigned long long)(tvf.tv_usec) / 1000;
    std::cout << "RRT* elapsed time: " << tf-t0 << " ms." << std::endl;

    // Write the resulting plan in a csv file.
    write_plan(plan_global);

    // ********** LOW-LEVEL PLANNING ********** //
    // **********   Dubins curves    ********** //

    std::vector<Pose> all_confs;
    Path opt_path;

    // Prepare RRT* points to be the baseline for Dubins paths
    all_confs.push_back(Pose{0.0, plan_global[0].x, plan_global[0].y, theta, 0.0});
    for ( unsigned int i = 1; i < plan_global.size(); i++ ){
        all_confs.push_back(Pose{0.0, plan_global[i].x, plan_global[i].y, float(atan2( plan_global[i-1].x - plan_global[i].x, plan_global[i-1].y - plan_global[i].y)), 0.0});
    }

    // Perform Iterative Dynamic Programming
    gettimeofday(&tv0, NULL);
    opt_path = refinementMarkovDubins(all_confs, Dubins_params.Kmax, Dubins_params.k, Dubins_params.M);
    gettimeofday(&tvf, NULL);
    t0 = (unsigned long long)(tv0.tv_sec) * 1000 + (unsigned long long)(tv0.tv_usec) / 1000;
    tf = (unsigned long long)(tvf.tv_sec) * 1000 + (unsigned long long)(tvf.tv_usec) / 1000;
    std::cout << "Dubins elapsed time: " << tf-t0 << " ms." << std::endl;

    // Write the full path to a csv file
    write_path(opt_path);

    return opt_path;
}