// ********** Strategy for the mission **********
// All obstacles, victims, and goals, are considered obstacles; except the current goal.
// The current goal might be one of the numbered victims or the gate.
// The rescued victims will be permanently removed from the obstacles list.
// Once the current goal is reached, the next numbered victim or the gate is set as the new goal.
// Keep going until the gate is reached, when the mission is considered successful.

#include "mission.hpp"

// Custom function to sort a vector of pairs by their first element in ascending order.
// See: https://www.geeksforgeeks.org/sorting-vector-of-pairs-in-c-set-1-sort-by-first-and-second/
bool sortPair(const std::pair<int,Polygon>& v1, const std::pair<int,Polygon>& v2){ return v1.first < v2.first; }

// Function to inflate a set of obstacles obs by a distance R. Returns a new set of obstacles,possibly containing less elements if inflation made some obstacles merge.
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

Path mission1(const struct RRTS_params& RRTS_params, const struct Dubins_params& Dubins_params, const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, const float& R){

    // Variables to measure computation time
    struct timeval tv0, tvf;
    unsigned long long t0;
    unsigned long long tf;

    // *********** MISSION PLANNING ************ //
    // ***********    MISSION #1    ************ //

    // The number of goals is the number of victims plus the gate
    const int numGoals = victim_list.size() + 1;

    // Sort victims on the basis of first element of pairs in ascending order.
    std::vector<std::pair<int,Polygon>> mission_vts = victim_list;
    sort(mission_vts.begin(), mission_vts.end(), sortPair);

    // ********** HIGH-LEVEL PLANNING ********** //
    // **********        RRT*         ********** //

    // Measure execution time (start)
    gettimeofday(&tv0, NULL);

    // Initialize starting point of the robot, current goal, and global plan
    Point p_start{x, y};
    Point current_goal;
    Plan plan_global;

    // Inflate obstacles by a distance equal to the robot radius.
    std::vector<Polygon> mission_obs = obstacle_list;
    mission_obs = inflate(mission_obs, R);

    // Loop over all goals, in order to select one goal at a time in a certain order
    for ( unsigned int currGoal = 1; currGoal <= numGoals; currGoal++ ){

        // Select next goal as next victim in the list or the gate.
        if (currGoal < numGoals){ current_goal = centroid(mission_vts[currGoal-1].second); }
        else if (currGoal == numGoals){ current_goal = centroid(gate); }

        // Computations: 1) Compute RRT* graph, 2) Create plan from graph.
        Graph graph = RRTstar(p_start, current_goal, borders, mission_obs, RRTS_params.maxIt, RRTS_params.tol, RRTS_params.d_lim, RRTS_params.b);
        Plan plan_local = fetchPlan(graph, graph[0], graph[graph.size()-1]);

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
    write_plan(plan_global, "Mission1_RRTS.csv");

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
    opt_path.points = refinementMarkovDubins(all_confs, Dubins_params.Kmax, Dubins_params.k, Dubins_params.M);
    gettimeofday(&tvf, NULL);
    t0 = (unsigned long long)(tv0.tv_sec) * 1000 + (unsigned long long)(tv0.tv_usec) / 1000;
    tf = (unsigned long long)(tvf.tv_sec) * 1000 + (unsigned long long)(tvf.tv_usec) / 1000;
    std::cout << "Dubins elapsed time: " << tf-t0 << " ms." << std::endl;

    // Write the full path to a csv file
    write_path(opt_path.points, "Mission1_Dubins.csv");

    return opt_path;
}

Path mission2(const struct RRTS_params& RRTS_params, const struct Dubins_params& Dubins_params, const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, const float& R, const float& v, const float& zz){

    // Variables to measure computation time
    struct timeval tv0, tvf;
    unsigned long long t0;
    unsigned long long tf;

    // *********** MISSION PLANNING ************ //
    // ***********    MISSION #2    ************ //

    // Inflate obstacles, by a distance equal to the robot radius.
    std::vector<Polygon> mission_obs = obstacle_list;
    mission_obs = inflate(obstacle_list, R);

    // The number of goals is the number of victims plus the gate
    int numGoals = victim_list.size() + 1;

    // Gather nodes for the mission graph: starting point, victims, and gate.
    std::vector<MissionPlanningNode> nodes;
    std::vector<float> distances;
    for (unsigned int i = 0; i < numGoals + 1; i++){ distances.push_back(0.0); }

    // Add all nodes to the graph, initially with null relative distances and a given reward.
    nodes.push_back(MissionPlanningNode{Point{x,y},distances,0.0});
    for (unsigned int i = 0; i < victim_list.size(); i++){
        nodes.push_back(MissionPlanningNode{centroid(victim_list[i].second),distances,zz*v});
    }
    nodes.push_back(MissionPlanningNode{centroid(gate),distances,0.0});

    // Compute relative distances between pairs of nodes
    for (unsigned int i = 0; i < nodes.size(); i++){
        for (unsigned int j = i + 1; j < nodes.size(); j++){
            // Computations: 1) Compute RRT* graph, 2) Create plan from graph, 3) Compute length of path.
            Graph graph = RRTstar(nodes[i].location, nodes[j].location, borders, mission_obs, RRTS_params.maxIt, RRTS_params.tol, RRTS_params.d_lim, RRTS_params.b);
            Plan plan_local = fetchPlan(graph, graph[0], graph[graph.size()-1]);
            float L = 0.0;
            for (unsigned int k = 1; k < plan_local.size(); k++){ L += hypot( plan_local[k].x - plan_local[k-1].x, plan_local[k].y - plan_local[k-1].y ); }
            nodes[i].distances[j] = L;
            nodes[j].distances[i] = L;
        }
    }

    // Initialize GLPK
    glp_prob *lp;
    int N = nodes.size();
    int ia[1 + (N*N-1) * (N*N+N)], ja[1 + (N*N-1) * (N*N+N)];
    double ar[1 + (N*N-1) * (N*N+N)];

    // Set problem - Choose minimization (instead of maximization)
    lp = glp_create_prob();
    glp_set_prob_name(lp, "mission2");
    glp_set_obj_dir(lp, GLP_MIN);

    // Rows of constraints. There are N^2 - 1 constraints for mission 2.
    unsigned int row = 0;
    glp_add_rows(lp, (N+1)*(N-1));

    // cond_ex_3 (1 rows)
    row += 1;
    glp_set_row_name(lp, row, "cond_ex_3");
    glp_set_row_bnds(lp, row, GLP_FX, 1, 0); // Fixed to 1
    
    // cond_ent_2 (1 rows)
    row += 1;
    glp_set_row_name(lp, row, "cond_ent_2");
    glp_set_row_bnds(lp, row, GLP_FX, 1, 0); // Fixed to 1
    
    // conn_1 (N-2 rows)
    for (unsigned int k = 2; k <= N-1; k++){
        row += 1;
        const char *varname = ("conn_1_k"+std::to_string(k)).c_str();
        glp_set_row_name(lp, row, varname);
        glp_set_row_bnds(lp, row, GLP_FX, 0, 0); // Fixed to 0
    }
    
    // conn_2 (N-2 rows)
    for (unsigned int k = 2; k <= N-1; k++){
        row += 1;
        const char *varname = ("conn_2_k"+std::to_string(k)).c_str();
        glp_set_row_name(lp, row, varname);
        glp_set_row_bnds(lp, row, GLP_UP, 0, 1); // Upper bound 1
    }
    
    // conn_3 ((N-1)^2 rows)
    for (unsigned int i = 2; i <= N; i++){
        for (unsigned int j = 2; j <= N; j++){
            row += 1;
            const char *varname = ("conn_3_i"+std::to_string(i)+"_j"+std::to_string(j)).c_str();
            glp_set_row_name(lp, row, varname);
            glp_set_row_bnds(lp, row, GLP_UP, 0, N-2); // Upper bound N-2
        }
    }

    // Decision variables x and u. There are N^2 of type x plus N of type u.
    glp_add_cols(lp, N*N+N);
    int n = 0;
    for (unsigned int i = 1; i <= N; i++){
        for (unsigned int j = 1; j <= N; j++){
            n += 1;
            const char *varname = ("x_i"+std::to_string(i)+"_j"+std::to_string(j)).c_str();
            glp_set_col_name(lp, n, varname); // Name - "x_iI_jJ"
            glp_set_col_kind(lp, n, GLP_BV);  // Binary variable - integer bounded [0,1]
            double coef = 0.0;
            if (i != N && j != 1){ coef = nodes[i-1].distances[j-1] - nodes[i-1].reward; }
            glp_set_obj_coef(lp, n, coef); // Coefficient - cost of visiting node minus associated reward
        }
    }
    for (unsigned int i = 1; i <= N; i++){
        n += 1;
        const char *varname = ("u_i"+std::to_string(i)).c_str();
        glp_set_col_name(lp, n, varname);       // Name - "u_iI"
        int lb = 2;                             // Define lower bound
        if (i == 1){ lb = 1; }                  // Define lower bound
        glp_set_col_bnds(lp, n, GLP_IV, lb, N); // Integer variable bounded [lb,N]
        glp_set_obj_coef(lp, n, 0.0);           // The u are not part of the function to be minimized
    }
    
    // Prepare the constraint matrix (i.e. constraint coefficients).
    // There should be N^2 - 1 rows (number of constraints) and N^2 + N columns (number of decision variables).
    n = 0;
    row = 0;

    // cond_ex_3 (1 rows)
    int nj = 0;
    row += 1;
    // Set coefficients for x_ij
    for (unsigned int i = 1; i <= N; i++){
        for (unsigned int j = 1; j <= N; j++){
            n += 1;
            nj += 1;
            ia[n] = row, ja[n] = nj;
            if (i != N && j == N){ ar[n] = 1; }
            else { ar[n] = 0; }
        }
    }
    // Set coefficients for u_i
    for (unsigned int i = 1; i <= N; i++){
        n += 1;
        nj += 1;
        ia[n] = row, ja[n] = nj, ar[n] = 0;
    }

    // cond_ent_2 (1 rows)
    nj = 0;
    row += 1;
    // Set coefficients for x_ij
    for (unsigned int i = 1; i <= N; i++){
        for (unsigned int j = 1; j <= N; j++){
            n += 1;
            nj += 1;
            ia[n] = row, ja[n] = nj;
            if (i == 1 && j != 1){ ar[n] = 1; }
            else { ar[n] = 0; }
        }
    }
    // Set coefficients for u_i
    for (unsigned int i = 1; i <= N; i++){
        n += 1;
        nj += 1;
        ia[n] = row, ja[n] = nj, ar[n] = 0;
    }

    // conn_1 (N-2 rows)
    for (int k = 2; k <= N-1; k++){
        nj = 0;
        row += 1;
        // Set coefficients for x_ij
        for (int i = 1; i <= N; i++){
            for (int j = 1; j <= N; j++){
                n += 1;
                nj += 1;
                ia[n] = row, ja[n] = nj, ar[n] = 0;
                if (i != N && j == k){ar[n]+=1;}
                if (i == k && j != 1){ar[n]-=1;}
            }
        }
        // Set coefficients for u_i
        for (int i = 1; i <= N; i++){
            n += 1;
            nj += 1;
            ia[n] = row, ja[n] = nj, ar[n] = 0;
        }
    }

    // conn_2 (N-2 rows)
    for (int k = 2; k <= N-1; k++){
        nj = 0;
        row += 1;
        // Set coefficients for x_ij
        for (int i = 1; i <= N; i++){
            for (int j = 1; j <= N; j++){
                n += 1;
                nj += 1;
                ia[n] = row, ja[n] = nj, ar[n] = 0;
                if (i != N && j == k){ ar[n] = 1; }
            }
        }
        // Set coefficients for u_i
        for (unsigned int i = 1; i <= N; i++){
            n += 1;
            nj += 1;
            ia[n] = row, ja[n] = nj, ar[n] = 0;
        }
    }

    // conn_3 ((N-1)^2 rows)
    for (int i = 2; i <= N; i++){
        for (int j = 2; j <= N; j++){
            nj = 0;
            row += 1;
            // Set coefficients for x_ij
            for (int ii = 1; ii <= N; ii++){
                for (int jj = 1; jj <= N; jj++){
                    n += 1;
                    nj += 1;
                    ia[n] = row, ja[n] = nj, ar[n] = 0;
                    if (ii == i && jj == j){ ar[n]=N-1; }
                }
            }
            // Set coefficients for u_i
            for (int ii = 1; ii <= N; ii++){
                n += 1;
                nj += 1;
                ia[n] = row, ja[n] = nj, ar[n] = 0;
                if (ii == i){ar[n]+=1;}
                if (ii == j){ar[n]-=1;}
            }
        }
    }

    // Load the matrix.
    glp_load_matrix(lp, n, ia, ja, ar);

    // Call the routine. First, perform LP relaxation.
    glp_simplex(lp, NULL);

    // After that, Mixed Integer Programming (MIP) is ready to be performed.
    glp_intopt(lp, NULL);

    // Fetch the results.
    std::cout << "Mission 2 objective value: " << glp_mip_obj_val(lp) << std::endl;

    // Create list of selected victims.
    std::vector<Polygon> mission_goals;
    unsigned int vi = 1;
    for (unsigned int i = 1; i <= N; i++){
        for (unsigned int j = 1; j <=N; j++){
            if ( glp_mip_col_val(lp, (vi-1)*N + j) == 1 ){
                vi = j;
                if ( vi == N ){
                    mission_goals.push_back(gate);
                } else {
                    mission_goals.push_back(victim_list[vi-2].second);
                }
            }
        }
    }

    // The GLPK problem can now be deleted.
    glp_delete_prob(lp);

    // Update number of goals.
    numGoals = mission_goals.size();

    // ********** HIGH-LEVEL PLANNING ********** //
    // **********        RRT*         ********** //

    // Measure execution time (start)
    gettimeofday(&tv0, NULL);

    // Initialize starting point of the robot, current goal, and global plan
    Point p_start{x, y};
    Point current_goal;
    Plan plan_global;

    // Loop over all goals, in order to select one goal at a time in a certain order
    for ( unsigned int currGoal = 1; currGoal <= numGoals; currGoal++ ){

        // Define current goal       
        current_goal = centroid(mission_goals[currGoal-1]);

        // Computations: 1) Compute RRT* graph, 2) Create plan from graph.
        Graph graph = RRTstar(p_start, current_goal, borders, mission_obs, RRTS_params.maxIt, RRTS_params.tol, RRTS_params.d_lim, RRTS_params.b);
        Plan plan_local = fetchPlan(graph, graph[0], graph[graph.size()-1]);

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
    write_plan(plan_global, "Mission2_RRTS.csv");

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
    opt_path.points = refinementMarkovDubins(all_confs, Dubins_params.Kmax, Dubins_params.k, Dubins_params.M);
    gettimeofday(&tvf, NULL);
    t0 = (unsigned long long)(tv0.tv_sec) * 1000 + (unsigned long long)(tv0.tv_usec) / 1000;
    tf = (unsigned long long)(tvf.tv_sec) * 1000 + (unsigned long long)(tvf.tv_usec) / 1000;
    std::cout << "Dubins elapsed time: " << tf-t0 << " ms." << std::endl;

    // Write the full path to a csv file
    write_path(opt_path.points, "Mission2_Dubins.csv");

    return opt_path;
}

Path missionPlanner(const struct RRTS_params& RRTS_params, const struct Dubins_params& Dubins_params, const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, const float& R, const float& v, const float& zz, const int& mission){
    Path path;
    // Variables to measure computation time
    struct timeval tv0, tvf;
    unsigned long long t0;
    unsigned long long tf;
    switch (mission){
    case 1:
        // Measure execution time (start)
        gettimeofday(&tv0, NULL);
        // Compute mission
        path = mission1(RRTS_params, Dubins_params, borders, obstacle_list, victim_list, gate, x, y, theta, R);
        // Measure execution time (end)
        gettimeofday(&tvf, NULL);
        t0 = (unsigned long long)(tv0.tv_sec) * 1000 + (unsigned long long)(tv0.tv_usec) / 1000;
        tf = (unsigned long long)(tvf.tv_sec) * 1000 + (unsigned long long)(tvf.tv_usec) / 1000;
        std::cout << "Mission 1 total elapsed time: " << tf-t0 << " ms." << std::endl;
        return path;
    case 2:
        // Measure execution time (start)
        gettimeofday(&tv0, NULL);
        path = mission2(RRTS_params, Dubins_params, borders, obstacle_list, victim_list, gate, x, y, theta, R, v, zz);
        // Measure execution time (end)
        gettimeofday(&tvf, NULL);
        t0 = (unsigned long long)(tv0.tv_sec) * 1000 + (unsigned long long)(tv0.tv_usec) / 1000;
        tf = (unsigned long long)(tvf.tv_sec) * 1000 + (unsigned long long)(tvf.tv_usec) / 1000;
        std::cout << "Mission 2 total elapsed time: " << tf-t0 << " ms." << std::endl;
        return path;
    default:
        throw std::runtime_error("Invalid mission");
    }
}