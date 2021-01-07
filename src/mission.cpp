// Compile as:
// g++ -std=c++11 -o mission mission.cpp clipper.cpp dubins.cpp rrt_star.cpp

// Run as:
// ./mission

// ********** Strategy for the mission **********
// All obstacles, victims, and goals, are considered obstacles; except the current goal.
// The current goal might be one of the numbered victims or the gate.
// The rescued victims will be permanently removed from the obstacles list.
// Once the current goal is reached, the next numbered victim or the gate is set as the new goal.
// Keep going until the gate is reached, when the mission is considered successful.

#include "mission.h"

Polygon loadBorders(){
    Polygon borders;

    // Define rectangle
    const double x0 = 0.0;
    const double y0 = 0.0;
    const double xN = 1.5;
    const double yN = 1.0;

    borders.push_back(Point{x0,y0});
    borders.push_back(Point{xN,y0});
    borders.push_back(Point{xN,yN});
    borders.push_back(Point{x0,yN});

    return borders;
}

vector<Polygon> loadObstacles(){
    vector<Polygon> obstacle_list;

    // Obstacle 1
    Polygon rect_1;
    rect_1.push_back(Point{0.25,0.00});
    rect_1.push_back(Point{0.30,0.00});
    rect_1.push_back(Point{0.30,0.80});
    rect_1.push_back(Point{0.25,0.80});
    obstacle_list.push_back(rect_1);

    // Obstacle 2
    Polygon rect_2;
    rect_2.push_back(Point{0.65,0.20});
    rect_2.push_back(Point{0.75,0.20});
    rect_2.push_back(Point{0.75,1.00});
    rect_2.push_back(Point{0.65,1.00});
    obstacle_list.push_back(rect_2);

/*     // Obstacle 3
    Polygon rect_3;
    rect_3.push_back(Point{0.55,0.00});
    rect_3.push_back(Point{0.65,0.00});
    rect_3.push_back(Point{0.65,0.80});
    rect_3.push_back(Point{0.55,0.80});
    obstacle_list.push_back(rect_3);

    // Obstacle 4
    Polygon rect_4;
    rect_4.push_back(Point{0.75,0.20});
    rect_4.push_back(Point{0.85,0.20});
    rect_4.push_back(Point{0.85,1.00});
    rect_4.push_back(Point{0.75,1.00});
    obstacle_list.push_back(rect_4); */

    return obstacle_list;
}

vector<pair<int,Polygon>> loadVictims(){
    vector<pair<int,Polygon>> victim_list;

    // Victim 1
    Polygon rect_1;
    rect_1.push_back(Point{0.00,0.40});
    rect_1.push_back(Point{0.05,0.40});
    rect_1.push_back(Point{0.05,0.45});
    rect_1.push_back(Point{0.00,0.45});
    victim_list.push_back(pair<int,Polygon>{1,rect_1});

    // Victim 2
    Polygon rect_2;
    rect_2.push_back(Point{1.40,0.40});
    rect_2.push_back(Point{1.45,0.40});
    rect_2.push_back(Point{1.45,0.45});
    rect_2.push_back(Point{1.40,0.45});
    victim_list.push_back(pair<int,Polygon>{2,rect_2});

    return victim_list;
}

Polygon loadGate(){
    Polygon gate;
    gate.push_back(Point{0.90,0.90});
    gate.push_back(Point{1.00,0.90});
    gate.push_back(Point{1.00,1.00});
    gate.push_back(Point{0.90,1.00});
    return gate;
}

// See: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperOffset/_Body.htm
vector<Polygon> inflate(vector<Polygon> obs, double R){
    // The Clipper library only works with int, therefore our doubles are scaled to int trying not to lose much precision.
    double scale_int = 10000;
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

    // Scale back to double and return
    vector<Polygon> obs_infl;
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

int main(){

    // Variables to measure computation time
    struct timeval tv0, tvf;
    unsigned long long t0;
    unsigned long long tf;
    
    // Tune RRT* parameters
    const unsigned int maxIt = 5000;   // Limit of iterations
    const double tol = 0.02;           // Tolerance to goal center point
    const double d_lim = 0.07;         // Maximum allowed distance at which a new candidate shall be from any existing node
    const double b = 0.30;             // RRT* ball radius to rewire

    // Tune Dubins parameters
    const double Kmax = 10.0;          // Maximum curvature
    const unsigned int k = 4;          // Variable to set the number of angles to be considered at each point (minimum should be 4 for refinement to work)
    const unsigned int M = 16;         // Variable to choose the number of refinement steps

    // Define radius of the circle containing the robot, to later inflate the obstacles; and initial point and orientation.
    const double R = 0.10;
    Point p_start{0.10, 0.10};
    const double th0 = M_PI/4;

    // Define arena borders, obstacles, victims, and gate
    const Polygon                   borders       = loadBorders();
    const vector<Polygon>           obstacle_list = loadObstacles();
    const vector<pair<int,Polygon>> victim_list   = loadVictims();
    const Polygon                   gate          = loadGate();

    // ********** HIGH-LEVEL PLANNING ********** //
    // **********        RRT*         ********** //

    // Measure execution time (start)
    gettimeofday(&tv0, NULL);

    Plan plan_global;

    // The number of goals is the number of victims plus the gate
    const int numGoals = victim_list.size() + 1;

    for ( unsigned int currGoal = 1; currGoal <= numGoals; currGoal++ ){

        vector<Polygon> mission_obs = obstacle_list;
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
        Graph graph = RRTstar(p_start, current_goal, borders, mission_obs, maxIt, tol, d_lim, b);
        Plan plan_local = fetchPlan(graph, graph[0], graph[graph.size()-1]);

        cout << "Size graph: " << graph.size() << ". Size plan: " << plan_local.size() << endl;

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
    cout << "RRT* elapsed time: " << tf-t0 << " ms." << endl;

    // Write the resulting plan in a csv file.
    write_plan(plan_global);

    // ********** LOW-LEVEL PLANNING ********** //
    // **********   Dubins curves    ********** //

    vector<conf2D> all_confs;
    vector<dubinsCurve> opt_path;

    // Prepare RRT* points to be the baseline for Dubins paths
    all_confs.push_back(conf2D{plan_global[0].x, plan_global[0].y, th0});
    for ( unsigned int i = 1; i < plan_global.size(); i++ ){
        all_confs.push_back(conf2D{plan_global[i].x, plan_global[i].y, atan2( plan_global[i-1].x - plan_global[i].x, plan_global[i-1].y - plan_global[i].y)});
    }

    // Perform Iterative Dynamic Programming
    gettimeofday(&tv0, NULL);
    opt_path = refinementMarkovDubins(all_confs, Kmax, k, M);
    gettimeofday(&tvf, NULL);
    t0 = (unsigned long long)(tv0.tv_sec) * 1000 + (unsigned long long)(tv0.tv_usec) / 1000;
    tf = (unsigned long long)(tvf.tv_sec) * 1000 + (unsigned long long)(tvf.tv_usec) / 1000;
    cout << "Dubins elapsed time: " << tf-t0 << " ms." << endl;

    // Write the full path to a csv file
    write_path(opt_path);
}