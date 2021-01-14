//#include "rrt_star.hpp"
#include "rrt_star.h"
// Function to check if a segment s1, defined by s1_p0 initial and s1_pf final points; intersects a segment s2, defined by s2_p0 initial and s2_pf final points.
bool intersLineLine(Point s1_p0, Point s1_pf, Point s2_p0, Point s2_pf){

    bool isInters;
    float t, u, det;

    // Minimum and maximum coordinates of sector s1
    float s1_min_x = fmin(s1_p0.x, s1_pf.x);
    float s1_max_x = fmax(s1_p0.x, s1_pf.x);
    float s1_min_y = fmin(s1_p0.y, s1_pf.y);
    float s1_max_y = fmax(s1_p0.y, s1_pf.y);

    // Minimum and maximum coordinates of sector s2
    float s2_min_x = fmin(s2_p0.x, s2_pf.x);
    float s2_max_x = fmax(s2_p0.x, s2_pf.x);
    float s2_min_y = fmin(s2_p0.y, s2_pf.y);
    float s2_max_y = fmax(s2_p0.y, s2_pf.y);

    // Segments s_i, s_j do not intersect if the maximum of s_i is smaller than the minimum of s_j
    if (s1_max_x < s2_min_x || s2_max_x < s1_min_x || s1_max_y < s2_min_y || s2_max_y < s1_min_y){
        isInters = false;
    // Otherwise, check for intersection equations
    } else {
        det = (s2_pf.x - s2_p0.x) * (s1_p0.y - s1_pf.y) - (s2_pf.y - s2_p0.y) * (s1_p0.x - s1_pf.x);
        // Theoretical intersection at infinity => Parallel segments
        if (det == 0.0){
            isInters = false;
        // Otherwise, check whether the segments intersect
        } else {
            t = ( (s2_p0.y - s2_pf.y) * (s1_p0.x - s2_p0.x) + (s2_pf.x - s2_p0.x) * (s1_p0.y - s2_p0.y) ) / det;
            u = ( (s1_p0.y - s1_pf.y) * (s1_p0.x - s2_p0.x) + (s1_pf.x - s1_p0.x) * (s1_p0.y - s2_p0.y) ) / det;
            // The given segments intersect
            if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0){
                isInters = true;
            // The given segments do not intersect
            } else {
                isInters = false;
            }
        }
    }

    return isInters;
}

// Check if a segment s is obstructed by any obstacles contained within obs.
bool isObstacleFree(std::vector<Point> s, std::vector<Polygon> obs){
    // Loop for the different obstacles
    for ( unsigned int i = 0; i < obs.size(); i++ ){
        Polygon pol_i = obs[i];
        // Loop for the different segments within the obstacle
        for ( unsigned int j = 0; j < pol_i.size(); j++ ){
            unsigned int j1 = j + 1;
            if ( j == pol_i.size()-1 ){ j1 = 0; }
            bool isInters = intersLineLine(s[0], s[1], pol_i[j], pol_i[j1]);
            // At the point there is an intersection with a polygon, return false.
            if ( isInters == true ){ return false; }
        }
    }
    // If no intersections were found, then return true.
    return true;
}

// This function forces the new random point to stay at a minimum 0.5*d and maximum d distance from its nearest neighbor
void limitDistance(const Point p0, Point &pf, const float d_lim){
    if ( hypot( pf.x - p0.x, pf.y - p0.y ) > d_lim ){
        const float th = atan2( pf.y - p0.y, pf.x - p0.x );
        pf.x = p0.x + d_lim*cos(th);
        pf.y = p0.y + d_lim*sin(th);
    } else if ( hypot( pf.x - p0.x, pf.y - p0.y ) < 0.9*d_lim ){
        const float th = atan2( pf.y - p0.y, pf.x - p0.x );
        pf.x = p0.x + d_lim/2.0*cos(th);
        pf.y = p0.y + d_lim/2.0*sin(th);
    }
}

// Function to backtrace from a node pf to a node p0, tracking parent by parent.
Plan fetchPlan(Graph graph, Node p0, Node pf){
    Graph plan;
    Plan opt_plan;
    plan.push_back(pf);
    // While the root point p0 is not reached, keep pushing parents back.
    while ( plan.back().p.x != p0.p.x || plan.back().p.y != p0.p.y ){
        plan.push_back( graph[plan.back().parent] );
    }
    // Reverse path is reversed back and points are returned.
    for ( unsigned int i = 1; i <= plan.size(); i++ ){
        opt_plan.push_back( plan[plan.size()-i].p );
    }
    return opt_plan;
}

// Function to check whether a point p is contained within any of the obstacles in the obs vector of obstacles.
// See: https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
bool isInsidePolygon(Point p, std::vector<Polygon> obs){
    // Draw a horizontal line from the point to infinity
    Point p_inf{FLT_MAX, p.y};
    // Loop for the different obstacles
    for ( unsigned int i = 0; i < obs.size(); i++ ){
        Polygon pol_i = obs[i];
        // Loop for the different segments within the obstacle
        unsigned int n = 0;
        for ( unsigned int j = 0; j < pol_i.size(); j++ ){
            unsigned int j1 = j + 1;
            if ( j == pol_i.size()-1 ){ j1 = 0; }
            bool isInters = intersLineLine(p, p_inf, pol_i[j], pol_i[j1]);
            n += isInters;
        }
        // If the number of intersections is odd (=1 for convex pol), the point is inside the polygon.
        if ( n % 2 != 0 ){ return true; }
    }
    // If no polygon contains the point, then return false.
    return false;
}

// Function to find the nearest neighbor of a given point p, among a list of candidate nodes cand.
unsigned int nearestNeighbor(Point p, Graph cand){
    float d_nn = FLT_MAX;
    unsigned int i_nn = UINT_MAX;
    for ( unsigned int i = 0; i < cand.size(); i++ ){
        // Compute Euclidean distance
        float d_i = hypot( cand[i].p.x - p.x, cand[i].p.y - p.y );
        if ( d_i < d_nn ){
            d_nn = d_i;
            i_nn = i;
        }
    }
    return i_nn;
}

// Function to find a suitable parent for a given point p, among a list of candidate nodes cand.
unsigned int bestNeighbor(Point p, Graph cand, float b, std::vector<Polygon> obs){
    float d_nn = FLT_MAX;
    unsigned int i_nn = UINT_MAX;
    for ( unsigned int i = 0; i < cand.size(); i++ ){
        float d_ij = hypot( cand[i].p.x - p.x, cand[i].p.y - p.y );
        // Dismiss candidate if it is not within the ball of radius b.
        if ( d_ij > b ){ continue; }

        // Create a segment to check whether rewiring would interfer with an obstacle
        std::vector<Point> s;
        s.push_back(p);
        s.push_back(cand[i].p);
        // Discard the candidate to best neighbor straight away if the segment finds an obstacle.
        if ( isObstacleFree(s, obs) == false ){ continue; }
        
        // Not only the distance is considered, but the whole cost from the root node.
        float d_i = cand[i].cost + d_ij;
        if ( d_i < d_nn ){
            d_nn = d_i;
            i_nn = i;
        }
    }
    return i_nn;
}

// Function to rewire around the last added node n_new, considering a radius b, and avoiding obstacles obs.
// NOTE: neighbors can only be rewired to the new node.
// This is because existing neighbors are assumed to have been optimally attached already.
// So just check whether the new member can improve that.
void rewire(Graph &cand, const float b, const std::vector<Polygon> obs){
    Node n_new = cand.back();
    for ( unsigned int i = 0; i < cand.size()-1; i++ ){
        float d_ij = hypot( cand[i].p.x - n_new.p.x, cand[i].p.y - n_new.p.y );
        // Dismiss candidate if it is not within the ball of radius b.
        if ( d_ij > b ){ continue; }
        
        // Create a segment to check whether rewiring would interfer with an obstacle
        std::vector<Point> s;
        s.push_back(cand[i].p);
        s.push_back(n_new.p);
        // Discard the rewiring straight away if the segment finds an obstacle.
        if ( isObstacleFree(s, obs) == false ){ continue; }

        // Compute whether changing parent reduces cost.
        float d_i = n_new.cost + d_ij;
        if ( d_i < cand[i].cost ){
            cand[i].cost = d_i;
            cand[i].parent = cand.size()-1;
        }
    }
}

// Generate a random point in the range [(x0, y0), (xN, yN)].
// See: https://en.cppreference.com/w/cpp/numeric/random/uniform_real_distribution
Point rnd_point(const float x0, const float xN, const float y0, const float yN){
    // Will be used to obtain a seed for the random number engine
    std::random_device rd;
    // Standard mersenne_twister_engine seeded with rd()
    std::mt19937 gen(rd());
    // Use unif_x and unif_y to transform the random unsigned int generated by gen into a 
    // float in [x0, xN), [y0, yN), respectively.
    std::uniform_real_distribution<float> unif_x(x0, xN), unif_y(y0, yN);
    // Each call to unif_x(gen) or unif_y(gen) generates a new random float.
    return Point{unif_x(gen), unif_y(gen)};
}

// Perform RRT* within a rectangular arena defined by arena, limiting the iterations to maxIt.
// Threshold distance d_lim, RRT* ball radius b, starting point p0, goal pf, set of obstacles obs.
Graph RRTstar(const Point p0, const Point pf, const Polygon borders, const std::vector<Polygon> obs, const unsigned int maxIt, const float tol, const float d_lim, const float b){

    // Initialize the graph with the root node, corresponding to point p0.
    Graph graph;
    graph.push_back(Node{p0, UINT_MAX, 0.0});

    // Create a test segment from the start to the goal
    std::vector<Point> start_to_goal;
    start_to_goal.push_back(p0);
    start_to_goal.push_back(pf);
    // If the goal can be already reached, do it and avoid searching for further nodes.
    if ( isObstacleFree(start_to_goal, obs) == true ){
        graph.push_back(Node{pf, 0, float(hypot( p0.x - pf.x, p0.y - pf.y ))});
        std::cout << "RRT* converged. The graph has a total of " << graph.size() << " nodes. Straight path to goal was already obstacle free!" << std::endl;
        return graph;
    }

    // Assume the arena has the shape of a rectangle
    const float x0 = borders[0].x;
    const float y0 = borders[0].y;
    const float xN = borders[1].x;
    const float yN = borders[2].y;

    // Initialize like this, to check whether a one-shot direct path p0-pf exists.
    Point potential_node = p0;
    float potential_cost = 0.0;
    unsigned int ind_potential_parent;

    // Start finding random points and connecting them to the graph.
    unsigned int it = 0;
    while ( it < maxIt ){
        it += 1;
        potential_node = rnd_point(x0, xN, y0, yN);
        ind_potential_parent = nearestNeighbor(potential_node, graph);
        // Recompute the candidate node so as to keep it within the limit connection distance.
        limitDistance(graph[ind_potential_parent].p, potential_node, d_lim);
        // EXCLUSIVE to RRT*: choose best parent among those contained in a ball of radius b around the generated child.
        unsigned int temp_ind_potential_parent = bestNeighbor(potential_node, graph, b, obs);
        if ( temp_ind_potential_parent != UINT_MAX ){ ind_potential_parent = temp_ind_potential_parent; }
        // Create a segment from the parent to the child
        std::vector<Point> s;
        s.push_back(graph[ind_potential_parent].p);
        s.push_back(potential_node);
        // Discard the point straight away if the segment finds an obstacle, whether the generated child is inside an obstacle or outside.
        if ( isObstacleFree(s, obs) == false ){ continue; }
        // Otherwise, add it to the graph.
        potential_cost = graph[ind_potential_parent].cost + hypot( graph[ind_potential_parent].p.x - potential_node.x, graph[ind_potential_parent].p.y - potential_node.y );
        graph.push_back(Node{potential_node, ind_potential_parent, potential_cost});
        // EXCLUSIVE to RRT*: once the new child has been added, try to rewire among neighbors.
        rewire(graph, b, obs);
        // If the goal is already reached, stop searching for further nodes.
        if ( hypot( pf.x - graph.back().p.x, pf.y - graph.back().p.y ) < tol ){ break; }
    }
    if (it == maxIt){ throw std::runtime_error("RRT* could not converge in the specified number of iterations."); }
    std::cout << "RRT* converged. The graph has a total of " << graph.size() << " nodes. Number of iterations has been " << it << "." << std::endl;
    return graph;
}

// Write a RRT* plan in a csv file
void write_plan(Plan opt_plan, std::string filename){
    std::ofstream csv_file;
    //csv_file.open("rrt_star.csv");
    csv_file.open(filename);
    csv_file << "x" << "," << "y" << std::endl;
    // Loop over the RRT* points within the input array
    for (unsigned int n=0; n<opt_plan.size(); n++){
        Point p = opt_plan[n];
        csv_file << p.x << "," << p.y << std::endl;
    }
    csv_file.close();
}
