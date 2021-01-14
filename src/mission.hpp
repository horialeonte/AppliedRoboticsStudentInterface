#include "clipper.hpp"
#include "rrt_star.hpp"
#include <algorithm>   // Vector sorting
#include <glpk.h>      // GNU Linear Programming Kit
// See: https://en.wikibooks.org/wiki/GLPK/Obtaining_GLPK
// See: https://en.wikibooks.org/wiki/GLPK/Linux_OS
// Find latest version here: http://ftp.gnu.org/gnu/glpk/

struct RRTS_params{
    unsigned int maxIt;  // Limit of iterations
    float tol;           // Tolerance to goal center point
    float d_lim;         // Maximum allowed distance at which a new candidate shall be from any existing node
    float b;             // RRT* ball radius to rewire
};

struct Dubins_params{
    float Kmax;          // Maximum curvature
    unsigned int k;      // Variable to set the number of angles to be considered at each point (minimum should be 4 for refinement to work)
    unsigned int M;      // Variable to choose the number of refinement steps
};

struct MissionPlanningNode{
    Point location;
    std::vector<float> distances;
    float reward;
};

// TODO: comment next 3 lines out
/* struct Path{
    std::vector<Pose> points;
}; */

bool sortPair(const std::pair<int,Polygon>& v1, const std::pair<int,Polygon>& v2);

std::vector<Polygon> inflate(std::vector<Polygon> obs, float R);

Point centroid(const Polygon pol);

Path mission1(const struct RRTS_params& RRTS_params, const struct Dubins_params& Dubins_params, const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, const float& R);

Path mission2(const struct RRTS_params& RRTS_params, const struct Dubins_params& Dubins_params, const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, const float& R, const float& v, const float& zz);

Path missionPlanner(const struct RRTS_params& RRTS_params, const struct Dubins_params& Dubins_params, const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, const float& R, const float& v, const float& zz, const int& mission);