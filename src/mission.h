#include "clipper.hpp"
#include "rrt_star.h"

using namespace std;

Polygon loadArena();

vector<Polygon> loadObstacles();

vector<pair<int,Polygon>> loadVictims();

Polygon loadGate();

vector<Polygon> inflate(vector<Polygon> obs, double R);

Point centroid(const Polygon pol);

int main();