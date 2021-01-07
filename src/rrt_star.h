#include "dubins.h"
#include <float.h>        // Infinity
#include <fstream>        // Write file
#include <iostream>       // Inputs and outputs
#include <limits.h>       // Infinity
#include <math.h>         // Math operations
#include <random>         // Random
#include <sys/time.h>     // Execution time
#include <vector>         // Vectors

using namespace std;

struct Point{
    double x;        // Generic 2D point - x
    double y;        // Generic 2D point - y
};

struct Node{
    Point p;
    unsigned int parent;
    double cost;
};

typedef vector<Point> Polygon, Plan;

typedef vector<Node> Graph;

bool intersLineLine(Point s1_p0, Point s1_pf, Point s2_p0, Point s2_pf);

bool isObstacleFree(vector<Point> s, vector<Polygon> obs);

void limitDistance(const Point p0, Point &pf, const double d_lim);

Plan fetchPlan(Graph graph, Node p0, Node pf);

bool isInsidePolygon(Point p, vector<Polygon> obs);

unsigned int nearestNeighbor(Point p, Graph cand);

unsigned int bestNeighbor(Point p, Graph cand, double b, vector<Polygon> obs);

void rewire(Graph &cand, const double b, const vector<Polygon> obs);

Point rnd_point(double x0, double xN, double y0, double yN);

Graph RRTstar(const Point p0, const Point pf, const Polygon borders, const vector<Polygon> obs, const unsigned int maxIt, const double tol, const double d_lim, const double b);

void write_plan(Plan opt_plan);