#pragma once
#include "dubins.h"
#include <float.h>        // Infinity
#include <fstream>        // Write file
#include <iostream>       // Inputs and outputs
#include <limits.h>       // Infinity
#include <math.h>         // Math operations
#include <random>         // Random
#include <sys/time.h>     // Execution time
#include <vector>         // Vectors

// TODO: comment next 4 lines out
//struct Point{
//    float x;        // Generic 2D point - x
//    float y;        // Generic 2D point - y
//};

struct Node{
    Point p;
    unsigned int parent;
    float cost;
};

typedef std::vector<Point> Plan;

// TODO: comment next line out
//typedef std::vector<Point> Polygon;

typedef std::vector<Node> Graph;

bool intersLineLine(Point s1_p0, Point s1_pf, Point s2_p0, Point s2_pf);

bool isObstacleFree(std::vector<Point> s, std::vector<Polygon> obs);

void limitDistance(const Point p0, Point &pf, const float d_lim);

Plan fetchPlan(Graph graph, Node p0, Node pf);

bool isInsidePolygon(Point p, std::vector<Polygon> obs);

unsigned int nearestNeighbor(Point p, Graph cand);

unsigned int bestNeighbor(Point p, Graph cand, float b, std::vector<Polygon> obs);

void rewire(Graph &cand, const float b, const std::vector<Polygon> obs);

Point rnd_point(float x0, float xN, float y0, float yN);

Graph RRTstar(const Point p0, const Point pf, const Polygon borders, const std::vector<Polygon> obs, const unsigned int maxIt, const float tol, const float d_lim, const float b);

void write_plan(Plan opt_plan, std::string filename);
