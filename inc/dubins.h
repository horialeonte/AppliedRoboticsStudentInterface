#pragma once
#include "utils.hpp"
#include <float.h>        // Infinity
#include <fstream>        // Write file
#include <iostream>       // Inputs and outputs
#include <limits.h>       // Infinity
#include <math.h>         // Math operations
#include <sys/time.h>     // Execution time

// Structure defining a generic transformation of two points to standard coordinates.
struct confStandard{
    float sc_th0;   // Orientation of starting point
    float sc_thf;   // Orientation of final point
    float sc_Kmax;  // Maximum curvature desired
    float lambda;   // Scaling factor - useful for transforming back
};

// Structure defining a generic Dubins CSC primitive, which could be LSL, RSR, LSR, RSL, RLR, or LRL.
struct dubinsPrimitive{
    float s1;       // Length of first sector, in standard coordinates, of a generic CSC primitive
    int k1;         // Curvature sign for first sector (L --> +1, S --> 0, R --> -1)
    float s2;       // Length of second sector, in standard coordinates, of a generic CSC primitive
    int k2;         // Curvature sign for second sector (L --> +1, S --> 0, R --> -1)
    float s3;       // Length of third sector, in standard coordinates, of a generic CSC primitive
    int k3;         // Curvature sign for third sector (L --> +1, S --> 0, R --> -1)
    bool ok;        // Indicates whether it has been mathematically possible to compute the CSC primitive
};

// Structure defining a Dubins 2D arc, with starting point, curvature, length, and final point.
struct dubinsArc{
    Pose p0;        // Starting point
    Pose pf;        // Final point
    float k;        // Curvature, can take values +Kmax (L), 0 (S), or -Kmax (R)
    float L;        // Length of the arc
};

// Structure defining a full Dubins 2D curve, composed of three Dubins arcs and with total length L.
struct dubinsCurve{
    dubinsArc a1;   // First arc of the curve
    dubinsArc a2;   // Second arc of the curve
    dubinsArc a3;   // Third arc of the curve
    float L;        // Length of the curve
};

float mod2pi(float angle);

float my_sinc(float t);

confStandard scale_to_standard(Pose p0, Pose pf, float Kmax);

dubinsPrimitive LSL(confStandard p_std);

dubinsPrimitive RSR(confStandard points);

dubinsPrimitive LSR(confStandard points);

dubinsPrimitive RSL(confStandard points);

dubinsPrimitive RLR(confStandard points);

dubinsPrimitive LRL(confStandard points);

Pose dubinspoint(float s, Pose p0, int k);

dubinsArc dubinsarc(Pose p0, int k, float L);

dubinsCurve dubinscurve(Pose p0, dubinsPrimitive prim, float Kmax, float lambda);

dubinsCurve twoPointsMarkovDubins(Pose p0, Pose pf, float Kmax);

std::vector<Pose> multipointMarkovDubins(const std::vector<Pose> all_confs, const float Kmax, const unsigned int k, const unsigned int m);

std::vector<Pose> refinementMarkovDubins(std::vector<Pose> all_confs, const float Kmax, const unsigned int k, const unsigned int M);

void write_path(std::vector<Pose> opt_path, std::string filename);
