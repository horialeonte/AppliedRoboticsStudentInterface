#include <float.h>        // Infinity
#include <fstream>        // Write file
#include <iostream>       // Inputs and outputs
#include <limits.h>       // Infinity
#include <math.h>         // Math operations
#include <sys/time.h>     // Execution time
#include <vector>         // Vectors

using namespace std;

// Structure defining a generic transformation of two points to standard coordinates.
struct confStandard{
    double sc_th0;   // Orientation of starting point
    double sc_thf;   // Orientation of final point
    double sc_Kmax;  // Maximum curvature desired
    double lambda;   // Scaling factor - useful for transforming back
};

// Structure defining a generic Dubins CSC primitive, which could be LSL, RSR, LSR, RSL, RLR, or LRL.
struct dubinsPrimitive{
    double s1;       // Length of first sector, in standard coordinates, of a generic CSC primitive
    int k1;          // Curvature sign for first sector (L --> +1, S --> 0, R --> -1)
    double s2;       // Length of second sector, in standard coordinates, of a generic CSC primitive
    int k2;          // Curvature sign for second sector (L --> +1, S --> 0, R --> -1)
    double s3;       // Length of third sector, in standard coordinates, of a generic CSC primitive
    int k3;          // Curvature sign for third sector (L --> +1, S --> 0, R --> -1)
    bool ok;         // Indicates whether it has been mathematically possible to compute the CSC primitive
};

// Structure defining a generic configuration in 2D, with x and y coordinates and orientation angle.
struct conf2D{
    double x;        // Generic 2D point - x
    double y;        // Generic 2D point - y
    double th;       // Generic 2D point - orientation
};

// Structure defining a Dubins 2D arc, with starting point, curvature, length, and final point.
struct dubinsArc{
    conf2D p0;       // Starting point
    conf2D pf;       // Final point
    double k;        // Curvature, can take values +Kmax (L), 0 (S), or -Kmax (R)
    double L;        // Length of the arc
};

// Structure defining a full Dubins 2D curve, composed of three Dubins arcs and with total length L.
struct dubinsCurve{
    dubinsArc a1;    // First arc of the curve
    dubinsArc a2;    // Second arc of the curve
    dubinsArc a3;    // Third arc of the curve
    double L;        // Length of the curve
};

double mod2pi(double angle);

double my_sinc(double t);

confStandard scale_to_standard(conf2D p0, conf2D pf, double Kmax);

dubinsPrimitive LSL(confStandard p_std);

dubinsPrimitive RSR(confStandard points);

dubinsPrimitive LSR(confStandard points);

dubinsPrimitive RSL(confStandard points);

dubinsPrimitive RLR(confStandard points);

dubinsPrimitive LRL(confStandard points);

conf2D dubinspoint(double s, conf2D p0, int k);

dubinsArc dubinsarc(conf2D p0, int k, double L);

dubinsCurve dubinscurve(conf2D p0, dubinsPrimitive prim, double Kmax, double lambda);

dubinsCurve twoPointsMarkovDubins(conf2D p0, conf2D pf, double Kmax);

vector<conf2D> multipointMarkovDubins(const vector<conf2D> all_confs, const double Kmax, const unsigned int k, const unsigned int m);

vector<dubinsCurve> refinementMarkovDubins(vector<conf2D> all_confs, const double Kmax, const unsigned int k, const unsigned int M);

void write_path(vector<dubinsCurve> opt_path);