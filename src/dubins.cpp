#include "dubins.h"

// Auxiliary function to wrap any angle within the interval [0, 2*pi)
float mod2pi(float angle){ return angle - (2.0 * M_PI) * floor( angle / (2.0 * M_PI) ); }

// Modified sinc function to avoid numerical instabilities close to the origin
float my_sinc(float t){
    float s;
    if ( fabs(t) < 0.002 ){ s = 1 - pow(t,2)/6 * ( 1 - pow(t,2)/20 ); }
    else { s = sin(t)/t; }
    return s;
}

// Function to scale points, from generic setting to standard setting
confStandard scale_to_standard(Pose p0, Pose pf, float Kmax){

    // Computations
    float dx = pf.x - p0.x;
    float dy = pf.y - p0.y;
    float phi = atan2( dy, dx );
    float lambda = hypot( dx, dy ) / 2.0;

    // Scaled values in standard coordinates
    float sc_th0 = mod2pi(p0.theta - phi);
    float sc_thf = mod2pi(pf.theta - phi);
    float sc_Kmax = Kmax * lambda;

    return confStandard{sc_th0, sc_thf, sc_Kmax, lambda};
}

// Primitive 1: LSL
dubinsPrimitive LSL(confStandard p_std){
    float s1, s2, s3, rhomin, C, S, temp1, temp2;
    bool ok;

    float sc_th0 = p_std.sc_th0;
    float sc_thf = p_std.sc_thf;
    float sc_Kmax = p_std.sc_Kmax;

    rhomin = 1 / sc_Kmax;
    C = cos(sc_thf) - cos(sc_th0);
    S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
    temp1 = atan2(C, S);
    s1 = rhomin * mod2pi(temp1 - sc_th0);
    temp2 = 2 + 4 * pow(sc_Kmax, 2) - 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * ( sin(sc_th0) - sin(sc_thf) );
    if ( temp2 < 0 ){
        s1 = 0;
        s2 = 0;
        s3 = 0;
        ok = false;
    } else {
        s2 = rhomin * sqrt(temp2);
        s3 = rhomin * mod2pi(sc_thf - temp1);
        ok = true;
    }
    return dubinsPrimitive{s1, 1, s2, 0, s3, 1, ok};
}

// Primitive 2: RSR
dubinsPrimitive RSR(confStandard points){
    float s1, s2, s3, rhomin, C, S, temp1, temp2;
    bool ok;

    float sc_th0 = points.sc_th0;
    float sc_thf = points.sc_thf;
    float sc_Kmax = points.sc_Kmax;

    rhomin = 1 / sc_Kmax;
    C = cos(sc_th0) - cos(sc_thf);
    S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
    temp1 = atan2(C, S);
    s1 = rhomin * mod2pi(sc_th0 - temp1);
    temp2 = 2 + 4 * pow(sc_Kmax, 2) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * ( sin(sc_th0) - sin(sc_thf) );
    if ( temp2 < 0 ){
        s1 = 0;
        s2 = 0;
        s3 = 0;
        ok = false;
    } else {
        s2 = rhomin * sqrt(temp2);
        s3 = rhomin * mod2pi(temp1 - sc_thf);
        ok = true;
    }
    return dubinsPrimitive{s1, -1, s2, 0, s3, -1, ok};
}

// Primitive 3: LSR
dubinsPrimitive LSR(confStandard points){
    float s1, s2, s3, rhomin, C, S, temp1, temp2, temp3;
    bool ok;

    float sc_th0 = points.sc_th0;
    float sc_thf = points.sc_thf;
    float sc_Kmax = points.sc_Kmax;

    rhomin = 1 / sc_Kmax;
    C = cos(sc_th0) + cos(sc_thf);
    S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);
    temp1 = atan2(-C, S);
    temp3 = 4 * pow(sc_Kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * ( sin(sc_th0) + sin(sc_thf) );
    if ( temp3 < 0 ){
        s1 = 0;
        s2 = 0;
        s3 = 0;
        ok = false;
    } else {
        s2 = rhomin * sqrt(temp3);
        temp2 = -atan2(-2, s2 * sc_Kmax);
        s1 = rhomin * mod2pi(temp1 + temp2 - sc_th0);
        s3 = rhomin * mod2pi(temp1 + temp2 - sc_thf);
        ok = true;
    }
    return dubinsPrimitive{s1, 1, s2, 0, s3, -1, ok};
}

// Primitive 4: RSL
dubinsPrimitive RSL(confStandard points){
    float s1, s2, s3, rhomin, C, S, temp1, temp2, temp3;
    bool ok;

    float sc_th0 = points.sc_th0;
    float sc_thf = points.sc_thf;
    float sc_Kmax = points.sc_Kmax;

    rhomin = 1 / sc_Kmax;
    C = cos(sc_th0) + cos(sc_thf);
    S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);
    temp1 = atan2(C, S);
    temp3 = 4 * pow(sc_Kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * ( sin(sc_th0) + sin(sc_thf) );
    if ( temp3 < 0 ){
        s1 = 0;
        s2 = 0;
        s3 = 0;
        ok = false;
    } else {
        s2 = rhomin * sqrt(temp3);
        temp2 = atan2(2, s2 * sc_Kmax);
        s1 = rhomin * mod2pi(sc_th0 - temp1 + temp2);
        s3 = rhomin * mod2pi(sc_thf - temp1 + temp2);
        ok = true;
    }
    return dubinsPrimitive{s1, -1, s2, 0, s3, 1, ok};
}

// Primitive 5: RLR
dubinsPrimitive RLR(confStandard points){
    float s1, s2, s3, rhomin, C, S, temp1, temp2;
    bool ok;

    float sc_th0 = points.sc_th0;
    float sc_thf = points.sc_thf;
    float sc_Kmax = points.sc_Kmax;

    rhomin = 1 / sc_Kmax;
    C = cos(sc_th0) - cos(sc_thf);
    S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
    temp1 = atan2(C, S);
    temp2 = 0.125 * ( 6 - 4 * pow(sc_Kmax, 2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * ( sin(sc_th0) - sin(sc_thf) ) );
    if ( fabs(temp2) > 1 ){
        s1 = 0;
        s2 = 0;
        s3 = 0;
        ok = false;
    } else {
        s2 = rhomin * mod2pi( 2 * M_PI - acos(temp2) );
        s1 = rhomin * mod2pi( sc_th0 - temp1 + 0.5 * s2 * sc_Kmax );
        s3 = rhomin * mod2pi( sc_th0 - sc_thf + sc_Kmax * (s2 - s1) );
        ok = true;
    }
    return dubinsPrimitive{s1, -1, s2, 1, s3, -1, ok};
}

// Primitive 6: LRL
dubinsPrimitive LRL(confStandard points){
    float s1, s2, s3, rhomin, C, S, temp1, temp2;
    bool ok;

    float sc_th0 = points.sc_th0;
    float sc_thf = points.sc_thf;
    float sc_Kmax = points.sc_Kmax;

    rhomin = 1 / sc_Kmax;
    C = cos(sc_thf) - cos(sc_th0);
    S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
    temp1 = atan2(C, S);
    temp2 = 0.125 * ( 6 - 4 * pow(sc_Kmax, 2) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * ( sin(sc_th0) - sin(sc_thf) ) );
    if ( fabs(temp2) > 1 ){
        s1 = 0;
        s2 = 0;
        s3 = 0;
        ok = false;
    } else {
        s2 = rhomin * mod2pi( 2 * M_PI - acos(temp2) );
        s1 = rhomin * mod2pi( temp1 - sc_th0 + 0.5 * s2 * sc_Kmax );
        s3 = rhomin * mod2pi( sc_thf - sc_th0 + sc_Kmax * (s2 - s1) );
        ok = true;
    }
    return dubinsPrimitive{s1, 1, s2, -1, s3, 1, ok};
}

// Evaluate an arc (circular or straight) composing a Dubins curve, at a given arc-length s
Pose dubinspoint(float s, Pose p0, float k){
    float x = p0.x + s * my_sinc(k * s / 2.0) * cos(p0.theta + k * s / 2);
    float y = p0.y + s * my_sinc(k * s / 2.0) * sin(p0.theta + k * s / 2);
    float theta = mod2pi(p0.theta + k * s);
    return Pose{s, x, y, theta, k};
}

// Evaluate arc point at curvilinear abscissa L to get the final point of the arc
dubinsArc dubinsarc(Pose p0, float k, float L){
    Pose pf = dubinspoint(L, p0, k);
    return dubinsArc{p0, pf, k, L};
}

// Create a complete curve in the absolute frame from a Dubins primitive
dubinsCurve dubinscurve(Pose p0, dubinsPrimitive prim, float Kmax, float lambda){
    dubinsArc a1 = dubinsarc(p0,    Kmax*prim.k1, lambda*prim.s1);
    dubinsArc a2 = dubinsarc(a1.pf, Kmax*prim.k2, lambda*prim.s2);
    dubinsArc a3 = dubinsarc(a2.pf, Kmax*prim.k3, lambda*prim.s3);
    float L = a1.L + a2.L + a3.L;
    return dubinsCurve{a1, a2, a3, L};
}

// Compute the optimal Dubins curve between two 2D configurations.
dubinsCurve twoPointsMarkovDubins(Pose p0, Pose pf, float Kmax){

    // Transform to standard coordinates
    confStandard p_std = scale_to_standard(p0, pf, Kmax);
    float lambda = p_std.lambda;

    // Compute the six possible Dubins primitives and add them to an array of primitives
    dubinsPrimitive LSL_arc = LSL(p_std);
    dubinsPrimitive RSR_arc = RSR(p_std);
    dubinsPrimitive LSR_arc = LSR(p_std);
    dubinsPrimitive RSL_arc = RSL(p_std);
    dubinsPrimitive RLR_arc = RLR(p_std);
    dubinsPrimitive LRL_arc = LRL(p_std);
    dubinsPrimitive primitives[6] = {LSL_arc, RSR_arc, LSR_arc, RSL_arc, RLR_arc, LRL_arc};

    // Iterate over the six calculated primitives to find one with the minimum length
    float opt_L = FLT_MAX;
    dubinsPrimitive opt_prim;
    int ind = -1;
    for( unsigned int i = 0; i < 6; i++ ){
        dubinsPrimitive prim_i = primitives[i];
        float prim_L = prim_i.s1 + prim_i.s2 + prim_i.s3;
        if (prim_L < opt_L && prim_i.ok == 1){
            opt_L = prim_L;
            opt_prim = prim_i;
            ind = i;
        }
    }

    // If any primitive was found, then compute the corresponding curve
    dubinsCurve opt_curve;
    if (ind > -1){
        opt_curve = dubinscurve(p0, opt_prim, Kmax, lambda);
    } else { std::cout << "Sorry, no Dubins curve was found for these inputs." << std::endl; }

    return opt_curve;
}

// Compute the optimal Dubins path between (at least) four 2D configurations.
// The initial and final configurations must be totally fixed, including orientation.
// The intermediate configurations must be fixed only in x and y.
// The orientations of the intermediate configurations can be set to any number, they are going to be optimized.
std::vector<Pose> multipointMarkovDubins(const std::vector<Pose> all_confs, const float Kmax, const unsigned int k, const unsigned int m){

    // Initialize variables for the loop
    std::vector<Pose> opt_confs;
    Pose conf_j, conf_j1, conf_j2, opt_conf_j, opt_conf_j1;
    float opt_L_j1_j2, h_m;
    float L_j0_j_k = 0.0;
    float L_j_j1_k = 0.0;
    float L_j1_j2_k = 0.0;
    float L_j2_n  = 0.0;

    // First and last configurations are constraints of the problem
    const Pose conf_j0 = all_confs[0];
    const Pose conf_jn = all_confs[all_confs.size()-1];

    // Add the nth configuration to the solution vector
    opt_confs.push_back(conf_jn);

    // Refinement steps
    if ( m == 1 ){ h_m = M_PI; } else { h_m = 2.0*M_PI/3.0*pow(3.0/(float)k, (float)m); }

    // Iterate over pair of points, starting from j = n-2; and going backwards down to j = 1.
    for ( unsigned int j = all_confs.size() - 3; j > 0; j-- ){

        // Take configurations at points j+2, j+1, and j
        conf_j2 = all_confs[j + 2];
        conf_j1 = all_confs[j + 1];
        conf_j  = all_confs[j];

        // Initialize distance from point j to point n, for a set k of configurations, as infinity
        float L_j_n_k = FLT_MAX;

        // Refinement steps
        const float th_j_orig = conf_j.theta;
        const float th_j1_orig = conf_j1.theta;

        // Double for loop to consider several pairs of angles th_j, th_j1.
        for ( unsigned int k_j = 0; k_j < k + 1; k_j++ ){
            conf_j.theta = mod2pi( th_j_orig + h_m * ( 2.0 * (float)k_j/(float)k - 1.0 ) );
            for ( unsigned int k_j1 = 0; k_j1 < k + 1; k_j1++ ){
                conf_j1.theta = mod2pi( th_j1_orig + h_m * ( 2.0 * (float)k_j1/(float)k - 1.0 ) );

                // Compute the Dubins curve from j+1 to j+2. Recall that j+2 already has a fixed angle.
                L_j1_j2_k = twoPointsMarkovDubins(conf_j1, conf_j2, Kmax).L;

                // Compute the Dubins curve for the given pair k-k of angles j, j+1.
                L_j_j1_k = twoPointsMarkovDubins(conf_j, conf_j1, Kmax).L;

                // For the last iteration (j=1), consider that j=0 already has a fixed angle
                if (j == 1){ L_j0_j_k = twoPointsMarkovDubins(conf_j0, conf_j, Kmax).L; }

                // Evaluate whether this subproblem solution improves the current best
                if (L_j0_j_k + L_j_j1_k + L_j1_j2_k + L_j2_n < L_j_n_k){

                    // Save the current best distance and the corresponding j+1 configuration
                    L_j_n_k = L_j0_j_k + L_j_j1_k + L_j1_j2_k + L_j2_n;
                    opt_conf_j1 = conf_j1;
                    opt_L_j1_j2 = L_j1_j2_k;

                    // Only for the last pair of points, save the j=1 configuration as well
                    if (j == 1){ opt_conf_j = conf_j; }
                }
            }
        }

        // When the j+1 optimal configuration has been fixed, add the distance from j+1 to j+2 to the total distance.
        L_j2_n += opt_L_j1_j2;

        // Add the optimal configurations found to the vector of optimal configurations
        opt_confs.push_back(opt_conf_j1);
        if (j == 1){ opt_confs.push_back(opt_conf_j); }
    }

    // Add the fixed 0th configuration to the solution vector
    opt_confs.push_back(conf_j0);
    
    return opt_confs;
}

// Compute the optimal Dubins path between (at least) four 2D configurations.
// The initial and final configurations must be totally fixed, including orientation.
// The intermediate configurations must be fixed only in x and y.
// The orientations of the intermediate configurations can be set to any number, they are going to be optimized.
std::vector<Pose> refinementMarkovDubins(std::vector<Pose> all_confs, const float Kmax, const unsigned int k, const unsigned int M){

    // Initialize variables for the loop
    std::vector<Pose> opt_confs_m, opt_confs;
    std::vector<Pose> opt_path;

    // Perform refinement
    for (unsigned int m = 1; m < M + 1; m++ ) {
        opt_confs_m = multipointMarkovDubins(all_confs, Kmax, k, m);

        // The result comes as a reversed vector
        for ( unsigned int i = 0; i < opt_confs_m.size(); i++ ){
            all_confs[i] = opt_confs_m[opt_confs_m.size()-i-1];
        }
    }
    opt_confs = opt_confs_m;

    float tot_L = 0;

    // Stack together the Dubins curves in a vector of poses and return
    for ( unsigned int i = 1; i < opt_confs.size(); i++ ){
        Pose conf_i = opt_confs[opt_confs.size()-i];
        Pose conf_i1 = opt_confs[opt_confs.size()-i-1];
        dubinsCurve curve_i = twoPointsMarkovDubins(conf_i, conf_i1, Kmax);
        tot_L += curve_i.L;
        // Transform to Path object
        // Loop over the three Dubins arcs composing the given Dubins curve
        for (unsigned int i=0; i<3; i++){
            dubinsArc arc_i;
            if      (i==0){ arc_i = curve_i.a1; }
            else if (i==1){ arc_i = curve_i.a2; }
            else if (i==2){ arc_i = curve_i.a3; }
            // Compute 100 equally spaced points within the arc (unless the arc is of length 0)
            if (arc_i.L > 0.0){
                for (unsigned int j=0; j<101; j++){
                    Pose p_ij = dubinspoint(arc_i.L*j/100.0, arc_i.p0, arc_i.k);
                    opt_path.push_back(p_ij);
                }
            }
        }
    }
    std::cout << "Dubins k = " << k << ", M = " << M << ", L = " << tot_L << std::endl;
    
    return opt_path;
}

void write_path(std::vector<Pose> opt_path){
    std::ofstream csv_file;
    csv_file.open("dubins.csv");
    csv_file << "x" << "," << "y" << std::endl;

    // Loop over the poses within the input array
    for (unsigned int n=0; n<opt_path.size(); n++){
        csv_file << opt_path[n].x << "," << opt_path[n].y << std::endl;
    }
    csv_file.close();
}