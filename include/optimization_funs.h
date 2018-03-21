//
// Created by jbs on 18. 3. 19.
//

#ifndef IMAGE_TRACKING_CLION_OPTIMIZATION_FUNS_H
#define IMAGE_TRACKING_CLION_OPTIMIZATION_FUNS_H

#include <iostream>
#include "cmath"
#include <eigen3/Eigen/Dense>
#include <nlopt.hpp>
#define Pi 3.141592

typedef struct {
    int N_azim, N_elev;
    double w_d, w_v,elev_min,elev_max,d_track ;
    Eigen::MatrixXd castResult;
    Eigen::Vector3d target_position;
    Eigen::Vector3d tracker_position;

} param;



/**
 * cost function
 */

double obj_fun(unsigned n, const double *, double *, void *);


/**
 * non linear inequality constraint in the form of piecewise linear
 * c_i w.r.t x_j corresponds to grad[i*n + j]
 */
double nonlcon_PWL(unsigned , const double* , double* , void* );



#endif //IMAGE_TRACKING_CLION_OPTIMIZATION_FUNS_H
