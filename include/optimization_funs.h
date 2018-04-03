//
// Created by jbs on 18. 3. 19.
//

#ifndef IMAGE_TRACKING_CLION_OPTIMIZATION_FUNS_H
#define IMAGE_TRACKING_CLION_OPTIMIZATION_FUNS_H

#include <iostream>
#include "cmath"
#include <eigen3/Eigen/Dense>
#include <nlopt.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <igl/slice.h>  //really convient
#include <igl/cat.h>

// B spline surface

#include <SPLINTER/datatable.h>
#include <SPLINTER/bspline.h>
#include <SPLINTER/bsplinebuilder.h>

#define Pi 3.141592



using namespace cv;
using namespace Eigen;
struct azim_elev_mesh{

    MatrixXd azim_mesh_mat;
    MatrixXd elev_mesh_mat;

    void print_mesh(){

        std::cout<<"azim_mesh: "<<std::endl;
        std::cout<<azim_mesh_mat<<std::endl;
        std::cout<<"elev_mesh: "<<std::endl;
        std::cout<<elev_mesh_mat<<std::endl;

    }
};

// this is for buffer calculation
//struct Filter_Buffer{
//
//    int N_buffer; //buffer number
//    std::vector<std::vector> Buffer;
//    std::vector<double> Residual;
//    std::vector<double> Weights;
//
//
//    Filter_Buffer(int N){
//
//        Buffer.reserve(N);
//        Residual.reserve(N);
//        Weights.reserve(N);
//
//    }
//
//    void buffer_update(std::vector new_point){
//
//
//
//    }






/**
 * cost function
 */

double obj_fun(unsigned n, const double *, double *, void *);
double constraint(unsigned n, const double *, double *, void *);

/**
 * non linear inequality constraint in the form of piecewise linear
 * c_i w.r.t x_j corresponds to grad[i*n + j]
 */


/**
 * special class for visibility cost
 */

class Optimizer {
public:
    Optimizer(); //hrrr...
    Optimizer(int,int,int,int,double,double); //int N_azim and N_elev / nx,ny / elev_range
    azim_elev_mesh mesh_generate(); // this is for surf fit and surf drawing
    MatrixXd periodic_reshape(MatrixXd, double);
    MatrixXd col_slice_real_value(double,double,MatrixXd); //column-wise slicing with real value
    RowVectorXd poly_coeff; //
    MatrixXd castRayResultBinary;

    double pi;

    void castRayResultUpdate(MatrixXd& ); // update the matrix
    void poly_surf_fit(double ); // poly surf fit
    void SEDT(double); // SEDT to given matrix

    // get n th derivative of vector X=[azim^i * elev^j]_(i=0...,j=0...)
    MatrixXd get_X_derivative(double,double,int,std::string);

private:
    //optimization quadratic function
    MatrixXd SDF; //signed distance transform field / N_elev x N_azim
    MatrixXd trial_A; // used in surface fitting
    VectorXd trial_b; // also
    int nx,ny; // order of x , y in polynomial surface
    int N_azim,N_elev;
    double elev_min,elev_max;
    VectorXd mesh_azim; // (N_azim*N_elev) x 1
    VectorXd mesh_elev; // same dim
    VectorXd azim_set; // (N_azim*N_elev) x 1
    VectorXd elev_set; // same dim
    std::vector<Vector2d> order_pair;  // this is possible combination of powers
};

using namespace SPLINTER;

struct param{


    int N_azim, N_elev;
    double w_d, w_v,elev_min,elev_max,d_track ;
    MatrixXd castResult;
    Vector3d target_position;
    Vector3d tracker_position;
    Optimizer optimizer; // need for visibility cost computation

};




#endif //IMAGE_TRACKING_CLION_OPTIMIZATION_FUNS_H
