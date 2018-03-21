// 
//
// Created by jbs on 18. 3. 20.
//
#include "optimization_funs.h"

double obj_fun(unsigned n, const double *x, double *grad, void *param_info)
{
    // parsing
    param *p=(param *) param_info;
    double w_d=p->w_d, w_v=p->w_v, elev_min=p->elev_min, elev_max=p->elev_max;
    double d_track=p->d_track;
    int N_azim=p->N_azim, N_elev=p->N_elev;
    //variable
    Eigen::MatrixXd castResult=p->castResult;
    Eigen::Vector3d target_position=p->target_position;
    Eigen::Vector3d tracker_position=p->tracker_position;


    double r=x[0],azim=x[1], elev=x[2];
    printf("------------------------------- \n");
    printf("current r: %f azim %f elev %f\n",r,azim,elev);
    // modular operator
    azim=fmod(azim,2*Pi);
    if (azim<0)
        azim+=2*Pi;

    Eigen::Vector3d view_vector;
    view_vector<<r*cos(elev)*cos(azim), r*cos(elev)*sin(azim) , r*sin(elev);



    // objective functions (Q_t, Q_d)
    double translational_cost=pow((target_position-tracker_position+view_vector).norm(),2);
    double tracking_distance_cost=w_d*pow(r-d_track,2);

    // gradients
    // w.r.t r
    if (grad) {
        Eigen::Vector3d v1(cos(elev) * cos(azim), cos(elev) * sin(azim), sin(elev));
        Eigen::Vector3d v2(r * cos(elev) * cos(azim), r * cos(elev) * sin(azim), r * sin(elev));
        v2 = v2 + target_position - tracker_position;

        grad[0] = 2 * v1.dot(v2) + w_d * 2 * (r - d_track);
        // w.r.t azim
        Eigen::Vector3d v3(-r * cos(elev) * sin(azim), r * cos(elev) * cos(azim), 0.0);
        grad[1] = 2 * v3.dot(v2);
        // w.r.t elev
        Eigen::Vector3d v4(-r * sin(elev) * cos(azim), -r * sin(elev) * sin(azim), r * cos(elev));
        grad[2] = 2 * v4.dot(v2);
    }
    // iterate over castResult
    Eigen::VectorXd azim_set; azim_set.setLinSpaced(N_azim,0,2*Pi);
    Eigen::VectorXd elev_set; elev_set.setLinSpaced(N_elev,elev_min,elev_max);

    double norm_length_azim=2*Pi, norm_length_elev=elev_max-elev_min;

    Eigen::Vector2d query_azim_elev_pair_normalized(azim/norm_length_azim,
                                                    (elev-elev_min)/norm_length_elev);

    double visibility_cost=0;
    double decaying_factor=0.1; //bigger = smoother
    // we translate the potential field along [-1 0 1] to consider periodic effect
    Eigen::Vector3d field_translate(-1,0,1);

    for (int translate_idx=0;translate_idx<3;translate_idx++)
    for (int azim_idx=0;azim_idx<N_azim;azim_idx++)
        for (int elev_idx=0; elev_idx<N_elev;elev_idx++)
        {

            double cur_azim=azim_set[azim_idx]+2*Pi*field_translate[translate_idx],cur_elev=elev_set[elev_idx];
            cur_azim/=norm_length_azim; cur_elev=(cur_elev-elev_min)/norm_length_elev;
            Eigen::Vector2d sampled_azim_elev_pair_normalized(cur_azim,cur_elev);

            double dist=(sampled_azim_elev_pair_normalized-query_azim_elev_pair_normalized).norm();
            if (dist<1e-6)
                std::cout<<"caution: distance become zero"<<std::endl;
            double cur_visibility_cost=w_v*exp(-dist/decaying_factor)
                                       *pow(d_track-castResult.coeff(elev_idx,azim_idx),2);

            visibility_cost+=cur_visibility_cost;
            if(grad) {
                grad[1] += cur_visibility_cost / (-decaying_factor) * (query_azim_elev_pair_normalized[0] - cur_azim) /
                           dist / norm_length_azim;
                grad[2] += cur_visibility_cost / (-decaying_factor) * (query_azim_elev_pair_normalized[1] - cur_elev) /
                           dist / norm_length_elev;
            }

        }


    printf( "transitional cost : %f \n",translational_cost);
    printf( "tracking distance cost : %f\n",tracking_distance_cost);
    printf( "visibility cost : %f\n",visibility_cost);
    printf("total cost: %f\n",translational_cost+tracking_distance_cost+visibility_cost);
    printf("------------------------------- \n");


    return translational_cost+tracking_distance_cost+visibility_cost;



}


/**
 * non linear inequality constraint in the form of piecewise linear
 * c_i w.r.t x_j corresponds to grad[i*n + j]
 */
double nonlcon_PWL(unsigned n, const double* x, double* grad, void* param_info)
{

    param *p=(param *) param_info;
    double w_d=p->w_d, w_v=p->w_v, elev_min=p->elev_min, elev_max=p->elev_max;
    double d_track=p->d_track;
    int N_azim=p->N_azim, N_elev=p->N_elev;
    Eigen::MatrixXd castResult=p->castResult;

    double D_azim=(2*Pi)/double(N_azim-1);
    double D_elev=(elev_max-elev_min)/double(N_elev-1);

    // iterate over castResult
    Eigen::VectorXd azim_set; azim_set.setLinSpaced(N_azim,0,2*Pi);
    Eigen::VectorXd elev_set; elev_set.setLinSpaced(N_elev,elev_min,elev_max);

    double azim=x[1], elev=x[2];
    azim=fmod(azim,2*Pi);
    if (azim<0)
        azim+=2*Pi;
    std::cout<<"current azim: "<<azim<<" elev: "<<elev<<std::endl;

    int azim_lower_idx=floor(azim/D_azim);
    if (azim_lower_idx<0)
        azim_lower_idx=0;
    if(azim_lower_idx>=N_azim-1)
        azim_lower_idx=N_azim-2;
    int azim_upper_idx=azim_lower_idx+1;


    int elev_lower_idx=floor((elev-elev_min)/D_elev);

    if (elev_lower_idx<0)
        elev_lower_idx=0;
    if(elev_lower_idx>=N_elev-1)
        elev_lower_idx=N_elev-2;
    int elev_upper_idx=elev_lower_idx+1;

    std::cout<<"current lower index azim: "<<azim_lower_idx<<" elev: "<<elev_lower_idx<<std::endl;


    double azim_lower=azim_set[azim_lower_idx], azim_upper=azim_set[azim_upper_idx];
    double elev_lower=elev_set[elev_lower_idx], elev_upper=elev_set[elev_upper_idx];

    double safe_margin=0.5;
    if (elev <elev_lower+(azim-azim_lower)*D_elev/D_azim) // lower triangle
    {
        Eigen::Vector3d b(castResult.coeff(elev_lower_idx,azim_lower_idx)==d_track ?
                          castResult.coeff(elev_lower_idx,azim_lower_idx) :std::max(castResult.coeff(elev_lower_idx,azim_lower_idx) - safe_margin,0.0),

                          castResult.coeff(elev_lower_idx,azim_upper_idx)==d_track ?
                          castResult.coeff(elev_lower_idx,azim_upper_idx) :std::max(castResult.coeff(elev_lower_idx,azim_upper_idx) - safe_margin,0.0),

                          castResult.coeff(elev_upper_idx,azim_upper_idx)==d_track ?
                          castResult.coeff(elev_upper_idx,azim_upper_idx) :std::max(castResult.coeff(elev_upper_idx,azim_upper_idx) - safe_margin,0.0));
        Eigen::Matrix3d A;
        A<<azim_lower, elev_lower, 1,
                azim_upper,elev_lower, 1,
                azim_upper, elev_upper, 1;

        // z=ax + by + c
        Eigen::Vector3d plane_coeff=A.inverse()*b;
        if (grad) {
            grad[0] = 1;
            grad[1] = -plane_coeff[0];
            grad[2] = -plane_coeff[1];
        }
        return x[0]-plane_coeff[0]*x[1]-plane_coeff[1]*x[2]-plane_coeff[2];

    }
    else
    {

        Eigen::Vector3d b(castResult.coeff(elev_lower_idx,azim_lower_idx)==d_track ?
                          castResult.coeff(elev_lower_idx,azim_lower_idx) :std::max(castResult.coeff(elev_lower_idx,azim_lower_idx) - safe_margin,0.0),

                          castResult.coeff(elev_upper_idx,azim_lower_idx)==d_track ?
                          castResult.coeff(elev_upper_idx,azim_lower_idx) :std::max(castResult.coeff(elev_upper_idx,azim_lower_idx) - safe_margin,0.0),

                          castResult.coeff(elev_upper_idx,azim_upper_idx)==d_track ?
                          castResult.coeff(elev_upper_idx,azim_upper_idx) :std::max(castResult.coeff(elev_upper_idx,azim_upper_idx) - safe_margin,0.0));

        Eigen::Matrix3d A;
        A<<azim_lower, elev_lower, 1,
                azim_lower,elev_upper, 1,
                azim_upper, elev_upper, 1;

        // z=ax + by + c
        Eigen::Vector3d plane_coeff=A.inverse()*b;
        if(grad) {
            grad[0] = 1;
            grad[1] = -plane_coeff[0];
            grad[2] = -plane_coeff[1];
        }
        return x[0]-plane_coeff[0]*x[1]-plane_coeff[1]*x[2]-plane_coeff[2];

    }

}

/**  example usage
int main() {
    using Eigen::MatrixXd;
    int N_azim=8,N_elev=4;
    MatrixXd castResult(N_elev,N_azim);
    castResult << 4,4,4,4,4,4,4,4,
            4,4,2,1,1,2,4,4,
            4,4,3,2,1,2,4,4,
            4,4,4,4,4,4,4,4;

    param param_;
    param_.N_azim=N_azim;
    param_.N_elev=N_elev;
    param_.w_d=5; param_.w_v=1;
    param_.elev_min=Pi/8.0; param_.elev_max=Pi/3.0;
    param_.d_track=4.0;
    param_.castResult=castResult;
    param_.target_position=*(new Eigen::Vector3d(3,0,1));
    param_.tracker_position=*(new Eigen::Vector3d(3,1,0));

    double elev_cur=atan2(param_.target_position[2]-param_.tracker_position[2],
                          sqrt(pow(param_.target_position[0]-param_.tracker_position[0],2)+
                               pow(param_.target_position[1]-param_.tracker_position[1],2)));


    double azim_cur=atan2(param_.target_position[1]-param_.tracker_position[1],
                          param_.target_position[0]-param_.tracker_position[0]);

    double r_cur=(param_.target_position-param_.tracker_position).norm();
    if (azim_cur<0)
        azim_cur+=2*Pi;

    std::vector<double> x;
    x.push_back(r_cur);
    x.push_back(azim_cur);
    x.push_back(elev_cur);

    std::vector<double> lb;
    lb.push_back(0);
    lb.push_back(-HUGE_VAL);
    lb.push_back(param_.elev_min);


    std::vector<double> ub;
    ub.push_back(+HUGE_VAL);
    ub.push_back(+HUGE_VAL);
    ub.push_back(+param_.elev_max);

    nlopt::opt opt(nlopt::LD_MMA,3);

    opt.set_min_objective(obj_fun,&param_);
    opt.set_upper_bounds(ub);
    opt.set_lower_bounds(lb);
    opt.add_inequality_constraint(nonlcon_PWL,&param_,1e-8);
    opt.set_xtol_rel(1e-3);
    clock_t begin = clock();
    double minf=1;

    if (opt.optimize(x,minf) < 0) {

        printf("nlopt failed!" );
    }
    else {
        clock_t end = clock();
        auto elapsed_secs = double(end - begin);
        printf("found minimum at f(%g,%g,%g) = %0.10g\n / elapsed time : %f", x[0], x[1],x[2], minf,elapsed_secs);
    }

}
**/
