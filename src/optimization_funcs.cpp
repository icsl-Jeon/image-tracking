// 
//
// Created by jbs on 18. 3. 20.
//
#include <optimization_funs.h>
#include "optimization_funs.h"

double clamping(double x)
{
    if(x>=4.5)
        return 4.5;
}



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
    // modular operator
    azim=fmod(azim,2*Pi);
    if (azim<0)
        azim+=2*Pi;

    Eigen::Vector3d view_vector;
    view_vector<<r*cos(elev)*cos(azim), r*cos(elev)*sin(azim) , r*sin(elev);



    /**
     * objective functions (Q_t, Q_d)
     */
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



    /*
     * objective function (Q_v)
    */

    // if SEDT and fitting was over in Optimizer class ...


    double visibility_cost=w_v*(p->optimizer.poly_coeff*p->optimizer.get_X_derivative(azim,elev,0," "))(0);
    Vector2d X(azim,elev);
//    double visibility_cost=w_v*(p->bspline->eval(X));

    if (grad){
        grad[1]+=w_v*(p->optimizer.poly_coeff*p->optimizer.get_X_derivative(azim,elev,1,"azim"))(0);
       // grad[1]+=w_v*(p->bspline->evalJacobian(X))(0);
        grad[2]+=w_v*(p->optimizer.poly_coeff*p->optimizer.get_X_derivative(azim,elev,1,"elev"))(0);
       // grad[2]+=w_v*(p->bspline->evalJacobian(X))(1);

        printf("current r: %f azim %f elev %f ::: ",r,azim,elev);
        //printf( "dQ_v : [%f,%f]  ",visibility_cost_gradient[0],visibility_cost_gradient[1]);

        printf("Q_t: %f Q_d:%f  Q_v: %f\n",translational_cost,tracking_distance_cost,visibility_cost);

    }


    return translational_cost+tracking_distance_cost+visibility_cost;


}


double constraint(unsigned n, const double *x, double *grad, void *param_info){
    // x=[r azim elev]
    param *p=(param *) param_info;
    double r=x[0];

    VectorXd X(2);

    for(int i=1; i<n;i++)
        X(i-1)=x[i];

    double res=r-p->bspline3->eval(X);
    if (grad){

        grad[0]=1;
        grad[1]=-(p->bspline3->evalJacobian(X))(0);
        grad[2]=-(p->bspline3->evalJacobian(X))(1);
    }


    return res;

}



/**
 *  special class for visibility cost
 */

Optimizer::Optimizer() {};
Optimizer::Optimizer(int N_azim, int N_elev, int nx, int ny, double elev_min ,double elev_max)
{


    pi=3.141592;
    this->N_azim=N_azim;
    this->N_elev=N_elev;
    this->nx=nx;
    this->ny=ny;
    this->elev_min=elev_min;
    this->elev_max=elev_max;

    this->azim_set.setLinSpaced(N_azim,0,2*pi);
    this->elev_set.setLinSpaced(N_elev,elev_min,elev_max);

    // mesh generate
    mesh_generate();
    castRayResultBinary.resize(N_elev,N_azim);

    // possible pair of order of poly
    for(int order=0;order<=nx;order++)
        for (int i=order;i>=0;i--)
            if (order-i<=ny)
                order_pair.push_back(*(new Vector2d(i, order - i)));

    // construct trial_A for fitting
    trial_A.resize(mesh_azim.size(),order_pair.size());

}



azim_elev_mesh Optimizer::mesh_generate() {

    MatrixXd azim_mesh_mat(N_elev,N_azim);
    MatrixXd elev_mesh_mat(N_elev,N_azim);

    for (int i=0; i<N_elev ;i++)
        for(int j=0; j<N_azim;j++)
        {
            azim_mesh_mat.coeffRef(i,j)=azim_set[j];
            elev_mesh_mat.coeffRef(i,j)=elev_set[i];

        }
    // reshape columwise
    Map<RowVectorXd> mesh_azim1(azim_mesh_mat.data(), azim_mesh_mat.size());
    Map<RowVectorXd> mesh_elev1(elev_mesh_mat.data(), elev_mesh_mat.size());

    // update
    this->mesh_azim=mesh_azim1;
    this->mesh_elev=mesh_elev1;

    azim_elev_mesh mesh;
    mesh.azim_mesh_mat=azim_mesh_mat;
    mesh.elev_mesh_mat=elev_mesh_mat;

    return mesh;

}


void Optimizer::castRayResultUpdate(MatrixXd & castresult) {
    // matrix update
    this->castRayResultBinary=castresult;

}
// signed distance transform
void Optimizer::SEDT(double query_azim)
{
    //firstly, reshape considering periodicity centering query_azim
    MatrixXd castRayResultBinaryReshaped=periodic_reshape(castRayResultBinary,query_azim);
    std::cout<<"reshaped castRayResult:"<<std::endl;
    std::cout<<castRayResultBinaryReshaped<<std::endl;
    Mat bw_cast(N_azim,N_elev,CV_64F,castRayResultBinaryReshaped.data());
    bw_cast.convertTo(bw_cast,CV_8UC(1));
    transpose(bw_cast,bw_cast);

    // calculate SEDT
    cv::Mat dist,dist1,dist2;
    // in this world 1= white... 0=black... fuck!
    distanceTransform(1-bw_cast, dist1, CV_DIST_L2, 0);
    std::vector<Vec4i> hierarchy;
    std::vector<std::vector<Point> > contours;
    findContours(bw_cast,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,Point(0, 0));
    for( int i = 0; i< contours.size(); i++ ) // for each cluster
        for (int j=0;j<contours[i].size();j++)
            bw_cast.at<uchar>(contours[i][j].y,contours[i][j].x)=0;


    distanceTransform(bw_cast, dist2, CV_DIST_L2, 0);
    dist=dist1-dist2;
    // to eigen
    cv2eigen(dist,SDF);


    SDF=SDF.unaryExpr(std::ptr_fun( clamping));

    //clamping SEDT value for better optimization



    std::cout<<"SEDT"<<std::endl;
    std::cout<<SDF<<std::endl;



    // fitting targets(z) are changed.
    Map<RowVectorXd> SDF_flat(SDF.data(), SDF.size());

    trial_b.resize(mesh_azim.size());
    for(int k=0;k<mesh_azim.size();k++) //for each data sample
        trial_b[k]=-SDF_flat[k];

}

// n=0,1
MatrixXd Optimizer::get_X_derivative(double azim, double elev, int n,std::string wrt) {
    MatrixXd X(order_pair.size(),1); //column matrix
    X.setZero();
    if(n==0) //no derivative
        for (int i=0;i<order_pair.size();i++)
            X.coeffRef(i,0)=pow(azim, order_pair[i][0]) * pow(elev, order_pair[i][1]);

    else {

        if (wrt == "azim") //1st derivative wrt azim
        {
            for (int i = 0; i < order_pair.size(); i++)
                if (order_pair[i][0] > 0)
                    X.coeffRef(i, 0) = order_pair[i][0] * pow(azim, order_pair[i][0] - 1) * pow(elev, order_pair[i][1]);
        }

        else //1st derivative wrt elev
        {
            for (int i = 0; i < order_pair.size(); i++)
                if (order_pair[i][1] > 0)
                    X.coeffRef(i, 0) = pow(azim, order_pair[i][0]) * order_pair[i][1] * pow(elev, order_pair[i][1] - 1);
        }
    }

    return X;
}


MatrixXd Optimizer::col_slice_real_value(double lower_val,double upper_val,MatrixXd mat){
    // slice mat with real valued index
    double D_azim=2*pi/N_azim;
    MatrixXd sub_mat;
    int lower_idx=round(lower_val/D_azim);
    int col_num=mat.cols();
    int upper_idx=round(upper_val/D_azim)-1;
    if (upper_idx>=col_num)
        upper_idx=col_num-1;


    VectorXi slicing_row,slicing_col;
    slicing_row.setLinSpaced(mat.rows(),0,mat.rows());
    slicing_col.setLinSpaced(upper_idx-lower_idx+1,lower_idx,upper_idx);
    igl::slice(mat,slicing_row,slicing_col,sub_mat);
    return sub_mat;
}

// this function produce reshaped castray result binary
MatrixXd Optimizer::periodic_reshape(MatrixXd mat,double query_azim) {
    query_azim=fmod(query_azim,2*Pi);
    if (query_azim<0)
        query_azim+=2*Pi;

    if (query_azim-pi>0)
        return igl::cat(2,col_slice_real_value(query_azim-pi,2*pi,mat),
                        col_slice_real_value(0,query_azim-pi,mat));
    else
        return igl::cat(2,col_slice_real_value(query_azim+pi,2*pi,mat),
                        col_slice_real_value(0,query_azim+pi,mat));
}

// perform fitting to stored data ({azim_set elev_set , SDF_flattened})
void Optimizer::poly_surf_fit(double query_azim) {
    // periodic reshape and fitting the cost
    SEDT(query_azim);
    // independent variable [azim,elev]
    for(int k=0;k<mesh_azim.size();k++) //for each data sample
        for (int i=0;i<order_pair.size();i++)
            trial_A.coeffRef(k,i)=pow(mesh_azim[k]+(query_azim-pi), order_pair[i][0]) * pow(mesh_elev[k], order_pair[i][1]);

    // dependent variable [cost]

    poly_coeff=trial_A.colPivHouseholderQr().solve(trial_b);


};





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
