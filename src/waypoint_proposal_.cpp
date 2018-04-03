//
// Created by jbs on 18. 3. 19.
//

#include <waypoint_proposal_.h>
#include <octomap/OcTreeIterator.hxx>
#include <optimization_funs.h>

double kernel_mean(MatrixXd input_mat,int mask_size_row,int mask_size_col,int mask_center_row,int mask_center_col){
    int row=input_mat.rows();
    int col=input_mat.cols();


    double sum=0;
    int count=0;
    for (int col_idx=mask_center_col-mask_size_col;col_idx<=mask_center_col+mask_size_col;col_idx++)
        for (int row_idx=mask_center_row-mask_size_row;row_idx<=mask_center_row+mask_size_row;row_idx++)
            if (row_idx>=0 && row_idx<row && col_idx>=0 && col_idx<col)
            {   count++;
                sum+=input_mat.coeffRef(row_idx,col_idx);
            }


    return sum/count;

}



/**
  Waypoint Proposal class
**/


//constructor
WaypointProposer::WaypointProposer(float elev_min,float elev_max,unsigned int N_azim,unsigned int N_elev,float track_d,
                                   octomap::point3d min_point,octomap::point3d max_point,ros::NodeHandle private_nh,Optimizer optimizer)
{
    this->octree_obj=new OcTree(0.1); //fake initialization
    this->tracking_distance=track_d; //desired tracking distance
    this->elev_min=elev_min;
    this->elev_max=elev_max;
    this->N_azim=N_azim;
    this->N_elev=N_elev;
    this->optimizer=optimizer;

    // hovering first
    trajectory_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
            Eigen::Vector3d( 0,0,1.0),0, &trajectory_msg);



    // cast result matrix
    this->castResult=MatrixXd::Constant(N_elev,N_azim,track_d);

    // node params
    this->private_nh=private_nh;
    private_nh.getParam("target_name",this->targetName);
    private_nh.getParam("tracker_name",this->trackerName);
    private_nh.getParam("w_v",param_.w_v);
    private_nh.getParam("w_d",param_.w_d);

    ROS_INFO_STREAM("target name: "<<this->trackerName<<" / tracker name: "<<this->targetName);

    state_callback_flag=octomap_callback_flag= false;

    ros::NodeHandle nh;

    //rviz publisher
    this->marker_pub=nh.advertise<visualization_msgs::Marker>("proposed_waypoint", 4);
    this->boundingCube_pub=nh.advertise<visualization_msgs::Marker>("target_bounding_box",4);

    //waypoint publisher
    this->trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                    "/"+trackerName+"/"+mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

    // octree subscribe
    this->Octbin_sub=nh.subscribe("/octomap_full",3,&WaypointProposer::OctreeCallback,this);


    // states subscribe
    this->states_sub=nh.subscribe("/gazebo/model_states",10,&WaypointProposer::StateCallback,this);


    // test query
    /**
    this->server_query = nh.advertiseService("cast_query", &WaypointProposer::QueryfromTarget,this);
    this->server_debug = nh.advertiseService("octomap_leaf_debug", &WaypointProposer::OctreeDebug,this);
    **/

    // free the box around target
    this->freebox_max_point=max_point;
    this->freebox_min_point=min_point;

    // these are used in ray casting iterations

    azim_set.setLinSpaced(N_azim,0,2*Pi);
    elev_set.setLinSpaced(N_elev,elev_min,elev_max);



    //std::cout<<"azim_size: "<<azim_set.size()<<" elev_size: "<<elev_set.size()<<std::endl;


    //casted lights for query
    waypointMarker.header.frame_id = "world";
    waypointMarker.header.stamp  = ros::Time::now();
    waypointMarker.ns = "proposed_waypoint";
    waypointMarker.action = visualization_msgs::Marker::ADD;
    waypointMarker.pose.orientation.w = 1.0;
    waypointMarker.id = 0;
    waypointMarker.type = visualization_msgs::Marker::LINE_LIST;
    waypointMarker.scale.x = 0.05;
    waypointMarker.color.r = 1;
    waypointMarker.color.a = 0.5;



    // Bounded Box around target
    BBMarker.header.frame_id="world";
    BBMarker.header.stamp=ros::Time::now();
    BBMarker.ns="targetBB";
    BBMarker.action=visualization_msgs::Marker::ADD;
    BBMarker.id=0;
    BBMarker.type=visualization_msgs::Marker::CUBE;

    BBMarker.pose.orientation.x = 0.0;
    BBMarker.pose.orientation.y = 0.0;
    BBMarker.pose.orientation.z = 0.0;
    BBMarker.pose.orientation.w = 1.0;

    BBMarker.scale.x = this->freebox_max_point.x()-this->freebox_min_point.x();
    BBMarker.scale.y = BBMarker.scale.x;
    BBMarker.scale.z=this->freebox_max_point.z()-this->freebox_min_point.z();

    BBMarker.color.r=1.0;
    BBMarker.color.a=0.2;



    // optimization params
    param_.N_azim=N_azim;
    param_.N_elev=N_elev;
    param_.elev_min=elev_min; param_.elev_max=elev_max;
    param_.d_track=tracking_distance;
    param_.castResult=castResult;
    param_.target_position=*(new Eigen::Vector3d(3,0,1));
    param_.tracker_position=*(new Eigen::Vector3d(3,1,0));

    // logging
    ROS_INFO("waypoint proposer constructor success!");


}


// destructor
WaypointProposer::~WaypointProposer() {
    delete octree_obj;
}

// Callback functions

void WaypointProposer::StateCallback(const gazebo_msgs::ModelStates::ConstPtr &gazebo_msg) {
    state_callback_flag=true;
    std::vector<std::string> model_names=gazebo_msg->name;
    std::vector<geometry_msgs::Pose> pose_vector=gazebo_msg->pose;


    int target_idx=std::find(model_names.begin(),model_names.end(),this->targetName)-model_names.begin();
    int tracker_idx=std::find(model_names.begin(),model_names.end(),this->trackerName)-model_names.begin();


    //extract target state
    if (target_idx<model_names.size())
        targetPose=pose_vector[target_idx];
    else
        ROS_WARN("specified target name was not found in gazebo");

    //update for visualize
    BBMarker.pose=targetPose;
    BBMarker.header.stamp=ros::Time::now();

    //extract tracker state
    if (tracker_idx<model_names.size())
        trackerPose=pose_vector[tracker_idx];
    else
        ROS_WARN("specified tracker name was not found in gazebo");

}

void WaypointProposer::OctreeCallback(const octomap_msgs::Octomap& msg){
    octomap_callback_flag=true;
    ROS_INFO("octree callback");

    AbstractOcTree* octree=octomap_msgs::fullMsgToMap(msg);
    //octree updater
    this->octree_obj=(dynamic_cast<OcTree*>(octree));
}


/** this function cast Rays from a given light source **/

void WaypointProposer::castRay(geometry_msgs::Point rayStartPnt,bool verbose
    ,double azim_cur,double elev_cur) {




    // initialize the elements

    if (octree_obj->size()) {

        bool ignoreUnknownCells = true;

        point3d light_start(float(rayStartPnt.x),float(rayStartPnt.y),float(rayStartPnt.z));
       //free octomap around target

        double thresMin = octree_obj->getClampingThresMin();


        for (OcTree::leaf_bbx_iterator it = octree_obj->begin_leafs_bbx(freebox_min_point + light_start,
                                                                        freebox_max_point + light_start),
                     end = octree_obj->end_leafs_bbx(); it != end; ++it)
            it->setLogOdds(octomap::logodds(thresMin));

        octree_obj->updateInnerOccupancy();
        // ray casting & update castResult and castResultBinary
        for (unsigned int ind_elev = 0; ind_elev < N_elev; ind_elev++) {

            for (unsigned int ind_azim = 0; ind_azim < N_azim; ind_azim++) {
                point3d light_end; //endpoint of ray
                point3d light_dir(float (tracking_distance *cos(elev_set[ind_elev])*cos(azim_set[ind_azim])),
                                  float(tracking_distance * cos(elev_set[ind_elev]) * sin(azim_set[ind_azim])),
                                  float(tracking_distance * sin(elev_set[ind_elev])));


                if(octree_obj->castRay(light_start, light_dir, light_end,
                                       ignoreUnknownCells, tracking_distance)) {
                    // hit distance - safty margin
                    this->castResult.coeffRef(ind_elev, ind_azim) = light_end.distance(light_start)-0.2;
                    this->optimizer.castRayResultBinary.coeffRef(ind_elev,ind_azim)=1;
                }
                else {
                    // no hit = just tracking distance
                    this->castResult.coeffRef(ind_elev, ind_azim) = tracking_distance;
                    this->optimizer.castRayResultBinary.coeffRef(ind_elev,ind_azim)=0;
                }
            }

        }



        // desired distance
        double D_azim=2*PI/N_azim;
        double D_elev=(elev_max-elev_min)/N_elev;

        int azim_idx=floor(azim_cur/D_azim);
        int elev_idx=floor((elev_cur-elev_min)/D_elev);

        int azim_kernel_size=1;
        int elev_kernel_size=1;

        double raycut_distance=kernel_mean(castResult,elev_kernel_size,azim_kernel_size,elev_idx,azim_idx);

        param_.d_track=raycut_distance;
        std::cout<<"desired distance: "<<param_.d_track<<std::endl;

        // print the cast result
        if (verbose)
            std::cout<<this->castResult<<std::endl;


    }// if octree is not empty

    else
        ROS_WARN("Octree has zero size");

}

/** using the castResult in this class, optimization process **/

void  WaypointProposer::viewProposal(){


    ROS_INFO("casted Rays from target");



    //parametrization of positions
    param_.target_position[0]=targetPose.position.x;
    param_.target_position[1]=targetPose.position.y;
    param_.target_position[2]=targetPose.position.z;
    /**
    printf("current target: [%f, %f , %f]\n",
           targetPose.position.x,
           targetPose.position.y,
           targetPose.position.z);
    **/

    param_.tracker_position[0]=trackerPose.position.x;
    param_.tracker_position[1]=trackerPose.position.y;
    param_.tracker_position[2]=trackerPose.position.z;

    //update cast Result
    param_.castResult=this->castResult;

    /**
    printf("current tracker: [%f, %f , %f]\n",
           trackerPose.position.x,
           trackerPose.position.y,
           trackerPose.position.z);
    **/

    double elev_cur=atan2(param_.target_position[2]-param_.tracker_position[2],
                          sqrt(pow(param_.target_position[0]-param_.tracker_position[0],2)+
                               pow(param_.target_position[1]-param_.tracker_position[1],2)));

    // clamp the elev_cur for warm start
    if (elev_cur<elev_min)
        elev_cur=elev_min+1e-6;
    if (elev_cur>elev_max)
        elev_cur=elev_max-1e-6;

    double azim_cur=atan2(-param_.target_position[1]+param_.tracker_position[1],
                          -param_.target_position[0]+param_.tracker_position[0]);
    if (azim_cur<0)
        azim_cur+=2*Pi;

    double r_cur=(param_.target_position-param_.tracker_position).norm();

    // cast ray and update castResult matrix
    castRay(targetPose.position,true,azim_cur,elev_cur);

    /**
 *  perform fitting for visibility cost  !!
 */
    optimizer.poly_surf_fit(azim_cur);
    param_.optimizer=this->optimizer;


    // b spline surface fitting

    azim_elev_mesh xy_mesh=optimizer.mesh_generate();


//    std::cout<<"mesh check: "<<std::endl;
//    std::cout<<xy_mesh.azim_mesh_mat<<std::endl;
//    std::cout<<xy_mesh.elev_mesh_mat<<std::endl;
//

    MatrixXd castResultReshaped=optimizer.periodic_reshape(
            castResult,azim_cur);

    DataTable samples;

    Eigen::VectorXd X(2);
    double y;

    for(int i = 0; i < N_elev; i++)
    {
        for(int j = 0; j < N_azim; j++)
        {

            X(0) = xy_mesh.azim_mesh_mat.coeff(i,j) + (azim_cur-Pi)  ;
            X(1) = xy_mesh.elev_mesh_mat.coeff(i,j);

            y = castResultReshaped.coeff(i,j);
            // Store sample
            samples.addSample(X,y);
        }
    }

    BSpline bspline3 = BSpline::Builder(samples).degree(3).build();

    param_.bspline3=&bspline3;



    if (r_cur<=3)
        r_cur=3+1e-3;
    if(r_cur>=6)
        r_cur=6-1e-3;

    printf("------------------------------- \n");
    printf("initial r: %f azim %f elev %f\n",r_cur,azim_cur,elev_cur);



    // initial value setting
    std::vector<double> x;
    x.push_back(r_cur);
    x.push_back(azim_cur);
    x.push_back(elev_cur);
    std::vector<double> cur_x=x;
    std::vector<double> min_x=x;

    double min_cost=1000;

    // lower bound
    std::vector<double> lb;
    lb.push_back(3);
    lb.push_back(azim_cur-PI+2*Pi/N_azim);
    lb.push_back(param_.elev_min);
    // upper bound
    std::vector<double> ub;
    ub.push_back(6);
    ub.push_back(azim_cur+PI-2*Pi/N_azim);
    ub.push_back(param_.elev_max);

    nlopt::opt opt(nlopt::LD_MMA,3);

    std::vector<double> step_sizes;
    double step_size=1e-6;
    step_sizes.push_back(step_size);
    step_sizes.push_back(step_size);
    step_sizes.push_back(step_size);


    opt.set_initial_step(step_sizes);
    opt.set_min_objective(obj_fun,&param_);
    opt.add_inequality_constraint(constraint,&param_);
    opt.set_upper_bounds(ub);
    opt.set_lower_bounds(lb);
    opt.set_xtol_rel(1e-2);
    auto begin = std::chrono::high_resolution_clock::now();
    double minf;

    Eigen::Vector3d rand_scale(0.1,Pi/16,Pi/10);

//    int count=0;
//    while(count<10) {
//        count++;
//        //perturb little bit
//        for (int i=0;i<3;i++) {
//            x[i] = cur_x[i]+(-rand_scale[i] + static_cast <double> (std::rand()) /( static_cast <double> (RAND_MAX/(2*rand_scale[i]))));
//            if(x[i]<lb[i])
//                x[i]=lb[i]+1e-4;
//            if(x[i]>ub[i])
//                x[i]=ub[i]-1e-4;
//        }
//
//
//        int result = opt.optimize(x, minf);
//
//        if(min_cost>minf)
//        {
//            min_cost=minf;
//            min_x=x;
//        }
//
//        ROS_INFO("optimization completed");
//
//        if (result < 0) {
//            printf("nlopt failed! reason:%d", result);
//        } else {
//            auto end = std::chrono::high_resolution_clock::now();
//            std::chrono::duration<double> elapsed = (end - begin);
//            printf("found minimum at f(%g,%g,%g) = %0.10g\n", x[0], x[1], x[2], minf);
//        }
//    }
//
    int result = opt.optimize(x, minf);

    printf("found minimum at f(%g,%g,%g) = %0.10g\n", x[0], x[1], x[2], minf);

    min_x=x;
    desired_pose.centerPnt=targetPose.position;
    desired_pose.ray_length=min_x[0];
    min_x[1]=fmod(min_x[1],2*Pi);
    if (min_x[1]<0)
        min_x[1]+=2*Pi;
    desired_pose.azim=min_x[1];
    desired_pose.elev=min_x[2];

    trajectory_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
            Eigen::Vector3d( desired_pose.getPoseFromViewVector().translation.x,
                             desired_pose.getPoseFromViewVector().translation.y,
                             desired_pose.getPoseFromViewVector().translation.z)
            ,desired_pose.getYaw(), &trajectory_msg);

    // for visualization

    waypointMarker.points.clear();
    waypointMarker.points.push_back(desired_pose.centerPnt);
    waypointMarker.points.push_back(desired_pose.getEndPnt());
    waypointMarker.header.stamp=ros::Time::now();

}


void WaypointProposer::marker_publish(){
    // waypoint visualization
    if (waypointMarker.points.size())
        marker_pub.publish(waypointMarker);

    this->boundingCube_pub.publish(this->BBMarker);

}

void WaypointProposer::waypoint_publish(){

    trajectory_pub.publish(trajectory_msg);

}

// query (for debug)
//
//bool WaypointProposer::QueryfromTarget( image_tracking::CastQuery::Request &req, image_tracking::CastQuery::Response &resp){
//
//    castedLightMarker.points.clear();
//    yawingMarkerList.clear();
//
//
//    geometry_msgs::Point point;
//    point.x=(req.query_pose.x); point.y=(req.query_pose.y); point.z=(req.query_pose.z);
//
//
//    ProposedView PV=viewProposal(point,true);
//
//    int id_count=0;
//    if (PV.ProposedBoxes.size()) // if it has proposed view
//        for (std::vector<Box>::iterator it=PV.ProposedBoxes.begin();it!=PV.ProposedBoxes.end();it++)
//        {
//
//            geometry_msgs::Transform proposedViewPose=getPoseFromViewProposal(point,tracking_distance,*it);
//
//            castedLightMarker.points.push_back(point);
//
//
//
//            geometry_msgs::Point endPoint;
//            endPoint.x=proposedViewPose.translation.x;
//            endPoint.y=proposedViewPose.translation.y;
//            endPoint.z=proposedViewPose.translation.z;
//
//
//
//
//
//            //yawing angle arrow prototype
//
//            yawingMarker.header.frame_id = "world";
//            yawingMarker.header.stamp  = ros::Time::now();
//            yawingMarker.ns = "yawing_arrow";
//            yawingMarker.action = visualization_msgs::Marker::ADD;
//            yawingMarker.id=id_count;
//
//            yawingMarker.type = visualization_msgs::Marker::ARROW;
//            yawingMarker.scale.x = 1;
//            yawingMarker.scale.y = 0.05;
//            yawingMarker.scale.z = 0.1;
//            yawingMarker.color.r = 1;
//            yawingMarker.color.a = 0.5;
//
//
//            yawingMarker.pose.position=endPoint;
//            yawingMarker.pose.orientation=proposedViewPose.rotation;
//
//            yawingMarkerList.push_back((yawingMarker));
//
//            castedLightMarker.points.push_back(endPoint);
//            id_count++;
//
//        }
//
//
//    return true;
//
//
//    // castRay from light source to all around in Rviz
//
//    /**
//    for (std::vector<double>::iterator it_elev = elev_set.begin() ; it_elev != elev_set.end(); ++it_elev)
//    {
//        int ind_azim=0;
//        for (std::vector<double>::iterator it_azim = azim_set.begin() ; it_azim != azim_set.end(); ++it_azim)
//        {
//              point3d light_dir(tracking_distance*cos(*it_elev)*cos(*it_azim),
//                tracking_distance*cos(*it_elev)*sin(*it_azim),
//                tracking_distance*sin(*it_elev));
//
//            // if the ray is not obstruded by voxel.
//            if(!octree_obj->castRay(light_start,light_dir,light_end,ignoreUnknownCells,5.0))
//            {
//                // vizualization marker
//                geometry_msgs::Point p;
//
//                p.x=light_start.x();
//                p.y=light_start.y();
//                p.z=light_start.z();
//
//                castedLightMarker.points.push_back(p);
//
//                p.x=light_start.x()+light_dir.x();
//                p.y=light_start.y()+light_dir.y();
//                p.z=light_start.z()+light_dir.z();
//
//
//                castedLightMarker.points.push_back(p);
//            }
//
//        }
//    }
//    **/
//
//}



/**
bool WaypointProposer::OctreeDebug(image_tracking::Debug::Request &req, image_tracking::Debug::Response &res){

    ROS_INFO("octree size: %f\n",this->octree_obj->size());

    for(OcTree::leaf_iterator it = this->octree_obj->begin_leafs(),end_it=this->octree_obj->end_leafs(); it!=end_it; ++it)
    {
        float p= exp(it->getValue())/(exp(it->getValue())+1);

        ROS_INFO_STREAM("Node center"<<it.getCoordinate()<<" Depth: "<<it.getDepth()<<" Size: "<<it.getSize()
                                     <<" p: "<<p<<"\n");

    }
    return true;
}
 **/






