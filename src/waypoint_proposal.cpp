#include <waypoint_proposal.h>
#include <octomap/OcTreeIterator.hxx>

/**
  CastResult class
**/

//constructor
CastResult::CastResult(int row,int col)
{
    //initialization with 0
    for(int i=0;i<row;i++)
    {
        std::vector<int> row_array;
        for(int j=0;j<col;j++)
            row_array.push_back(0);
        mat.push_back(row_array);
    }

}

void CastResult::printResult(){

    /**
        print out the cast space with clusterBB
    **/
    intMatrix intmat;
    int row=mat.size();

    int col=mat[0].size();


    // first, assign
    for (int i=0; i<row;i++)
    {
        std::vector<int> row_vector;
        for (int j=0; j<col;j++)
            row_vector.push_back(mat[i][j]);
        intmat.push_back(row_vector);
    }



    // for each cluster, save bounding box
//    for(std::vector<Box>::iterator it=clusterBB.begin();it!=clusterBB.end();it++)
//    {

//        // horizontal two lines
//        for(int col= it->lower_left_y;col <=it->upper_right_y;col++)
//        {
//            intmat[it->upper_right_x][col]=BB;
//            intmat[it->lower_left_x][col]=BB;
//        }
//        //vertical two lines
//        for(int row= it->upper_right_x;row <=it->lower_left_x;row++)
//        {
//            intmat[row][it->lower_left_y]=BB;
//            intmat[row][it->upper_right_y]=BB;
//        }
//    }

    for (int ind_elev=0;ind_elev!=row;ind_elev++)
    {
        for (int ind_azim=0;ind_azim!=col;ind_azim++)

            if (intmat[ind_elev][ind_azim]==2) //if its is BB
                std::cout<<"* ";
            else
                std::cout<<intmat[ind_elev][ind_azim]<<" ";

        std::cout<<"\n";

    }

}


/**
    ProposedView
**/


void ProposedView::printProposedView(const CastResult & castresult){

    intMatrix intmat=castresult.mat;

    int row=intmat.size();

    int col=intmat[0].size();

    // for each cluster, save bounding box
    for(std::vector<Box>::iterator it=ProposedBoxes.begin();it!=ProposedBoxes.end();it++)
    {

        // horizontal two lines
        for(int col= it->lower_left_y;col <=it->upper_right_y;col++)
        {
            intmat[it->upper_right_x][col]=BB;
            intmat[it->lower_left_x][col]=BB;
        }
        //vertical two lines
        for(int row= it->upper_right_x;row <=it->lower_left_x;row++)
        {
            intmat[row][it->lower_left_y]=BB;
            intmat[row][it->upper_right_y]=BB;
        }
    }

    for (int ind_elev=0;ind_elev!=row;ind_elev++)
    {
        for (int ind_azim=0;ind_azim!=col;ind_azim++)

            if (intmat[ind_elev][ind_azim]==2) //if its is BB
                std::cout<<"* ";
            else
                std::cout<<intmat[ind_elev][ind_azim]<<" ";

        std::cout<<"\n";

    }

}

// generate trajectory message
geometry_msgs::Transform getPoseFromViewProposal(
        geometry_msgs::Point target_pose,double tracking_distance,Box PV){


    double PV_elev=PV.center_y;
    double PV_azim=PV.center_x;


    geometry_msgs::Transform transform;

    geometry_msgs::Vector3 viewPosition;

    viewPosition.x=target_pose.x+tracking_distance*cos(PV_elev)*cos(PV_azim);
    viewPosition.y=target_pose.y+tracking_distance*cos(PV_elev)*sin(PV_azim);
    viewPosition.z=target_pose.z+tracking_distance*sin(PV_elev);


    geometry_msgs::Quaternion viewYaw;

    double yaw=PV_azim+PI;

    viewYaw=tf::createQuaternionMsgFromYaw(yaw);

    transform.translation=viewPosition;
    transform.rotation=viewYaw;

    return transform;
}




geometry_msgs::Transform getPoseFromViewRay(
        geometry_msgs::Point target_pose,double tracking_distance,image_tracking::ProposalRay ray){



    double PV_elev=ray.elevation;
    double PV_azim=ray.azimuth;


    geometry_msgs::Transform transform;

    geometry_msgs::Vector3 viewPosition;

    viewPosition.x=target_pose.x+tracking_distance*cos(PV_elev)*cos(PV_azim);
    viewPosition.y=target_pose.y+tracking_distance*cos(PV_elev)*sin(PV_azim);
    viewPosition.z=target_pose.z+tracking_distance*sin(PV_elev);


    geometry_msgs::Quaternion viewYaw;

    double yaw=PV_azim+PI;

    viewYaw=tf::createQuaternionMsgFromYaw(yaw);

    transform.translation=viewPosition;
    transform.rotation=viewYaw;

    return transform;
}

/**
  Waypoint Proposal class
**/


//constructor 
WaypointProposer::WaypointProposer(float elev_min,float elev_max,unsigned int N_azim,unsigned int N_elev,float track_d,
                                   octomap::point3d min_point,octomap::point3d max_point)
{
//ROS_INFO("constructor init");
this->octree_obj=NULL; //fake initialization
this->tracking_distance=track_d;
this->elev_min=elev_min;
this->N_azim=N_azim;
this->N_elev=N_elev;
this->cast_space.reserve(N_azim*N_elev);
this->elev_max=elev_max;

ros::NodeHandle nh;
this->targetPath_sub=nh.subscribe("/target_predition_path",1,&WaypointProposer::targetPathCallback,this);
this->Octbin_sub=nh.subscribe("/octomap_full",3,&WaypointProposer::OctreeCallback,this);
this->PRs_sub=nh.subscribe("/proposal_ray_path",3,&WaypointProposer::PRsCallback,this);
this->PBs_pub=nh.advertise<image_tracking::ProposalBoxes>("/proposal_box_path",2);
this->server_query = nh.advertiseService("cast_query", &WaypointProposer::QueryfromTarget,this);
this->server_debug = nh.advertiseService("octomap_leaf_debug", &WaypointProposer::OctreeDebug,this);

this->freebox_max_point=max_point;
this->freebox_min_point=min_point;


this->marker_pub=nh.advertise<visualization_msgs::Marker>("casted_light", 4);
this->yawingArrow_pub=nh.advertise<visualization_msgs::Marker>("proposed_yawing",4);
this->boundingCube_pub=nh.advertise<visualization_msgs::Marker>("target_bounding_box",4);


//casted light

castedLightMarker.header.frame_id = "world";
castedLightMarker.header.stamp  = ros::Time::now();
castedLightMarker.ns = "casted_lights";
castedLightMarker.action = visualization_msgs::Marker::ADD;
castedLightMarker.pose.orientation.w = 1.0;
castedLightMarker.id = 0;
castedLightMarker.type = visualization_msgs::Marker::LINE_LIST;
castedLightMarker.scale.x = 0.05;
castedLightMarker.color.g = 1;
castedLightMarker.color.a = 0.5;
   



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



}



// destructor
WaypointProposer::~WaypointProposer(void) {
    delete octree_obj;
}

void WaypointProposer::OctreeCallback(const octomap_msgs::Octomap& msg){
    AbstractOcTree* octree=octomap_msgs::fullMsgToMap(msg);
    //octree updater
    this->octree_obj=(dynamic_cast<OcTree*>(octree));
}

// subscribe PRs and publish corresponding marker to draw in Rviz
void WaypointProposer::PRsCallback(const image_tracking::ProposalRays & PRs){
    //update PR path
    this->PRs.Ray_Path=PRs.Ray_Path;

    castedLightMarker.points.clear();


    geometry_msgs::PoseStamped cur_pose= this->pred_target.poses[0];

    geometry_msgs::Point target_position;
    target_position.x=cur_pose.pose.position.x;
    target_position.y=cur_pose.pose.position.y;
    target_position.z=cur_pose.pose.position.z;

    //iterate through PV (max=2)
    for (std::vector<image_tracking::ProposalRay>::iterator it=this->PRs.Ray_Path.begin();it!=this->PRs.Ray_Path.end();it++)
    {


        //this is most important !!!!!!!!!!!!!!
        geometry_msgs::Transform proposedViewPose=getPoseFromViewRay(target_position,tracking_distance,*it);

        castedLightMarker.points.push_back(target_position);

        geometry_msgs::Point endPoint;
        endPoint.x=proposedViewPose.translation.x;
        endPoint.y=proposedViewPose.translation.y;
        endPoint.z=proposedViewPose.translation.z;


        castedLightMarker.points.push_back(endPoint);

    }

}


// get path, publish Proposal Boxes (PBs)
void WaypointProposer::targetPathCallback(const nav_msgs::Path& msg){
    //path update
    this->pred_target=msg;


    std::vector<geometry_msgs::PoseStamped> target_predicted_poses=this->pred_target.poses;

    std::vector<geometry_msgs::PoseStamped>::iterator it;

    //castedLightMarker.points.clear();

    //clear Boxpath

    image_tracking::ProposalBoxes PBs;


    //iterator through predicted path
    for(it=target_predicted_poses.begin();it!=target_predicted_poses.end();it++)
    {
        // current target position
        geometry_msgs::Point target_position;
        target_position.x=it->pose.position.x;
        target_position.y=it->pose.position.y;
        target_position.z=it->pose.position.z;

        this->BBMarker.pose.position.x=target_position.x;
        this->BBMarker.pose.position.y=target_position.y;
        this->BBMarker.pose.position.z=target_position.z+(this->freebox_max_point.z()+this->freebox_min_point.z())/2.0;


        // corresponding view proposal
        ProposedView PV=viewProposal(target_position,true);


        std::cout<<"proposal finished"<<std::endl;

        // it is not optimal.. we should get rid of Box struct. use ProposalBox
        image_tracking::ProposalBox cur_PB;



        cur_PB.lower_left_point.x=PV.ProposedBoxes[0].lower_left_x;
        cur_PB.lower_left_point.y=PV.ProposedBoxes[0].lower_left_y;
        cur_PB.upper_right_point.x=PV.ProposedBoxes[0].upper_right_x;
        cur_PB.upper_right_point.y=PV.ProposedBoxes[0].upper_right_y;
        cur_PB.center.x=PV.ProposedBoxes[0].center_x;
        cur_PB.center.y=PV.ProposedBoxes[0].center_y;


        // save PBs
        PBs.PB_path.push_back(cur_PB);


        //after proposal, just save the marker.
        /**
        if (PV.ProposedBoxes.size()) // if it has proposed view , we plot center ray of PB
            //iterate through PV (max=2)
            for (std::vector<Box>::iterator it=PV.ProposedBoxes.begin();it!=PV.ProposedBoxes.end();it++)
            {

                geometry_msgs::Transform proposedViewPose=getPoseFromViewProposal(target_position,tracking_distance,*it);

                castedLightMarker.points.push_back(target_position);

                geometry_msgs::Point endPoint;
                endPoint.x=proposedViewPose.translation.x;
                endPoint.y=proposedViewPose.translation.y;
                endPoint.z=proposedViewPose.translation.z;


                castedLightMarker.points.push_back(endPoint);

            }
**/

    }
    //update PBs of proposer
    this->PBs.PB_path=PBs.PB_path;

}



// for query, we find clustered castspace
CastResult WaypointProposer::castRayandClustering(geometry_msgs::Point query_point,bool verbose)
{

    std::vector<DBSCAN::Point> visible_space;
    CastResult castresult(N_elev,N_azim);    
    
    if(octree_obj->size())
    {
        
        bool ignoreUnknownCells = true;
        std::vector<double> azimuth_iter=linspace(float(0),float(2*PI),float(N_azim));
        std::vector<double> elevation_iter=linspace(float(elev_min),float(elev_max),float(N_elev));
        
        point3d light_end; //will not be used
        point3d light_start(query_point.x,query_point.y,query_point.z);


        

        //nullify octomap around target

        double thresMin = octree_obj->getClampingThresMin();


        for(OcTree::leaf_bbx_iterator it = octree_obj->begin_leafs_bbx(freebox_min_point+light_start,freebox_max_point+light_start),
            end=octree_obj->end_leafs_bbx(); it!= end; ++it)
              it->setLogOdds(octomap::logodds(thresMin));
        octree_obj->updateInnerOccupancy();

        // ray casting 
        for (int ind_elev=0;ind_elev<N_elev; ind_elev++)
        {
            
            for (int ind_azim=0;ind_azim<N_azim;ind_azim++)
            {
               
                point3d light_dir(tracking_distance*cos(elevation_iter[ind_elev])*cos(azimuth_iter[ind_azim]),
                  tracking_distance*cos(elevation_iter[ind_elev])*sin(azimuth_iter[ind_azim]),
                  tracking_distance*sin(elevation_iter[ind_elev]));
                 
                // 1 : invisible 0 : visible

                castresult.mat[ind_elev][ind_azim]= octree_obj->castRay(light_start,light_dir,light_end,ignoreUnknownCells,tracking_distance);


                // for visible castspace, we need to cluster them
                if (!castresult.mat[ind_elev][ind_azim])
                {
                    DBSCAN::Point pnt({ind_elev,ind_azim,0,DBSCAN::NOT_CLASSIFIED});
                    visible_space.push_back(pnt);
                }
                                
            }

        }
        

        // clustering         
        const double epsilon=1.1;
        const int max_group_number=3;
        unsigned int minpts=2; // need to be tuned
        
        if(visible_space.size())
        {
         
            DBSCAN::DBCAN dbscan(max_group_number,epsilon,minpts,visible_space);
            dbscan.run();
            
            int  Ncluster=dbscan.cluster.size();
            castresult.Ncluster=Ncluster;
            castresult.clusterBB.reserve(Ncluster);
            // we need to find cluster BB(bounding box). from them, we obtain PB(proposal box)

            for (int cluster_idx=0;cluster_idx<Ncluster;cluster_idx++)
            {

                Box clusterBB;
                std::vector<int>::iterator it=dbscan.cluster[cluster_idx].begin();
                clusterBB.lower_left_x=clusterBB.upper_right_x=visible_space[*it].x;
                clusterBB.lower_left_y=clusterBB.upper_right_y=visible_space[*it].y;


                //... worry
                it++;
                for(;it!=dbscan.cluster[cluster_idx].end();it++)
                {


                    clusterBB.lower_left_x=std::max(visible_space[*it].x,clusterBB.lower_left_x);  //row
                    clusterBB.lower_left_y=std::min(visible_space[*it].y,clusterBB.lower_left_y);  //col

                    clusterBB.upper_right_x=std::min(visible_space[*it].x,clusterBB.upper_right_x);
                    clusterBB.upper_right_y=std::max(visible_space[*it].y,clusterBB.upper_right_y);

                 }

                castresult.clusterBB.push_back(clusterBB);
                castresult.clusterNvec.push_back(dbscan.cluster[cluster_idx].size());
                
            }
        //rayCast result print
        if(verbose)
            castresult.printResult();

        }
        else
        ROS_WARN("No visible castspace");
        


    } else ROS_WARN("No octree ");


    return castresult;
}


// for clustered castspace we propose corresponding region proposal
ProposedView WaypointProposer::regionProposal(CastResult castResult,bool verbose)
{
    ProposedView proposedView;

    // here box denote real coordinate
    int minPnts=6; //minimum number of cluster
    int count=0;
    //iterate through clusters
    if(castResult.clusterBB.size())
     {
        for(std::vector<Box>::iterator it=castResult.clusterBB.begin();
         it!=castResult.clusterBB.end();it++,count++)
        if(castResult.clusterNvec[count]>=minPnts)
        {
            int col=it->upper_right_y-it->lower_left_y+1;
            int row=-(it->upper_right_x-it->lower_left_x)+1;
            
            int transl_x=-it->upper_right_x;
            int transl_y=-it->lower_left_y;
            
            intMatrix matrix;
            for (int i=0;i<row;i++)
            {
                std::vector<int> row_vector;
                for (int j=0;j<col;j++)
                    row_vector.push_back(1-castResult.mat[i-transl_x][j-transl_y]);
                matrix.push_back(row_vector);
            }
            //if we have two or more in one cluster?
            std::vector<Box> bb_cur_cluster=maxRectangle(matrix);
            for(std::vector<Box>::iterator it=bb_cur_cluster.begin();it!=bb_cur_cluster.end();it++)
            {

                it->lower_left_x-=transl_x;
                it->upper_right_x-=transl_x;
                it->lower_left_y-=transl_y;
                it->upper_right_y-=transl_y;



                proposedView.ProposedBoxes.push_back(*it);
            }

        //std::cout<<proposedView.ProposedBoxes.size()<<std::endl;
        }
    if (verbose)
        proposedView.printProposedView(castResult);



    }
    else
        ROS_WARN("no region to proposal.");
    return proposedView;

}


// this function picks the most visible view point from a given light source
// also return in physical domain not the index


ProposedView WaypointProposer::viewProposal(geometry_msgs::Point point,bool verbose){


    if(octree_obj->size())

    {

        // perform raycast and clustering


        CastResult castresult=castRayandClustering(point,verbose);
        std::cout<<"---------------------------"<<std::endl;
        ProposedView proposedview=regionProposal(castresult,verbose); //it has view proposal more than two and it is index space

        if(proposedview.ProposedBoxes.size())
        {
            // after that, just pick two and convert to real space


            //std::cout<<"total number of proposed box: "<<proposedview.ProposedBoxes.size()<<std::endl;

            //sort in order of large area
            std::sort(proposedview.ProposedBoxes.begin(),proposedview.ProposedBoxes.end());

            //for(int i=0;i<proposedview.ProposedBoxes.size();i++)
                //std::cout<<"area: "<<proposedview.ProposedBoxes[i].area<<std::endl;

            // keep the beggest two

            if (proposedview.ProposedBoxes.size()>1 && proposedview.ProposedBoxes[0].area*0.5<proposedview.ProposedBoxes[1].area)
                proposedview.ProposedBoxes.erase(proposedview.ProposedBoxes.begin()+2,proposedview.ProposedBoxes.end());
            else
                proposedview.ProposedBoxes.erase(proposedview.ProposedBoxes.begin()+1,proposedview.ProposedBoxes.end());



            // keep the biggest for now
            //proposedview.ProposedBoxes.erase(proposedview.ProposedBoxes.begin()+1,proposedview.ProposedBoxes.end());


            std::vector<double> azimuth_iter=linspace(float(0),float(2*PI),float(N_azim));
            std::vector<double> elevation_iter=linspace(float(elev_min),float(elev_max),float(N_elev));


            // castRay from light source to proposoed view angle (max # = 2)
            std::vector<Box>  &PV=proposedview.ProposedBoxes;
            for (std::vector<Box>::iterator it=PV.begin();it!=PV.end();it++)
            {


                //proposed view point(= center of box)
                double PV_azim=(azimuth_iter[it->upper_right_y]+azimuth_iter[it->lower_left_y])/2;
                double PV_elev=(elevation_iter[it->lower_left_x]+elevation_iter[it->upper_right_x])/2;


                it->center_x=PV_azim;
                it->center_y=PV_elev;


                //now convert to real physical value

                int idx_lower_left_x=it->lower_left_x;
                int idx_lower_left_y=it->lower_left_y;
                int idx_upper_right_x=it->upper_right_x;
                int idx_upper_right_y=it->upper_right_y;

                it->lower_left_x=azimuth_iter[idx_lower_left_y];
                it->lower_left_y=elevation_iter[idx_upper_right_x];
                it->upper_right_x=azimuth_iter[idx_upper_right_y];
                it->upper_right_y=elevation_iter[idx_lower_left_x];

                /**
                std::cout<<"proposed center: ["<<PV_azim<<" , "<<PV_elev<<"]"<<std::endl;

                std::cout<<"lower left: ["<<it->lower_left_x<<" , "<<it->lower_left_y<<"]"<<std::endl;
                std::cout<<"upper_right: ["<< it->upper_right_x<<" , "<<it->upper_right_y<<"]"<<std::endl;
                **/
            }




        }

        return proposedview;

    }
    else
    {
      ROS_WARN( "no octree in proposer yet" );
      return ProposedView(); //throw empty PV

    }

}



// query
bool WaypointProposer::QueryfromTarget( image_tracking::CastQuery::Request &req, image_tracking::CastQuery::Response &resp){
    

    castedLightMarker.points.clear();
    yawingMarkerList.clear();


    geometry_msgs::Point point;
    point.x=(req.query_pose.x); point.y=(req.query_pose.y); point.z=(req.query_pose.z);


    ProposedView PV=viewProposal(point,true);

    int id_count=0;
    if (PV.ProposedBoxes.size()) // if it has proposed view
        for (std::vector<Box>::iterator it=PV.ProposedBoxes.begin();it!=PV.ProposedBoxes.end();it++)
        {

            geometry_msgs::Transform proposedViewPose=getPoseFromViewProposal(point,tracking_distance,*it);

            castedLightMarker.points.push_back(point);



            geometry_msgs::Point endPoint;
            endPoint.x=proposedViewPose.translation.x;
            endPoint.y=proposedViewPose.translation.y;
            endPoint.z=proposedViewPose.translation.z;





            //yawing angle arrow prototype

            yawingMarker.header.frame_id = "world";
            yawingMarker.header.stamp  = ros::Time::now();
            yawingMarker.ns = "yawing_arrow";
            yawingMarker.action = visualization_msgs::Marker::ADD;
            yawingMarker.id=id_count;

            yawingMarker.type = visualization_msgs::Marker::ARROW;
            yawingMarker.scale.x = 1;
            yawingMarker.scale.y = 0.05;
            yawingMarker.scale.z = 0.1;
            yawingMarker.color.r = 1;
            yawingMarker.color.a = 0.5;


            yawingMarker.pose.position=endPoint;
            yawingMarker.pose.orientation=proposedViewPose.rotation;

            yawingMarkerList.push_back((yawingMarker));

            castedLightMarker.points.push_back(endPoint);
            id_count++;

        }




        return true;


        // castRay from light source to all around in Rviz

        /**
        for (std::vector<double>::iterator it_elev = elevation_iter.begin() ; it_elev != elevation_iter.end(); ++it_elev)
        {
            int ind_azim=0;
            for (std::vector<double>::iterator it_azim = azimuth_iter.begin() ; it_azim != azimuth_iter.end(); ++it_azim)    
            {
                  point3d light_dir(tracking_distance*cos(*it_elev)*cos(*it_azim),
                    tracking_distance*cos(*it_elev)*sin(*it_azim),
                    tracking_distance*sin(*it_elev));

                // if the ray is not obstruded by voxel.
                if(!octree_obj->castRay(light_start,light_dir,light_end,ignoreUnknownCells,5.0))
                {
                    // vizualization marker 
                    geometry_msgs::Point p;

                    p.x=light_start.x();
                    p.y=light_start.y();
                    p.z=light_start.z();

                    castedLightMarker.points.push_back(p);

                    p.x=light_start.x()+light_dir.x();
                    p.y=light_start.y()+light_dir.y();
                    p.z=light_start.z()+light_dir.z();


                    castedLightMarker.points.push_back(p);
                }

            }
        }
        **/

}


void WaypointProposer::marker_publish(){
    if (castedLightMarker.points.size())
        marker_pub.publish(castedLightMarker);
    if (yawingMarkerList.size())
        for (std::vector<visualization_msgs::Marker>::iterator it=yawingMarkerList.begin();
             it!=yawingMarkerList.end();it++)
         yawingArrow_pub.publish(*it);

    this->boundingCube_pub.publish(this->BBMarker);

}


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






