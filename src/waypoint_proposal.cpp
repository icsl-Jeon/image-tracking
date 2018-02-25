#include <waypoint_proposal.h>




//constructor 
WaypointProposer::WaypointProposer(float elev_min,unsigned int N_azim,unsigned int N_elev,float track_d)
{
//ROS_INFO("constructor init");
this->octree_obj=NULL; //fake initialization
this->tracking_distance=track_d;
this->elev_min=elev_min;
this->N_azim=N_azim;
this->N_elev=N_elev;
this->cast_space.reserve(N_azim*N_elev);

ros::NodeHandle nh;
this->Octbin_sub=nh.subscribe("/octomap_binary",10,&WaypointProposer::OctreeCallback,this);
this->server_query = nh.advertiseService("cast_query", &WaypointProposer::QueryfromTarget,this);
this->server_debug = nh.advertiseService("octomap_leaf_debug", &WaypointProposer::OctreeDebug,this);
this->marker_pub=nh.advertise<visualization_msgs::Marker>("casted_light", 10);


castedLightMarker.header.frame_id = "world";
castedLightMarker.header.stamp  = ros::Time::now();
castedLightMarker.ns = "casted_lights";
castedLightMarker.action = visualization_msgs::Marker::ADD;
castedLightMarker.pose.orientation.w = 1.0;
castedLightMarker.id = 0;
castedLightMarker.type = visualization_msgs::Marker::LINE_LIST;
castedLightMarker.scale.x = 0.05;
castedLightMarker.color.r = 1;
castedLightMarker.color.a = 0.5;
   
}



// destructor
WaypointProposer::~WaypointProposer(void) {
    delete octree_obj;
}

void WaypointProposer::OctreeCallback(const octomap_msgs::Octomap& msg){
    AbstractOcTree* octree=octomap_msgs::binaryMsgToMap(msg);
    //octree updater
    this->octree_obj=(dynamic_cast<OcTree*>(octree));
}

// print out casted space

inline void WaypointProposer::printCastspace(){

    int count=0;
    for (int it_elev =0 ; it_elev <N_elev; ++it_elev)
    {
        for (int it_azim = 0; it_azim < N_azim; ++it_azim)
        {
                std::cout<< cast_space[count]<<" ";
                count++;

        }
            std::cout<<"\n";
    }

}




inline void WaypointProposer::printClusteredCastspace(std::vector<int>  NoneObstruded_count,DBSCAN::DBCAN dbscan){

    int count=0; int search_count=0;
    for (int ind_elev=0;ind_elev!=N_elev;ind_elev++)
    {
        for (int ind_azim=0;ind_azim!=N_azim;ind_azim++)
        {

            if(count==NoneObstruded_count[search_count]) // visible castspace
            {


                //print this point. if it is close to center, then check star
                int this_cluster=dbscan.points[search_count].cluster;

                if(this_cluster!=DBSCAN::NOISE)
                    std::cout<<this_cluster<<" ";
                else
                    std::cout<< "#"<<" " ;// treat NOISE as non-visible


                search_count++;

            }
            else
                std::cout<< "#"<<" " ;// non-visible castspace

            count++;


        }
        std::cout<<"\n";
    }

}




// query
bool WaypointProposer::QueryfromTarget( image_tracking::CastQuery::Request &req, image_tracking::CastQuery::Response &resp){
    

    castedLightMarker.points.clear();
    bool verbose(true);

    if(octree_obj->size())

    {


    //for now, we don't know exact number of none-obstruded rays


    std::vector<int> NoneObstruded_count;
    bool ignoreUnknownCells = true;
    double 	maxRange = tracking_distance;

    std::vector<double> azimuth_iter=linspace(float(0),float(2*PI),float(N_azim));
    std::vector<double> elevation_iter=linspace(float(elev_min),float(PI/2.0),float(N_elev));
    point3d light_end;
    point3d light_start(req.query_pose.x,req.query_pose.y,req.query_pose.z);
            
                
        int count=0,NumNoneObstruded=0;

        int ind_elev=0,ind_azim=0;

        for (std::vector<double>::iterator it_elev = elevation_iter.begin() ; it_elev != elevation_iter.end(); ++it_elev)
        {
            ind_azim=0;
            for (std::vector<double>::iterator it_azim = azimuth_iter.begin() ; it_azim != azimuth_iter.end(); ++it_azim)    
            {
                  point3d light_dir(tracking_distance*cos(*it_elev)*cos(*it_azim),
                    tracking_distance*cos(*it_elev)*sin(*it_azim),
                    tracking_distance*sin(*it_elev));
                    cast_space[count]=octree_obj->castRay(light_start,light_dir,light_end,ignoreUnknownCells); 
                
                // if the ray is not obstruded by voxel.
                if (!cast_space[count])
                {   
                    // keep this count
                    NoneObstruded_count.push_back(count);

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


                    //for clustering, visible region

                    DBSCAN::Point pnt({ind_azim,ind_elev,0,DBSCAN::NOT_CLASSIFIED});
                    clustered_cast_space.push_back(pnt);


                    ++NumNoneObstruded;

                }
                ind_azim++;
                count++;

            }

            ind_elev++;
        }


        if (verbose) //print cast space
            printCastspace();

        printf("---------------------------------------\n");


        //allocation is ended. let's cluster.
        double epsilon=1.1;
        unsigned int minpts=2; // need to be tuned

        if(clustered_cast_space.size()) // if there is any visible ray
        {

            DBSCAN::DBCAN dbscan(5,epsilon,minpts,clustered_cast_space);
            dbscan.run();

            //let's obtain the center of each clustered
            std::vector<std::vector<double>> cluster_centers=dbscan.getClusterCenter();


            // print out if clustering is completed
            if(verbose)
             printClusteredCastspace(NoneObstruded_count,dbscan);

        }
        else
            ROS_WARN("All rays were obstruded");

    }
    else ROS_INFO_STREAM("octree is empty\n");
    return true;

}


void WaypointProposer::marker_publish(){
    if (castedLightMarker.points.size())
        marker_pub.publish(castedLightMarker);

}

bool WaypointProposer::OctreeDebug(image_tracking::Debug::Request &req, image_tracking::Debug::Response &res){
    
    ROS_INFO("octree size: %f\n",this->octree_obj->size());
    
    for(OcTree::leaf_iterator it = this->octree_obj->begin_leafs(),end_it=this->octree_obj->end_leafs(); it!=end_it; ++it)
    {
       ROS_INFO_STREAM("Node center"<<it.getCoordinate()<<" Depth: "<<it.getDepth()<<" Size: "<<it.getSize()
       <<" p: "<<exp(it->getValue())/(exp(it->getValue())+1)<<"\n");

    }
    return true;
}



