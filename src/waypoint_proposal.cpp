#include <waypoint_proposal.h>


//constructor 
WaypointProposer::WaypointProposer(float elev_min,unsigned int N_azim,unsigned int N_elev,float track_d)
{
ROS_INFO("constructor init");
this->octree_obj=NULL; //fake initialization
this->tracking_distance=track_d;
this->elev_min=elev_min;
this->N_azim=N_azim;
this->N_elev=N_elev;
this->cast_space.reserve(N_azim*N_elev);
ros::NodeHandle nh;
this->Octbin_sub=nh.subscribe("octomap_binary",10,&WaypointProposer::OctreeCallback,this);
this->server_query = nh.advertiseService("cast_query", &WaypointProposer::QueryfromTarget,this);
this->server_debug = nh.advertiseService("octomap_leaf_debug", &WaypointProposer::OctreeDebug,this);
 
}


WaypointProposer::~WaypointProposer(void) {
    delete octree_obj;
}

void WaypointProposer::OctreeCallback(const octomap_msgs::Octomap& msg){
    AbstractOcTree* octree=octomap_msgs::binaryMsgToMap(msg);
    //octree update
    this->octree_obj=(dynamic_cast<OcTree*>(octree));
}

bool WaypointProposer::QueryfromTarget( image_tracking::CastQuery::Request &req, image_tracking::CastQuery::Response &resp){
    bool verbose(true);

    if(octree_obj->size())
    {
    bool ignoreUnknownCells = false;
    double 	maxRange = tracking_distance;

    std::vector<double> azimuth_iter=linspace(float(0),float(2*PI),float(N_azim));
    std::vector<double> elevation_iter=linspace(float(elev_min),float(2*PI),float(N_elev));
    point3d light_end;
    point3d light_start(req.query_pose.x,req.query_pose.y,req.query_pose.z);
            
                
        int count=0;
        for (std::vector<double>::iterator it_elev = elevation_iter.begin() ; it_elev != elevation_iter.end(); ++it_elev)
        {
            for (std::vector<double>::iterator it_azim = azimuth_iter.begin() ; it_azim != azimuth_iter.end(); ++it_azim)    
            {
                  point3d light_dir(tracking_distance*cos(*it_elev)*cos(*it_azim),
                    tracking_distance*cos(*it_elev)*sin(*it_azim),
                    tracking_distance*sin(*it_elev));
                    cast_space[count]=octree_obj->castRay(light_start,light_dir,light_end,ignoreUnknownCells,maxRange); 
                if (verbose) 
                    std::cout<< cast_space[count]<<" ";               
            }
            if (verbose)
                std::cout<<"\n";
        }
    }
    else ROS_INFO_STREAM("octree is empty\n");

    return true;

}

bool WaypointProposer::OctreeDebug(image_tracking::Debug::Request &req, image_tracking::Debug::Response &res){
    
    ROS_INFO("here is service\n");
    for(OcTree::leaf_iterator it = this->octree_obj->begin_leafs(),end_it=this->octree_obj->end_leafs(); it!=end_it; ++it)
    {
       ROS_INFO_STREAM("Node center"<<it.getCoordinate()<<" Depth: "<<it.getDepth()<<" Size: "<<it.getSize()
       <<" p: "<<exp(it->getValue())/(exp(it->getValue())+1)<<"\n");

    }
    return true;
}



