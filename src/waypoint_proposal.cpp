#include <waypoint_proposal.h>


//constructor 
WaypointProposer::WaypointProposer(float elev_min,unsigned int N_azim,unsigned int N_elev,float track_d){

this->tracking_distance=track_d;
this->elev_min=elev_min;
this->N_azim=N_azim;
this->N_elev=N_elev;
this->cast_space.reserve(N_azim*N_elev);
ros::NodeHandle nh;
this->Octbin_sub=nh.subscribe("/octomap_binary",10,this->OctreeCallback);
this->service = nh.advertiseService("cast_query", this->QueryfromTarget);

}
 

void WaypointProposer::OctreeCallback(const octomap_msgs::Octomap& msg){

    AbstractOcTree* octree=octomap_msgs::binaryMsgToMap(msg);
    //octree update
    this->octree_obj=*(dynamic_cast<OcTree*>(octree));
}


void WaypointProposer::QueryfromTarget(const WaypointPropser::CastQuery::Request &req,const WaypointPropser::CastQuery::Reponse &resp){


    if(octree.size())
    {
    bool ignoreUnknownCells = false
    double 	maxRange = tracking_distance;

    std::vector<double> azimuth_iter=linspace(0,2*PI,N_azim);
    std::vector<double> elevation_iter=&(linspace(elev_min,2*PI,N_elev));
    point3d light_end;
    point3d light_start(req.position.x,req.position.y,req.position.z);
            
                
        int count=0;
        for (std::vector<double>::iterator it_elev = elevation_iter.begin() ; it_elev != elevation_iter.end(); ++it_elev)
        for (std::vector<double>::iterator it_azim = azimuth_iter.begin() ; it_azim != azimuth_iter.end(); ++it_azim)
            {
            {
                  point3d light_dir(tracking_distance*cos(*it_elev)*cos(*it_azim),
                    tracking_distance*cos(*it_elev)*sin(*it_azim),
                    tracking_distance*sin(*it_elev));
                    cast_space[count]=octree.castRay(light_start.light_dir,ignoreUnknownCells,light_end,maxRange); 
                if (verbose) 
                    std::cout<< cast_space[count]<<" ";               
            }
            if (verbose)
                std::cout<<"\n";
            }
    }
    else cout<<"octree is empty"<<endl;

    return true;

}

void WaypointProposer::OctreeDebug(){
    
    
    for(OcTree::leaf_iterator it = this->octree_obj.begin_leafs(),end=this->octree_obj.end_leafs(); it!=end; ++it)
    {
        std::cout<<"Node center"<<it.getCoordinate();
        std::cout<<" Depth: "<<it.getDepth();
        std::cout<<" Size: "<<it.getSize();
        // naive value is log odd value.
        std::cout<<" p: "<<exp(it->getValue())/(exp(it->getValue())+1)<<"\n";
    }

}



