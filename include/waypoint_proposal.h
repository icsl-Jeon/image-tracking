#ifndef HEADER_H
#define HEADER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <nav_msgs/Path.h>
#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <math.h>
#include <iostream>
#include <vector>
  
#include <image_tracking/CastQuery.h>
#include <image_tracking/Debug.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <dbscan.h>
#include <boxoperator.h>


#define PI 3.141592
#define BB 2;

typedef std::vector<bool> CastSpace;  //1D azi-elev space (flattened)
typedef std::vector<DBSCAN::Point> ClusteredCastSpace; //2D azi-elev space. x,y member is index
typedef std::vector<CastSpace> CastSpaceBuffer; // buffer of CastSpace


using namespace octomap;



//this class has info about casted result with clusterd BBs(bounding box)
class CastResult{
public:
    intMatrix mat;
    std::vector<Box> clusterBB; //cluster bounding box
    std::vector<int> clusterNvec; //cluster numbers
    int Ncluster; //total number of cluster

    // constructor
    CastResult(int,int);

    // print result
    void printResult();

};


// this class has PBs(proposed box)
class ProposedView{

    public:
        std::vector<Box> ProposedBoxes; //proposed castspace regions(in real domain. not index)
        double secs; //time stamp
        void printProposedView(const CastResult & );

};



class WaypointProposer{

    public:
        float tracking_distance; 
        float elev_min; //min elevation of ray from target light source 
        float elev_max;
        unsigned int N_azim; //how many azimuth sample in [0, 2PI]
        unsigned int N_elev; //how many elevation sample in [elev_min, PI/2]
        ProposedView viewProposal(geometry_msgs::Point);
        bool QueryfromTarget( image_tracking::CastQuery::Request&, image_tracking::CastQuery::Response&);
        bool OctreeDebug(image_tracking::Debug::Request& , image_tracking::Debug::Response&);  
        nav_msgs::Path pred_target;  //predicted target path

        CastSpace cast_space;
        CastSpaceBuffer cast_space_buffer;
        //Octree 
        OcTree* octree_obj;
        //Marker
        visualization_msgs::Marker castedLightMarker;
        visualization_msgs::Marker yawingMarker;
        std::vector<visualization_msgs::Marker> yawingMarkerList;

        //target 3 dim scale : we need to nullify the octomap around the target

        // 3d box to nullify if target is (0,0,0)
        octomap::point3d freebox_min_point;
        octomap::point3d freebox_max_point;
        visualization_msgs::Marker BBMarker;


        //ROS 
        ros::Subscriber Octbin_sub;
        ros::Subscriber targetPath_sub;
        ros::ServiceServer server_query;
        ros::ServiceServer server_debug;
        ros::Publisher marker_pub;
        ros::Publisher boundingCube_pub;
        ros::Publisher yawingArrow_pub;

        void marker_publish();

        //Constructor / Destructor
        WaypointProposer(float,float,unsigned int,unsigned int,float,octomap::point3d,octomap::point3d);
        ~ WaypointProposer();
                
        // Ray casting
        CastResult castRayandClustering(geometry_msgs::Point,bool=false);
        // Observation proposal
        ProposedView regionProposal(CastResult,bool=false);

        // final processing
        ProposedView viewProposal(geometry_msgs::Point,bool);


    private:
        //constructor
        //callback from octree
        void OctreeCallback(const octomap_msgs::Octomap&); 
        void targetPathCallback(const nav_msgs::Path &);
        //for leaf node inspection
        //destructor

};




//linspace 
template<typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in)
{

  std::vector<double> linspaced;


  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1) 
    {
      linspaced.push_back(start);
      return linspaced;
    }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
    {
      linspaced.push_back(start + delta * i);
    }
  linspaced.push_back(end); // I want to ensure that start and end
                            // are exactly the same as the input
  return linspaced;
}




#endif 
