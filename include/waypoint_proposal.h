#ifndef HEADER_H
#define HEADER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <iostream>
#include <vector>
  
#include <image_tracking/CastQuery.h>
#include <image_tracking/Debug.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <dbscan.h>


#define PI 3.141592
#define BB 2;

typedef std::vector<bool> CastSpace;  //1D azi-elev space (flattened)
typedef std::vector<DBSCAN::Point> ClusteredCastSpace; //2D azi-elev space. x,y member is index
typedef std::vector<CastSpace> CastSpaceBuffer; // buffer of CastSpace
typedef std::vector< std::vector<bool> > boolMatrix; // bool matrix 
typedef std::vector< std::vector<int> > intMatrix;


using namespace octomap;


struct Box{

    double upper_right_x;
    double upper_right_y;
    double lower_left_x;
    double lower_left_y;

};


class CastResult{
public:
    boolMatrix mat;
    std::vector<Box> clusterBB; //cluster bounding box
    int Ncluster; //total number of cluster

    // constructor
    CastResult(int,int);

    // print result
    void printResult();

};



class ProposedView{

    public:
        std::vector<Box> ProposedBoxes; //proposed castspace regions(in real domain. not index)
        double secs; //time stamp
};



class WaypointProposer{

    public:
        float tracking_distance; 
        float elev_min; //min elevation of ray from target light source 
        unsigned int N_azim; //how many azimuth sample in [0, 2PI]
        unsigned int N_elev; //how many elevation sample in [elev_min, PI/2]
        bool QueryfromTarget( image_tracking::CastQuery::Request&, image_tracking::CastQuery::Response&);  
        bool OctreeDebug(image_tracking::Debug::Request& , image_tracking::Debug::Response&);  


        CastSpace cast_space;
        CastSpaceBuffer cast_space_buffer;
        //Octree 
        OcTree* octree_obj;
        //Marker
        visualization_msgs::Marker castedLightMarker;

        //ROS 
        ros::Subscriber Octbin_sub;
        ros::ServiceServer server_query;
        ros::ServiceServer server_debug;
        ros::Publisher marker_pub;
        void marker_publish();

        //Constructor / Destructor
        WaypointProposer(float,unsigned int,unsigned int,float);
        ~ WaypointProposer();
                
        // Ray casting
        CastResult castRayandClustering(geometry_msgs::Point,bool=false);
        // Observation proposal
        //std::vector<Box> regionProposal(CastResult,bool=false);


    private:
        //constructor
        //callback from octree
        void OctreeCallback(const octomap_msgs::Octomap&); 
        inline void printCastspace();
        inline void printClusteredCastspace(std::vector<int>,DBSCAN::DBCAN);
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
