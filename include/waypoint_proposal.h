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


#define PI 3.141592

typedef std::vector<bool> CastSpace;  //2D azi-elev space
typedef std::vector<CastSpace> CastSpaceBuffer; // buffer of CastSpace

using namespace octomap;

/**
 TODO : 
 octomap assignment?
**/

class WaypointProposer{

    public:
        //TODO
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
        //ROS 
        ros::Subscriber Octbin_sub;
        ros::ServiceServer server_query;
        ros::ServiceServer server_debug;

        WaypointProposer(float,unsigned int,unsigned int,float);
        ~ WaypointProposer();


    private:
        //constructor
        //callback from octree
        void OctreeCallback(const octomap_msgs::Octomap&); 
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
