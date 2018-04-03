//
// Created by jbs on 18. 3. 19.
//

#ifndef IMAGE_TRACKING_WAYPOINT_PROPOSAL_H
#define IMAGE_TRACKING_WAYPOINT_PROPOSAL_H


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Path.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

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
#include <gazebo_msgs/ModelStates.h>

#include <tf/transform_datatypes.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

// optimizer
#include <nlopt.h>
#include "optimization_funs.h"

#include <image_tracking/CastQuery.h>
#include <image_tracking/Debug.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Dense>

#define PI 3.141592
#define BB 2;

// B spline surface

#include <SPLINTER/datatable.h>
#include <SPLINTER/bspline.h>
#include <SPLINTER/bsplinebuilder.h>


struct viewVector{

    viewVector(){};

    double azim;
    double elev;
    double ray_length;
    geometry_msgs::Point centerPnt; // starting point of view vector

    /** this method returns endpoint of view vector  **/
    geometry_msgs::Point getEndPnt(){
        geometry_msgs::Point endPnt;
        endPnt.x=centerPnt.x+ray_length*cos(elev)*cos(azim);
        endPnt.y=centerPnt.y+ray_length*cos(elev)*sin(azim);
        endPnt.z=centerPnt.z+ray_length*sin(elev);
        return endPnt;
    }

    double getYaw(){
        return azim+PI;
    }
    /** this method returns proposed view pose
     *  the Transform class can be readily used to mavros waypoint publishing
    **/
    geometry_msgs::Transform getPoseFromViewVector(){
        geometry_msgs::Transform transform;

        geometry_msgs::Vector3 viewPosition;

        viewPosition.x=getEndPnt().x;
        viewPosition.y=getEndPnt().y;
        viewPosition.z=getEndPnt().z;

        geometry_msgs::Quaternion viewYaw;

        double yaw=elev+PI;

        viewYaw=tf::createQuaternionMsgFromYaw(yaw);

        transform.translation=viewPosition;
        transform.rotation=viewYaw;

        return transform;
    }

};


using namespace octomap;

using Eigen::MatrixXd;

/**
 * this class propose waypoint & pose for given target & octomap
 **/

class WaypointProposer{

public:

    //Constructor / Destructor
    WaypointProposer(float,float,unsigned int,unsigned int,float,octomap::point3d,octomap::point3d,ros::NodeHandle,Optimizer);
    ~ WaypointProposer();

    // castRay
    float tracking_distance;
    float elev_min; //min elevation of ray from target light source
    float elev_max;
    unsigned int N_azim; //how many azimuth sample in [0, 2PI]
    unsigned int N_elev; //how many elevation sample in [elev_min, elev_max]
    Eigen::VectorXd azim_set, elev_set;

    //Ray result
    MatrixXd castResult; // castResult matrix. [i,j] elements=hit length from i th azim and j th elev


    // objects
    std::string trackerName;
    std::string targetName;
    geometry_msgs::Pose trackerPose;
    geometry_msgs::Pose targetPose;
    void PB_path_publish(); //proposal boxes publish

    //waypoint
    viewVector desired_pose;
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;


    // Ray casting & print result
    void castRay(geometry_msgs::Point,bool,double,double);
    // Key function :cast rays and optimization
    void viewProposal();


    // TODO
    nav_msgs::Path pred_target;  //predicted target path

    // Octree
    OcTree* octree_obj;
    //Marker
    visualization_msgs::Marker castedLightMarker; // for query, we draw multiple rays
    visualization_msgs::Marker waypointMarker; // proposadddl waypoint ray from optimization


    //target 3 dim scale : we need to free octomap around the target

    // 3d box to nullify if target is (0,0,0)
    octomap::point3d freebox_min_point;
    octomap::point3d freebox_max_point;
    visualization_msgs::Marker BBMarker;

    // visibility cost computation

    Optimizer optimizer;

    //ROS
    ros::NodeHandle private_nh;
    ros::Subscriber Octbin_sub;
    ros::Subscriber states_sub;

    ros::Publisher marker_pub;
    ros::Publisher boundingCube_pub;
    ros::Publisher trajectory_pub;

    void waypoint_publish();
    void marker_publish();
    bool state_callback_flag,octomap_callback_flag;
    //bool QueryfromTarget( image_tracking::CastQuery::Request&, image_tracking::CastQuery::Response&);
    //bool OctreeDebug(image_tracking::Debug::Request& , image_tracking::Debug::Response&);

    //Optimization
    param param_;




private:
    //callback from octree
    void OctreeCallback(const octomap_msgs::Octomap&);    //for leaf node inspection
    // get state from gazebo
    void StateCallback(const gazebo_msgs::ModelStates::ConstPtr&);

};


#endif //IMAGE_TRACKING_WAYPOINT_PROPOSAL_H
