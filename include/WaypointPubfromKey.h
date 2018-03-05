#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Int8.h>
#include <iostream>
#include <string>
#include <math.h> 
#include <std_srvs/Empty.h>
#include <thread>
#include <chrono>     

#define PI 3.14159265

std::string intro[2]={"w=forward a=left d=right s=backward",
"q=left turn e=right turn z=elevation c=falling"};

class WaypointPubfromKey{
    private:
        //field
        Eigen::Vector3d desired_position;
        double desired_yaw; 
        //publisher
        ros::Publisher trajectory_pub;
        //subscriber
        ros::Subscriber key_sub;
        //msg 
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

    public:
        //constructor
        WaypointPubfromKey(ros::NodeHandle* );
        //publish
        void Publish();
        //callback 
        void Callback(const std_msgs::Int8Ptr&);
         
};


// of Course... constructer 
WaypointPubfromKey::WaypointPubfromKey(ros::NodeHandle* nh){


            //node handle
            
            ros::NodeHandle nh_pub;  
            ros::NodeHandle nh_sub;
            //initial value 
            desired_position.x()=0.0; desired_position.y()=0.0; desired_position.z()=1.0;    
            desired_yaw=0.0;
            //get param
            
            std::string ns = ros::this_node::getNamespace();
            nh->param("x", desired_position.x(), desired_position.x());
            nh->param("y", desired_position.y(), desired_position.y());
            nh->param("z", desired_position.z(), desired_position.z());
            nh->param("yaw", desired_yaw, desired_yaw);

            //set desired values to msg object
            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,desired_yaw
            ,&trajectory_msg); 

            //publisher
            trajectory_pub = nh_pub.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
            //subscriber
            ROS_INFO_STREAM("\n"<<intro[0]<<"\n"<<intro[1]<<"\n");
            std::string sub_name=ns+"/key";
            key_sub=nh_sub.subscribe(sub_name,10,&WaypointPubfromKey::Callback,this);

};

void WaypointPubfromKey::Publish(){


            this->trajectory_msg.header.stamp=ros::Time::now();
            this->trajectory_pub.publish(this->trajectory_msg); 

};

void WaypointPubfromKey::Callback(const std_msgs::Int8Ptr& key){

            float incrm=0.1; // increament of movement 
            int k=key->data; //key value 
            


                switch(k){
                    case 119 :   //key=w : forward
                            desired_position.x()+=incrm*cos(desired_yaw);
                            desired_position.y()+=incrm*sin(desired_yaw);       
                            break;                          
                    case 97:    //key=a : left 
                            desired_position.x()+=incrm*cos(desired_yaw+PI/2.0);
                            desired_position.y()+=incrm*sin(desired_yaw+PI/2.0); 
                            break;                          

                    case 115:   //key=s : backard 
                            desired_position.x()+=incrm*cos(desired_yaw+PI);
                            desired_position.y()+=incrm*sin(desired_yaw+PI); 
                            break;  
                    case 100:   //key=d : right 
                            desired_position.x()+=incrm*cos(desired_yaw-PI/2.0);
                            desired_position.y()+=incrm*sin(desired_yaw-PI/2.0); 
                            break;  
                    case 113:   //key=q : left turn 
                            desired_yaw+=incrm*PI/2.0;
                            break;  
                    case 101:   //key=e : right turn 
                            desired_yaw-=incrm*PI/2.0;
                            break;  
                    case 122:   //key=z : up 
                            desired_position.z()+=incrm; 
                            break;  
                    case 99:   //key=c : down
                            desired_position.z()-=incrm;
                            break;  
                    default: 
                            ROS_INFO("Entered wrong input\n");

                }


            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,desired_yaw, &trajectory_msg);

                

            
};
