/**
 * this node takes key value from key node 
 * then publish waypoint
 * params: intial waypoint (hovering position) 
**/
#include "WaypointPubfromKey.h"

int main(int argc,char ** argv){
    
    ros::init(argc, argv, "waypoint_publisher_from_key");
    ROS_INFO("Started waypoint_pub_from_key");

    ros::Time::init();
    ros::Rate loop_rate(30);
    ros::NodeHandle nh("~");
    std_srvs::Empty srv;

    //call unpaused 
    unsigned int i=0;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    
    while (i <= 10 && !unpaused) {
      ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      unpaused = ros::service::call("/gazebo/unpause_physics", srv);
      ++i;
    } 

    WaypointPubfromKey node_class(&nh);

    while (ros::ok())
    {

        node_class.Publish();
        ros::spinOnce();
        loop_rate.sleep();

    }

     return 0;

}



