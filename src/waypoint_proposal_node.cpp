#include <waypoint_proposal.h>

/** usage
 * arg : tracking_d elev_min N_azim N_elev
**/

int main(int argc, char  **argv)
{

    double track_d;
    double elev_min;
    double elev_max;
    int N_azim;
    int N_elev;

    // getting parameter from launch file



    ros::init(argc, argv, "waypoint_proposal_node");

    ros::NodeHandle nh_private("~");
    nh_private.param("track_d", track_d, 2.0);
    nh_private.param("elev_min", elev_min, PI/6.0);
    nh_private.param("elev_max",elev_max,PI/2.0);
    nh_private.param("N_azim", N_azim, 10);
    nh_private.param("N_elev",N_elev, 6);


    WaypointProposer waypoint_proposer(elev_min,elev_max,N_azim,N_elev,track_d);
    ros::Rate rate(10.0);
    while (ros::ok()){
    waypoint_proposer.marker_publish();
    ros::spinOnce();
    rate.sleep();
    }
    return 0;

}
