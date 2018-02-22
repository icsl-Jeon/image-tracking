#include <waypoint_proposal.h>
/** usage
 * arg : tracking_d elev_min N_azim N_elev
**/

int main(int argc, char  **argv)
{
    float tracking_d=2.0;
    float elev_min=PI/6.0;
    unsigned int N_azim=10;
    unsigned int N_elev=6;


	ros::init(argc, argv, "waypoint_proposal_node");
	WaypointProposer waypoint_proposer(elev_min,N_azim,N_elev,tracking_d);
    ros::Rate rate(10.0);
    while (ros::ok()){

    ros::spinOnce(); // this is where the magic happens!!
    rate.sleep();
    }
    return 0;

}
