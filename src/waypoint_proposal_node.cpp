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
    double xy_length_target;
    double z_min_target;
    double z_max_target;
    // getting parameter from launch file



    ros::init(argc, argv, "waypoint_proposal_node");

    ros::NodeHandle nh_private("~");
    nh_private.param("track_d", track_d, 2.0);
    nh_private.param("elev_min", elev_min, PI/6.0);
    nh_private.param("elev_max",elev_max,PI/2.0);
    nh_private.param("N_azim", N_azim, 10);
    nh_private.param("N_elev",N_elev, 6);
    nh_private.param("xy_length_target",xy_length_target,1.0);
    nh_private.param("z_min_target",z_min_target,-0.2);
    nh_private.param("z_max_target",z_max_target,0.7);


    octomap::point3d freebox_min_point(-xy_length_target/2,-xy_length_target/2,z_min_target);
    octomap::point3d freebox_max_point(xy_length_target/2,xy_length_target/2,z_max_target);



    WaypointProposer waypoint_proposer(elev_min,elev_max,N_azim,N_elev,track_d,freebox_min_point,freebox_max_point);

    ros::Rate rate(5.0);
    while (ros::ok()){
    waypoint_proposer.marker_publish();
    if (waypoint_proposer.PB_path.PB_path.size())
        waypoint_proposer.PB_path_pub.publish(waypoint_proposer.PB_path);
    ros::spinOnce();
    rate.sleep();
    }
    return 0;

}
