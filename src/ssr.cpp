#include <ros/ros.h>
#include "../include/usr.h"
#include<iostream>

int main(int argc, char** argv){
    ros::init(argc, argv, "USR_node");
    ros::NodeHandle nh;
    USR explorer(&nh);
    USR follower(&nh);

    std::vector < std::vector <double>> target_locations=explorer.m_get_target_locations();

    
    explorer.listen(explorer.tfBuffer);

    for (int i = 0;i < 4;++i) {
        explorer.m_move_to_target(target_locations[i]);
        explorer.m_search_at_target();
        explorer.m_aruco_found=false;
        explorer.listen(explorer.tfBuffer);
    }
    // explorer.m_move_to_target(target_locations[0]);
    explorer.m_move_to_base();
    auto sorted_array = explorer.arrange(explorer.fiducial_array);
    for (int i = 0; i < 4; i++)
    {
        follower.follow(sorted_array.at(i));
    }
    follower.f_move_to_base();
}   