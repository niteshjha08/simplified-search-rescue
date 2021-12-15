#include <ros/ros.h>
#include "../include/usr.h"
#include<iostream>

int main(int argc, char** argv){
    ros::init(argc, argv, "USR_node");
    ros::NodeHandle nh;
    USR explorer(&nh);
    USR follower(&nh);

    // Fetch aruco lookup locations and store in vector 
    std::vector < std::vector <double>> target_locations=explorer.m_get_target_locations();

    
    explorer.listen(explorer.tfBuffer);
    // Loop all target locations
    for (int i = 0;i < 4;++i) {
        //Move to location
        explorer.m_move_to_target(target_locations[i]);
        // Search at location for marker, broadcast if found
        explorer.m_search_at_target();
        //Set member variable to false again for next lookup search
        explorer.m_aruco_found=false;
        // lookupTransform between marker_frame and map
        explorer.listen(explorer.tfBuffer);
    }
    // explorer.m_move_to_target(target_locations[0]);

    // Move explorer back to base
    explorer.m_move_to_base();

    //Sort array based on fiducial id
    auto sorted_array = explorer.arrange(explorer.fiducial_array);

    //Loop the sorted 2D vector
    for (int i = 0; i < 4; i++)
    {   //Go to location
        follower.follow(sorted_array.at(i));
    }

    // Move follower back to base
    follower.f_move_to_base();
    ros::shutdown();

}   