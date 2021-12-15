#include <ros/ros.h>
#include "../include/explorer.h"
#include<iostream>

int main(int argc, char** argv){
    ros::init(argc, argv, "USR_node");
    ros::NodeHandle nh;
    Explorer explorer(&nh);

    std::vector < std::vector <double>> target_locations=explorer.m_get_target_locations();

    // for(int i=0;i<4;++i){
    //     for(int j=0;j<2;++j)
    //     std::cout<<target_locations[i][j];
    // }
    // explorer.listen(explorer.tfBuffer);
    for(int i=0;i<4;++i){

        explorer.m_move_to_target(target_locations[i]);
        explorer.m_search_at_target();
        explorer.m_aruco_found=false;
        explorer.listen(explorer.tfBuffer);
    }
    explorer.m_move_to_target(target_locations[0]);
    explorer.m_move_to_base();
    std::cout<<"Array locations: "; 
    for(int i=0;i<4;++i){
        for(int j=0;j<3;++j)
        std::cout<<explorer.fiducial_array[i][j];
        std::cout<<" ";}

}   
// canTransform
//tfBuffer and tfListener in initialization list