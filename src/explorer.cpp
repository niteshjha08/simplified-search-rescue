#include "../include/explorer.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <cmath>

Explorer::Explorer(ros::NodeHandle* nodehandle) :
    m_nh{ *nodehandle },
    explorer_client{"explorer/move_base",true},
    m_aruco_found{false},
    curr_fiducial_id{NULL}
    // tfBuffer{},
    // tfListener{tfBuffer}
    // tfListener{static tf2_ros::TransformListener tfListener(tfBuffer)}
    {
    m_initialize_subscribers();
    m_initialize_publishers();
    // m_initialize_movebase();
}

void Explorer::m_initialize_subscribers(){
    m_aruco_detect_subscriber = m_nh.subscribe("/fiducial_transforms", 1, &Explorer::m_aruco_detect_callback,this);

}

void Explorer::m_initialize_publishers(){
    m_explorer_rotator = m_nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 1000);

}



void Explorer::broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    //broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame";
    
    transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z-0.7; // tolerance of 0.5m
    transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
    transformStamped.transform.rotation.y = msg->transforms[0].transform.rotation.y;
    transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;
    transformStamped.transform.rotation.w = msg->transforms[0].transform.rotation.w;
    curr_fiducial_id=msg->transforms[0].fiducial_id;
    ROS_INFO("Broadcasting");
    br.sendTransform(transformStamped);
    // br.sendTransform(transformStamped);
    // ros::Duration(2).sleep();
}

void Explorer::listen(tf2_ros::Buffer& tfBuffer) {
  //for listener
  static tf2_ros::TransformListener tfListener(tfBuffer);
//   ros::Duration(2.0).sleep();
  geometry_msgs::TransformStamped transformStamped;
  bool success=false;
  int count=0;
  double fiducial_ids{curr_fiducial_id};
//   while(!success){

    try {
        transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));
        auto trans_x = transformStamped.transform.translation.x;
        auto trans_y = transformStamped.transform.translation.y;
        
        fiducial_array.push_back({fiducial_ids, trans_x,trans_y});
        
        ROS_INFO_STREAM("Position in map frame: ["
        << trans_x << ","
        << trans_y << "]"
        <<" for marker : "<<fiducial_ids
        );
        // m_aruco_found=true;
        // success=true;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
//   }
  
}

void Explorer::m_aruco_detect_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
    if (!msg->transforms.empty() && msg->transforms[0].fiducial_area>10000){
        std::cout<<"fiducial found with id:"<<msg->transforms[0].fiducial_id<<" with area :"<<msg->transforms[0].fiducial_area<<'\n';
        m_aruco_found=true;
        broadcast(msg);
    }
}
// void Explorer::m_initialize_movebase(){
//     MoveBaseClient explorer_client("/explorer/move_base", true);
// }

void Explorer::m_move_to_target(std::vector<double> aruco_lookup_location){
    move_base_msgs::MoveBaseGoal explorer_goal;
    explorer_goal.target_pose.header.frame_id = "map";
    explorer_goal.target_pose.header.stamp = ros::Time::now();
    explorer_goal.target_pose.pose.position.x = aruco_lookup_location[0];//
    explorer_goal.target_pose.pose.position.y = aruco_lookup_location[1];//
    explorer_goal.target_pose.pose.orientation.w = 1.0;
    while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
                                                                }
    explorer_client.sendGoal(explorer_goal);
    explorer_client.waitForResult();
}

void Explorer::m_search_at_target(){
    geometry_msgs::Twist msg;
    msg.angular.z=0.1;
    int count=0;
    ros::Rate rate(5);
    while(!m_aruco_found){
        m_explorer_rotator.publish(msg);
        count+=1;
        // std::cout<<"count:"<<count<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    msg.angular.z=0.0;
    m_explorer_rotator.publish(msg);
    
}

void Explorer::m_move_to_base(){
    move_base_msgs::MoveBaseGoal explorer_goal;
    explorer_goal.target_pose.header.frame_id = "map";
    explorer_goal.target_pose.header.stamp = ros::Time::now();
    explorer_goal.target_pose.pose.position.x = -4.0;//
    explorer_goal.target_pose.pose.position.y = 2.5;//
    explorer_goal.target_pose.pose.orientation.w = 1.0;
    while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
                                                                }
    explorer_client.sendGoal(explorer_goal);
    explorer_client.waitForResult();
}

std::vector < std::vector <double>> Explorer::m_get_target_locations(){
    std::vector < std::vector <double>> target_locations{};
    m_nh.getParam("/aruco_lookup_locations/target_1", aruco_lookup_locations);
    target_locations.push_back({aruco_lookup_locations[0],aruco_lookup_locations[1]});
    // target_locations.push_back({aruco_lookup_locations[0]});

    m_nh.getParam("/aruco_lookup_locations/target_2", aruco_lookup_locations);
    target_locations.push_back({aruco_lookup_locations[0],aruco_lookup_locations[1]});

    m_nh.getParam("/aruco_lookup_locations/target_3", aruco_lookup_locations);
    target_locations.push_back({aruco_lookup_locations[0],aruco_lookup_locations[1]});

    m_nh.getParam("/aruco_lookup_locations/target_4", aruco_lookup_locations);
    target_locations.push_back({aruco_lookup_locations[0],aruco_lookup_locations[1]});

    return target_locations;
}