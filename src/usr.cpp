#include "../include/usr.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <cmath>

USR::USR(ros::NodeHandle* nodehandle) :
    m_nh{ *nodehandle },
    explorer_client{ "explorer/move_base",true },
    follower_client{ "follower/move_base",true },
    m_aruco_found{ false },
    curr_fiducial_id{NULL}
    {
    m_initialize_subscribers();
    m_initialize_publishers();
    
}

void USR::m_initialize_subscribers(){
    m_aruco_detect_subscriber = m_nh.subscribe("/fiducial_transforms", 1, &USR::m_aruco_detect_callback,this);

}


void USR::m_initialize_publishers() {
    m_explorer_rotator = m_nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 1000);

}
void USR::broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    //broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame";

    // Obtain position and orientation of aruco    
    transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z-0.5; // tolerance of 0.5m
    transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
    transformStamped.transform.rotation.y = msg->transforms[0].transform.rotation.y;
    transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;
    transformStamped.transform.rotation.w = msg->transforms[0].transform.rotation.w;
    curr_fiducial_id=msg->transforms[0].fiducial_id;
    ROS_INFO("Broadcasting");
    br.sendTransform(transformStamped);
    ros::Duration(2).sleep(); // Buffer for lookupTransform
}

void USR::listen(tf2_ros::Buffer& tfBuffer) {
  //for listener
  static tf2_ros::TransformListener tfListener(tfBuffer);
//   ros::Duration(2.0).sleep();
  geometry_msgs::TransformStamped transformStamped;
  bool success=false;
  int count=0;
  double fiducial_ids{curr_fiducial_id};
 

    try {
        // Find transform between marker_frame and map frame at the current time
        transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));
        auto trans_x = transformStamped.transform.translation.x;
        auto trans_y = transformStamped.transform.translation.y;
        
        fiducial_array.push_back({fiducial_ids, trans_x,trans_y});
        
        ROS_INFO_STREAM("Position in map frame: ["
        << trans_x << ","
        << trans_y << "]"
        <<" for marker : "<<fiducial_ids
        );
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
  
}

void USR::m_aruco_detect_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
    // Check if the msg is non-empty (aruco found) and that the area is greater than 10000 
    // (to prevent markers at far away distances to be falsely detected)
    if (!msg->transforms.empty() && msg->transforms[0].fiducial_area>10000){
        std::cout<<"fiducial found with id:"<<msg->transforms[0].fiducial_id<<" with area :"<<msg->transforms[0].fiducial_area<<'\n';
        m_aruco_found=true;
        broadcast(msg);
    }
}
void USR::m_move_to_target(std::vector<double> aruco_lookup_location){
    move_base_msgs::MoveBaseGoal explorer_goal;
    explorer_goal.target_pose.header.frame_id = "map";
    explorer_goal.target_pose.header.stamp = ros::Time::now();
    explorer_goal.target_pose.pose.position.x = aruco_lookup_location[0];// x coordinate
    explorer_goal.target_pose.pose.position.y = aruco_lookup_location[1];// y coordinate
    explorer_goal.target_pose.pose.orientation.w = 1.0;
    while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
                                                                }
    explorer_client.sendGoal(explorer_goal);
    explorer_client.waitForResult();
}
void USR::m_move_to_base() {
    move_base_msgs::MoveBaseGoal explorer_goal;
    explorer_goal.target_pose.header.frame_id = "map";
    explorer_goal.target_pose.header.stamp = ros::Time::now();
    explorer_goal.target_pose.pose.position.x = -4.0;// start location x - explorer
    explorer_goal.target_pose.pose.position.y = 2.5;// start location y - explorer
    explorer_goal.target_pose.pose.orientation.w = 1.0;
    while (!explorer_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up for explorer");
    }
    explorer_client.sendGoal(explorer_goal);
    explorer_client.waitForResult();
}
void USR::f_move_to_base() {
    move_base_msgs::MoveBaseGoal follower_goal;
    follower_goal.target_pose.header.frame_id = "map";
    follower_goal.target_pose.header.stamp = ros::Time::now();
    follower_goal.target_pose.pose.position.x = -4.0;//x coordinate - start location for follower
    follower_goal.target_pose.pose.position.y = 3.5;// y coordinate - start location for follower
    follower_goal.target_pose.pose.orientation.w = 1.0;
    while (!follower_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up for explorer");
    }
    follower_client.sendGoal(follower_goal);
    follower_client.waitForResult();
}

void USR::m_search_at_target(){
    geometry_msgs::Twist msg;
    msg.angular.z = 0.1; // Angular velocity of explorer while searching at lookup location
    ros::Rate rate(5);
    while(!m_aruco_found){
        m_explorer_rotator.publish(msg);
        ros::spinOnce(); 
        rate.sleep();
    }
    msg.angular.z=0.0; // Stop rotation once aruco is found
    m_explorer_rotator.publish(msg);
    
}
// Driver function to execute sorting of 2D vector
bool sortcol(const std::vector<double>& v1, const std::vector<double>& v2) {
    return v1[0] < v2[0];
}

std::vector<std::vector<double>> USR::arrange(std::vector<std::vector<double>>fiducial_array) {
    
    sort(fiducial_array.begin(),fiducial_array.end(), sortcol);
    return fiducial_array;

}


void USR::follow(std::vector<double>fiducial_array) {
    
    move_base_msgs::MoveBaseGoal follower_goal;
    follower_goal.target_pose.header.frame_id = "map";
    follower_goal.target_pose.header.stamp = ros::Time::now();
    follower_goal.target_pose.pose.position.x = fiducial_array[1]; // x coordinate of fiducial marker
    follower_goal.target_pose.pose.position.y = fiducial_array[2]; // y coordinate of fiducial marker
    follower_goal.target_pose.pose.orientation.w = 1.0;
    while (!follower_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up for explorer");
    }
    follower_client.sendGoal(follower_goal);
    follower_client.waitForResult();


}

std::vector < std::vector <double>> USR::m_get_target_locations(){
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