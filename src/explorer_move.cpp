#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv){
    ros::init(argc, argv,"explorer_navigator");
    ros::NodeHandle nh;

    MoveBaseClient explorer_client("/explorer/move_base", true);

    while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }
    // Retrieving target coordinates from parameter server
    XmlRpc::XmlRpcValue aruco_lookup_locations_1;
    nh.getParam("/aruco_lookup_locations/target_1", aruco_lookup_locations_1);
    ROS_ASSERT(aruco_lookup_locations_1.getType() == XmlRpc::XmlRpcValue::TypeArray);

    std::cout<<"Goal from server is:"<<aruco_lookup_locations_1;

    
    //Build goal for explorer
    move_base_msgs::MoveBaseGoal explorer_goal;
    explorer_goal.target_pose.header.frame_id = "map";
    explorer_goal.target_pose.header.stamp = ros::Time::now();
    explorer_goal.target_pose.pose.position.x = aruco_lookup_locations_1[0];//
    explorer_goal.target_pose.pose.position.y = aruco_lookup_locations_1[1];//
    explorer_goal.target_pose.pose.orientation.w = 1.0;
    
    explorer_client.sendGoal(explorer_goal);
    explorer_client.waitForResult();
    std::cout<<"Reached target_1!";

    // std::cout<<"Rotating now:";
    // explorer_goal.target_pose.header.frame_id = "map";
    // explorer_goal.target_pose.header.stamp = ros::Time::now();
    // explorer_goal.target_pose.pose.orientation.z = 0.1;
    // explorer_client.sendGoal(explorer_goal);
    // std::cout<<"Rotated !!!";

    



    
}