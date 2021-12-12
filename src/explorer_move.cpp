

#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
bool aruco_found = false;
void base() {
  move_base_msgs::MoveBaseGoal explorer_goal;
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  explorer_goal.target_pose.pose.position.x = -4;//
  explorer_goal.target_pose.pose.position.y = 2.5;//
  explorer_goal.target_pose.pose.orientation.w = 1.0;
  MoveBaseClient explorer_client("/explorer/move_base", true);


  explorer_client.sendGoal(explorer_goal);
  explorer_client.waitForResult();

}
void move(XmlRpc::XmlRpcValue aruco_lookup_location) {
  ROS_ASSERT(aruco_lookup_location.getType() == XmlRpc::XmlRpcValue::TypeArray);

  move_base_msgs::MoveBaseGoal explorer_goal;
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  explorer_goal.target_pose.pose.position.x = aruco_lookup_location[0];//
  explorer_goal.target_pose.pose.position.y = aruco_lookup_location[1];//
  explorer_goal.target_pose.pose.orientation.w = 1.0;
  MoveBaseClient explorer_client("/explorer/move_base", true);


  explorer_client.sendGoal(explorer_goal);
  explorer_client.waitForResult();

}
void mycallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
{
  // std::cout << "Fiducial callback called!";
  if (!msg->transforms.empty()) {//check marker is detected
  //broadcaster object
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    //broadcast the new frame to /tf Topic
    ROS_INFO("marker detected !!!!!");
    ROS_INFO("Now moving to next aruco");

    aruco_found = true;
    transformStamped.header.stamp = ros::Time::now();
    
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame"; //name of the frame
    transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
      /*write the remaining code here*/
      br.sendTransform(transformStamped); //broadcast the transform on /tf Topic
  }
}

// void mycallback(fiducial_msgs::FiducialTransformArray &fiducial_transform) {

//   if (fiducial_transform.transforms. != 0) {
//     std::cout << "ARUCO FOUND!!!!";
//     aruco_found = true;
//   }
// }

int main(int argc, char** argv) {
  ros::init(argc, argv, "explorer_navigator");
  ros::NodeHandle nh;
  geometry_msgs::Twist msg;



  ros::Publisher explorer_rotator = nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 1000);
  // ros::Publisher trial = nh.advertise<std_msgs::String>("/explorer/cmd_vel", 1000);
  ros::Subscriber aruco_listener = nh.subscribe("/fiducial_transforms", 1000, mycallback);

  MoveBaseClient explorer_client("/explorer/move_base", true);

  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }
  // Retrieving target coordinates from parameter server
  XmlRpc::XmlRpcValue aruco_lookup_locations_1;
  XmlRpc::XmlRpcValue aruco_lookup_locations_2;
  XmlRpc::XmlRpcValue aruco_lookup_locations_3;
  XmlRpc::XmlRpcValue aruco_lookup_locations_4;
  XmlRpc::XmlRpcValue aruco_lookup_locations_5;

  nh.getParam("/aruco_lookup_locations/target_1", aruco_lookup_locations_1);
  std::cout << "Goal from server is:" << aruco_lookup_locations_1;

  // Build goal for explorer
  move(aruco_lookup_locations_1);
    std::cout << "Reached target_1!";

  // Rotate until aruco is found
  // Twist msg;
  msg.angular.z = 0.1;
  
    while (!aruco_found) {

      std::cout << "Rotating, searching for ARUCO marker";
      explorer_rotator.publish(msg);
      ros::spinOnce();
    }
  msg.angular.z = 0;
  explorer_rotator.publish(msg);


// Now moving to next location
  nh.getParam("/aruco_lookup_locations/target_2", aruco_lookup_locations_2);
  std::cout << "Goal from server is:" << aruco_lookup_locations_2;
  move(aruco_lookup_locations_2);
  // rotating
  




  
  while (!aruco_found) {
    std::cout << "Rotating, searching for ARUCO marker";
    explorer_rotator.publish(msg);
    ros::spinOnce();
  }
  
  nh.getParam("/aruco_lookup_locations/target_3", aruco_lookup_locations_3);
  std::cout << "Goal from server is:" << aruco_lookup_locations_3;
// Build goal for explorer
  move(aruco_lookup_locations_3);
  while (!aruco_found) {

    std::cout << "Rotating, searching for ARUCO marker";
    explorer_rotator.publish(msg);
    ros::spinOnce();
  }

  // Moving to final marker
  nh.getParam("/aruco_lookup_locations/target_4", aruco_lookup_locations_4);
  std::cout << "Goal from server is:" << aruco_lookup_locations_4;
  // Build goal for explorer
  move(aruco_lookup_locations_4);
  while (!aruco_found) {

    std::cout << "Rotating, searching for ARUCO marker";
    explorer_rotator.publish(msg);
    ros::spinOnce();
  }
  base();

}
