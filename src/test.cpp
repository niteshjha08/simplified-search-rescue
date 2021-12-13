#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <ros/ros.h>
#include <array>
bool aruco_found = false;
bool sortcol(const std::vector<double>& v1,
  const std::vector<double>& v2) {
  return (v1[0] < v2[0]);
}
// std::array<std::array<double,3>,4> marker_positions {};
std::vector < std::vector <double>>fiducial_array{};
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void base() {
  move_base_msgs::MoveBaseGoal explorer_goal;
  ROS_INFO("Moving back to base bitches!!!!!  ");
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  explorer_goal.target_pose.pose.position.x = -4.0;//
  explorer_goal.target_pose.pose.position.y = 2.5;//
  explorer_goal.target_pose.pose.orientation.w = 1.0;
  MoveBaseClient explorer_client("/explorer/move_base", true);
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  explorer_client.sendGoal(explorer_goal);
  explorer_client.waitForResult();

}
void move_to_target(XmlRpc::XmlRpcValue aruco_lookup_location) {
  ROS_ASSERT(aruco_lookup_location.getType() == XmlRpc::XmlRpcValue::TypeArray);

  move_base_msgs::MoveBaseGoal explorer_goal;
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  explorer_goal.target_pose.pose.position.x = aruco_lookup_location[0];//
  explorer_goal.target_pose.pose.position.y = aruco_lookup_location[1];//
  explorer_goal.target_pose.pose.orientation.w = 1.0;
  MoveBaseClient explorer_client("/explorer/move_base", true);
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }
  explorer_client.sendGoal(explorer_goal);
  explorer_client.waitForResult();

}

void search_at_target(ros::Publisher explorer_rotator){
    aruco_found=false;
    geometry_msgs::Twist msg;
    msg.angular.z=0.1;
    int count=0;
    while(!aruco_found){
        explorer_rotator.publish(msg);
        count+=1;
        // std::cout<<"count:"<<count;
        ros::spinOnce();
    }
    msg.angular.z=0.0;
    explorer_rotator.publish(msg);
}

void broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
  //for broadcaster
  if (!msg->transforms.empty()) {
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    //broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame";

    transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z;
    transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
    transformStamped.transform.rotation.y = msg->transforms[0].transform.rotation.y;
    transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;
    transformStamped.transform.rotation.w = msg->transforms[0].transform.rotation.w;

    //   ROS_INFO("Broadcasting");
    br.sendTransform(transformStamped);
  }
  }

void listen(tf2_ros::Buffer& tfBuffer, int fiducial_id) {
  //for listener
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  bool success = false;
  double new_fiducial_id{};
  new_fiducial_id = fiducial_id / 1.0;
  while (!success) {
      try {
      transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));
      auto trans_x = transformStamped.transform.translation.x;
        auto trans_y = transformStamped.transform.translation.y;
        auto trans_z = transformStamped.transform.translation.z;
        fiducial_array.push_back({new_fiducial_id, trans_x,trans_y,trans_z });
        // marker_positions.at(fiducial_id).at(0) = trans_x;
        // marker_positions.at(fiducial_id).at(1)=trans_x;
        // marker_positions.at(fiducial_id).at(2)=trans_x;
        
        ROS_INFO_STREAM("Position in map frame: ["
        << trans_x << ","
        << trans_y << ","
        << trans_z << "]"
        <<" for marker : "<<fiducial_id;
        success=true;
        );
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
  }
}
void mycallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
  // ROS_INFO("Callback called once!");
  tf2_ros::Buffer tfBuffer;


  if (!msg->transforms.empty()) {
    if (msg->transforms[0].fiducial_area >= 30000){
      aruco_found = true;
        broadcast(msg);
        std::cout << "Fiducial area is " << msg->transforms[0].fiducial_area;
        listen(tfBuffer, msg->transforms[0].fiducial_id);
    }
  
  }


}
// void base() {
//   move_base_msgs::MoveBaseGoal explorer_goal;
//   explorer_goal.target_pose.header.frame_id = "map";
//   explorer_goal.target_pose.header.stamp = ros::Time::now();
//   explorer_goal.target_pose.pose.position.x = -4;//
//   explorer_goal.target_pose.pose.position.y = 2.5;//
//   explorer_goal.target_pose.pose.orientation.w = 1.0;
//   MoveBaseClient explorer_client("/explorer/move_base", true);


//   explorer_client.sendGoal(explorer_goal);
//   explorer_client.waitForResult();

// }

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  ros::Subscriber aruco_listener = nh.subscribe("/fiducial_transforms", 1, mycallback);
  ros::Publisher explorer_rotator = nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 1000);
  XmlRpc::XmlRpcValue aruco_lookup_locations;

  // Go to target location 1 and execute search
  nh.getParam("/aruco_lookup_locations/target_1", aruco_lookup_locations);
  move_to_target(aruco_lookup_locations);
  search_at_target(explorer_rotator);

  nh.getParam("/aruco_lookup_locations/target_2", aruco_lookup_locations);
  move_to_target(aruco_lookup_locations);
  search_at_target(explorer_rotator);

  nh.getParam("/aruco_lookup_locations/target_3", aruco_lookup_locations);
  move_to_target(aruco_lookup_locations);
  search_at_target(explorer_rotator);

  nh.getParam("/aruco_lookup_locations/target_4", aruco_lookup_locations);
  move_to_target(aruco_lookup_locations);
  search_at_target(explorer_rotator);
  // base();
  // move_base_msgs::MoveBaseGoal explorer_goal;
  // ROS_INFO("Moving back to base bitches!!!!!  ");
  // explorer_goal.target_pose.header.frame_id = "map";
  // explorer_goal.target_pose.header.stamp = ros::Time::now();
  // explorer_goal.target_pose.pose.position.x = -4;//
  // explorer_goal.target_pose.pose.position.y = 2.5;//
  // explorer_goal.target_pose.pose.orientation.w = 1.0;
  // MoveBaseClient explorer_client("/explorer/move_base", true);
  // explorer_client.sendGoal(explorer_goal);
  // explorer_client.waitForResult();

  std::cout << "locations:";
  for (int i{}; i < 4 ; i++) {
    for (int j{}; j < 4; j++) {
      std::cout << fiducial_array.at(i).at(j) << " ";
      
    }
    std::cout<<'\n';
  }
  sort(fiducial_array.begin(), fiducial_array.end(), sortcol);
  for (int i{}; i < 4; i++) {
    for (int j{}; j < 4; j++) {
      std::cout << fiducial_array.at(i).at(j) << " ";

    }
    std::cout << '\n';
  }
  

  // ros::spin();

}