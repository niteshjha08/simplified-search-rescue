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


bool aruco_found=false;
// Driver function for sorting
bool sortcol(const std::vector<double>& v1,
  const std::vector<double>& v2) {
  return (v1[0] < v2[0]);
}

std::vector < std::vector <double>>fiducial_array{};
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void move_to_base() {
  move_base_msgs::MoveBaseGoal explorer_goal;
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

void follow_marker(std::vector<double>fiducial_array) {
  // ROS_ASSERT(aruco_lookup_location.getType() == XmlRpc::XmlRpcValue::TypeArray);

  move_base_msgs::MoveBaseGoal follower_goal;
  follower_goal.target_pose.header.frame_id = "map";
  follower_goal.target_pose.header.stamp = ros::Time::now();
  follower_goal.target_pose.pose.position.x = fiducial_array[1];//
  follower_goal.target_pose.pose.position.y = fiducial_array[2];//
  follower_goal.target_pose.pose.orientation.w = 1.0;
  MoveBaseClient follower_client("/follower/move_base", true);
  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }
  follower_client.sendGoal(follower_goal);
  follower_client.waitForResult();

}

void search_at_target(ros::Publisher explorer_rotator){
    std::cout<<"inside search_at_target function"<<std::endl;
    aruco_found=false;
    geometry_msgs::Twist msg;
    msg.angular.z=0.1;
    int count=0;
    ros::Rate rate(5);
    while(!aruco_found){
        explorer_rotator.publish(msg);
        count+=1;
        // std::cout<<"count:"<<count<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    msg.angular.z=0.0;
    explorer_rotator.publish(msg);
}

void broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
  //for broadcaster
  // tf2_ros::TransformBroadcaster br;
  // static tf2_ros::StaticTransformBroadcaster br;   //works with this
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  //broadcast the new frame to /tf Topic
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
  transformStamped.child_frame_id = "marker_frame";
  
  transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
  transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
  transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z-0.5; // tolerance of 0.5m
  transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
  transformStamped.transform.rotation.y = msg->transforms[0].transform.rotation.y;
  transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;
  transformStamped.transform.rotation.w = msg->transforms[0].transform.rotation.w;
  ROS_INFO("Broadcasting");
  br.sendTransform(transformStamped);
  // ros::Duration(1.0).sleep();

}

void listen(tf2_ros::Buffer &tfBuffer,int fiducial_id) {
  //for listener
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  bool success=false;
  int count=0;
  double fiducial_ids{fiducial_id};
  // while(!success){

    try {
        transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));
        auto trans_x = transformStamped.transform.translation.x;
        auto trans_y = transformStamped.transform.translation.y;
        
        fiducial_array.push_back({fiducial_ids, trans_x,trans_y});
        
        ROS_INFO_STREAM("Position in map frame: ["
        << trans_x << ","
        << trans_y << ","
        <<" for marker : "<<fiducial_ids
        );
        aruco_found=true;
        // success=true;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
  // }
  
}

void mycallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
    static tf2_ros::Buffer tfBuffer;
    ROS_INFO("Callback called once!");
    // static tf2_ros::Buffer tfBuffer;
    if (!msg->transforms.empty() && msg->transforms[0].fiducial_area>10000){
        std::cout<<"fiducial found with id:"<<msg->transforms[0].fiducial_id<<" with area :"<<msg->transforms[0].fiducial_area<<'\n';
        
        broadcast(msg);
        listen(tfBuffer,msg->transforms[0].fiducial_id);
    }
}

int main(int argc,char **argv){
    
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
    
    move_to_base();

    std::cout<<"locations:";
    for(int i=0;i<4;++i){
        for(int j=0;j<3;++j)
            std::cout<<fiducial_array.at(i).at(j);
        std::cout<<"\n";
    }

    sort(fiducial_array.begin(), fiducial_array.end(), sortcol);

    for (int i{}; i < 4; i++) {
      for (int j{}; j < 3; j++) {
      std::cout << fiducial_array.at(i).at(j) << " ";
                                }
    std::cout << '\n';
                              }

    for (int i{};i < 4;i++) {
    follow_marker(fiducial_array[i]);
  }
    // ros::spin();

}