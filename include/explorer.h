#ifndef EXPLORER_H
# define EXPLORER_H

#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <nav_msgs/Odometry.h>    //for nav_msgs::Odometry
#include <ros/ros.h>
#include <utility>
#include <tf/transform_datatypes.h> //to manipulate quaternions
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>



class Explorer {
    
    public:
    // main() will need to instantiate a ROS nodehandle, then pass it to the constructor
    Explorer(ros::NodeHandle* nodehandle);
    std::vector < std::vector <double>> m_get_target_locations();
    void m_move_to_target(std::vector<double> aruco_lookup_location);
    void m_search_at_target();
    void m_move_to_base();

    bool m_aruco_found;
    int curr_fiducial_id;
    tf2_ros::Buffer tfBuffer;
    void listen(tf2_ros::Buffer& tfBuffer);
    // static tf2_ros::TransformBroadcaster br;
    // static tf2_ros::TransformListener tfListener(tf2_ros::Buffer& tfBuffer);

    std::vector < std::vector <double>>fiducial_array{};
    
    private:
    ros::NodeHandle m_nh;
    

    ros::Subscriber m_aruco_detect_subscriber;
    ros::Publisher m_explorer_rotator;
    // MoveBaseClient explorer_client;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    MoveBaseClient explorer_client;

    void m_initialize_subscribers();
    void m_initialize_publishers();
    // void m_initialize_movebase();

    void m_aruco_detect_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);
    
    
    void broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);

    XmlRpc::XmlRpcValue aruco_lookup_locations;
    
};

#endif