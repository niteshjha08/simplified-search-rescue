#ifndef USR_H
# define USR_H

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



class USR {  
public:
    // main() will need to instantiate a ROS nodehandle, then pass it to the constructor
    USR(ros::NodeHandle* nodehandle);

    /**
     * @brief fetches the aruco_loopkup_locations from the YAML file and stores in the vector fiducial_array
     * 
     * @return std::vector < std::vector <double>> fiducial_array
     */
    std::vector < std::vector <double>> m_get_target_locations();

    /**
     * @brief Moves the bot to aruco lookup locations using MoveBaseClient
     * 
     * @param aruco_lookup_location target lookup location 
     */
    void m_move_to_target(std::vector<double> aruco_lookup_location);

    /**
     * @brief Moves the follower bot to a detected fiducial marker
     * Uses MoveBaseClient 
     * @param fiducial_array contains fiducial id, along with x,y coordinates of marker (with tolerance) in map frame
     */
    void follow(std::vector<double>fiducial_array);

    /**
     * @brief Searches for aruco marker at aruco lookup location by rotating at angular velocity of 0.15
     * Subscriber to \fiducial_transforms checks if aruco_detect node detects an aruco in camera frame, 
     * in which case the msg in subscriber callback becomes non-empty, and the member variable aruco_found is set to true and
     *  call to broadcast is made.
     */
    void m_search_at_target();

    /**
     * @brief Moves the explorer robot back to its starting location
     * 
     */
    void m_move_to_base();

    /**
     * @brief Moves the follower robot back to its starting location
     * 
     */
    void f_move_to_base();

    /**
     * @brief Performs sorting of the vector containing detected fiducial locations according to their fiducial IDs.
     * 
     * @return std::vector<std::vector<double>> sorted vector (with respect to fiducial id) of fiducial locations
     */
    std::vector<std::vector<double>> arrange(std::vector<std::vector<double>>fiducial_array);

    /**
     * @brief Variable storing state of search for explorer.
     * Is set to true when aruco is found at target location, false otherwise
     * 
     */
    bool m_aruco_found;

    /**
     * @brief Variable stores the value of fiducial id detected
     * Is initially set to NULL
     * 
     */
    int curr_fiducial_id;

    //tf Buffer for tflistener
    tf2_ros::Buffer tfBuffer;

    /**
     * @brief Transforms marker frame w.r.t the map frame and pushes the resulting location along with fiducial_id into fiducial_array
     * 
     * @param tfBuffer tf buffer
     */
    void listen(tf2_ros::Buffer& tfBuffer);

    /**
     * @brief Vector containing fiducial marker locations and their fiducial_id
     * 
     */
    std::vector < std::vector <double>>fiducial_array{};

private:
    // NodeHandle
    ros::NodeHandle m_nh;
    // Subscriber to /fiducial_transform
    ros::Subscriber m_aruco_detect_subscriber;
    // Publisher to /explorer/cmd_vel
    ros::Publisher m_explorer_rotator;
    
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    // MoveBaseClient explorer_client;
    MoveBaseClient explorer_client;
    // MoveBaseClient follower_client;
    MoveBaseClient follower_client;

    /**
     * @brief Define subscriber to /fiducial_transforms
     * 
     */
    void m_initialize_subscribers();

    /**
     * @brief Define publisher to explorer/cmd_vel for rotating the explorer for searching at the lookup locations 
     * 
     */
    void m_initialize_publishers();
    
    /**
     * @brief Define callback for subscriber to \fiducial_transforms topic.
     * Checks if the msg is empty (no aruco found) or not.
     * If the aruco is found, m_aruco_found is set to true and call to broadcast is made.
     * 
     * @param msg message on topic /fiducial_transform 
     */
    void m_aruco_detect_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);

    /**
     * @brief broadcast the frame marker_frame to /tf using the detected aruco position and orientations
     * 
     * @param msg message from /fiducial_transforms containing aruco information
     */
    void broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);

    // Variable to save aruco_lookup targets from the parameter server.
    XmlRpc::XmlRpcValue aruco_lookup_locations;

};

#endif