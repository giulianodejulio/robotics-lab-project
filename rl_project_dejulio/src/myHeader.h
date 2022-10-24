#include "ros/ros.h"

//move_base libraries
#include <move_base_msgs/MoveBaseAction.h> 
#include <actionlib/client/simple_action_client.h>

//aruco library
#include <aruco_msgs/MarkerArray.h>

//thread library
#include "boost/thread.hpp" //to have the control loop running

#include <iostream>

#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h" //to command the robot in velocity
#include "nav_msgs/Odometry.h" //to read the robot pose
#include "tf/tf.h" //to use the tf package (specifically, I want to use getRPY() )


//MoveIt Libraries
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std;


class NAVIGATION {
    public:
        NAVIGATION();
        void run(); //used to start all the parallel functions of my system, including callbacks
        
        void aruco_cb(const aruco_msgs::MarkerArrayConstPtr &marker);
        void odom_cb(const nav_msgs::OdometryConstPtr &odom);
        
        bool navigation(float x_goal, float y_goal); //It outputs 1 when the next waypoint is reached
        void human_input();

        //Threads
        void ctrl_loop();
        void diff_drive_loop();      
        
        //int getMarkerId(){return this->_marker_id;}
        //int getRequestedMarker(){return this->_requested_marker;}

    private:
        
        ros::NodeHandle _nh;
        
        geometry_msgs::Point _curr_pos;
        double _curr_yaw;
        
        int _marker_id;
        int _requested_marker;
        bool _marker_match;

        bool _last_room;
        bool _task_completed;

        int _wp_index;
        vector<geometry_msgs::Point> _waypoints; //rooms' centers' coordinates

        ros::Publisher _cmd_vel_pub;
        ros::Publisher _diff_drive_pub;

        ros::Subscriber _aruco_sub;
        ros::Subscriber _odom_sub;

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> _ac;

};


class MOVEIT_CLASS{
    public:
        MOVEIT_CLASS();
        void run();

        void aruco_cb(const aruco_msgs::MarkerArrayConstPtr &marker);
        void diff_drive_cb(std_msgs::Int32 req_marker); //usata per ricevere un intero che è il marker richiesto dall'utente


        void ctrl_loop();


    private:

        ros::NodeHandle _nh;

        int _marker_id;
        int _requested_marker;
        geometry_msgs::Pose _marker_pose;
        
        bool _enable_motion;
        bool _stop_motion;
        bool _exit_from_loop;

        moveit::planning_interface::MoveGroupInterface _move_group_interface;
        const moveit::core::JointModelGroup* _joint_model_group;
        moveit::core::RobotStatePtr _current_state;
        std::vector<double> _joint_group_positions;

        ros::Subscriber _aruco_sub;
        ros::Subscriber _diff_drive_sub;

};
