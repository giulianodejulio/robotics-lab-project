#include "myHeader.h"


static const std::string PLANNING_GROUP = "kuka_iiwa_arm";


MOVEIT_CLASS::MOVEIT_CLASS() : _move_group_interface(PLANNING_GROUP) {

    //move to default position specified in the moveit wizard (look_down_pose)
    _move_group_interface.setPoseReferenceFrame("map");
    _move_group_interface.setJointValueTarget(_move_group_interface.getNamedTargetValues("look_down_pose"));
    _move_group_interface.asyncMove(); // a differenza di move(), asyncMove() is not blocking

    _aruco_sub = _nh.subscribe("/kuka_iiwa/aruco_marker_publisher_kuka_iiwa/markers", 1, &MOVEIT_CLASS::aruco_cb, this);
    _diff_drive_sub = _nh.subscribe("/diff_drive_topic", 1, &MOVEIT_CLASS::diff_drive_cb, this);

    _enable_motion = false;
    _stop_motion = false;
    _exit_from_loop = false;
    _task_completed = false;
}


void MOVEIT_CLASS::aruco_cb(const aruco_msgs::MarkerArrayConstPtr &marker){
	_marker_id = marker->markers[0].id;
    if(_marker_id == _requested_marker){
        _stop_motion = true;
    }
    else _stop_motion = false;

    _marker_pose.position.x = marker->markers[0].pose.pose.position.x;
    _marker_pose.position.y = marker->markers[0].pose.pose.position.y;

}


void MOVEIT_CLASS::diff_drive_cb(std_msgs::Int32 req_marker){ //ci entra
    if(req_marker.data > 0){
        _enable_motion = true;
        _requested_marker = req_marker.data;
    }
    else _enable_motion = false;
}



void MOVEIT_CLASS::ctrl_loop(){

    ros::Rate rate(10);
    while(ros::ok() && !_task_completed){
        if(_enable_motion && !_stop_motion){
            
            //rotate joint 3. While rotating, stop if marker is found
            _current_state = _move_group_interface.getCurrentState();
            _joint_model_group = _move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
            _current_state->copyJointGroupPositions(_joint_model_group, _joint_group_positions);
            _joint_group_positions[2] = 2.9670;  // turn joint 3 from -2.9670 rad to 2.9670 rad
            _move_group_interface.setJointValueTarget(_joint_group_positions);
            
            _move_group_interface.asyncMove();
            //_move_group_interface.move();

            //stop if marker is found
            while(!_exit_from_loop){
                if(_stop_motion){
                    _move_group_interface.stop();
                    _exit_from_loop = true;
                }
                rate.sleep();
            }

            //execute the motion to reach the marker
            geometry_msgs::Pose goal_pose;
            goal_pose.position.x = _marker_pose.position.x;
            goal_pose.position.y = _marker_pose.position.y;
            goal_pose.position.z = 0.1;
            goal_pose.orientation.w = 0.0;
            goal_pose.orientation.x = 0.0;
            goal_pose.orientation.y = -1.0;
            goal_pose.orientation.z = 0.0;

            //_move_group_interface.setPoseReferenceFrame("map");
            _move_group_interface.setPoseTarget(goal_pose);
            _move_group_interface.move();
            _move_group_interface.stop();
            _task_completed = true;
            
        }
        
    }
    rate.sleep();
}


void MOVEIT_CLASS::run(){
    boost::thread ctrl_loop_t( &MOVEIT_CLASS::ctrl_loop, this);
    ros::spin();
}


int main(int argc, char** argv){
    ros::init(argc, argv, "move_group_main");
    MOVEIT_CLASS mov;
    mov.run(); 

    return 0;
}