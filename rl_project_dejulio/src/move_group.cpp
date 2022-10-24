#include "myHeader.h"


static const std::string PLANNING_GROUP = "kuka_iiwa_arm";



MOVEIT_CLASS::MOVEIT_CLASS() : _move_group_interface(PLANNING_GROUP) {
    //move to default position specified in the moveit wizard (look_down_pose)
    
    _move_group_interface.setJointValueTarget(_move_group_interface.getNamedTargetValues("look_down_pose"));
    _move_group_interface.asyncMove(); // a differenza di move(), asyncMove() non è bloccante

    _aruco_sub = _nh.subscribe("/kuka_iiwa/aruco_marker_publisher_kuka_iiwa/markers", 1, &MOVEIT_CLASS::aruco_cb, this);
    _diff_drive_sub = _nh.subscribe("/diff_drive_topic", 1, &MOVEIT_CLASS::diff_drive_cb, this);
    _enable_motion = false;
    _stop_motion = false;
    _exit_from_loop = false;
    
}


void MOVEIT_CLASS::aruco_cb(const aruco_msgs::MarkerArrayConstPtr &marker){
	_marker_id = marker->markers[0].id;
	//cout << "Marker Id:" << _marker_id<<endl;
	//ROS_INFO("id marker: %d", _marker_id);
    if(_marker_id == _requested_marker){
        _stop_motion = true;
    }
    else _stop_motion = false;

    //cout<<"_stop_motion: "<<_stop_motion<<endl;
    _marker_pose.position.x = marker->markers[0].pose.pose.position.x;
    _marker_pose.position.y = marker->markers[0].pose.pose.position.y;
    //_marker_pose.position.z = marker->markers[0].pose.pose.position.z;
    //cout << "_marker_pose.position.x: "<<_marker_pose.position.x<<endl;
    //cout << "_marker_pose.position.y: "<<_marker_pose.position.y<<endl;
    //cout << "_marker_pose.position.z: "<<_marker_pose.position.z<<endl<<endl;
    //_marker_pose.orientation.y = -sqrt(2)/2;
    //_marker_pose.orientation.w = sqrt(2)/2;

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
    while(ros::ok()){
        if(_enable_motion && !_stop_motion){
            
            //ruota il giunto 3. Mentre ruota, fermati se trovi il marker
            _current_state = _move_group_interface.getCurrentState();
            _joint_model_group = _move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
            _current_state->copyJointGroupPositions(_joint_model_group, _joint_group_positions);
            _joint_group_positions[2] = 2.9670;  // turn joint 3 from -2.9670 rad to 2.9670 rad
            _move_group_interface.setJointValueTarget(_joint_group_positions);
            
            _move_group_interface.asyncMove();
            //_move_group_interface.move();
            //ros::Duration(0.5).sleep();
            //se trovi il marker, fermati
            
            while(!_exit_from_loop){
                if(_stop_motion){
                    //cout<<"entrato nell'if"<<endl;
                    _move_group_interface.stop();
                    
                    _exit_from_loop = true;
                }
                rate.sleep();
            }
            //cout<<"ciao"<<endl;
            //esegui il moto per arrivare al marker
            geometry_msgs::Pose goal_pose;
            goal_pose.position.x = _marker_pose.position.x;
            goal_pose.position.y = _marker_pose.position.y;
            goal_pose.position.z = 0.1;
            goal_pose.orientation.w = 0.0;
            goal_pose.orientation.x = 0.0;
            goal_pose.orientation.y = -1.0;
            goal_pose.orientation.z = 0.0;
            _move_group_interface.setPoseReferenceFrame("map");
            _move_group_interface.setPoseTarget(goal_pose);
            _move_group_interface.move();
            //cout<<"ciao1"<<endl;
            /*
            std::vector<geometry_msgs::Pose> waypoints;
            
            //_move_group_interface.setPoseReferenceFrame("map");

            waypoints.push_back(_marker_pose);
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = _move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            _move_group_interface.execute(trajectory);
            
            
            //moveit::core::RobotState start_state(*_move_group_interface.getCurrentState());           
            //_move_group_interface.setStartState(start_state);
            
            
            //_move_group_interface.setPoseTarget(_marker_pose);

            */
            
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
    //ros::AsyncSpinner spinner(1);
    //spinner.start();



    /*
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    move_group_interface.setPoseTarget(target_pose1);*/
    
    /*
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    
    move_group_interface.move();*/
    
    

    return 0;
}