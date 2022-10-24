#include "myHeader.h"


NAVIGATION::NAVIGATION() : _ac("/move_base", true){
	_cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	_diff_drive_pub = _nh.advertise<std_msgs::Int32>("/diff_drive_topic",1);

	_aruco_sub = _nh.subscribe("/aruco_marker_publisher_turtlebot/markers", 1, &NAVIGATION::aruco_cb, this);
	_odom_sub  = _nh.subscribe("/odom", 1, &NAVIGATION::odom_cb, this);
	
	_ac.waitForServer();

	_marker_match = false;
	_task_completed = false;
	_last_room = false;
	_requested_marker = -1;
}


void NAVIGATION::aruco_cb(const aruco_msgs::MarkerArrayConstPtr &marker){
	_marker_id = marker->markers[0].id;
	if(_marker_id == _requested_marker){
		_marker_match = true;
	}
	else _marker_match = false;
}


void NAVIGATION::odom_cb(const nav_msgs::OdometryConstPtr &odom){
    _curr_pos.x = odom->pose.pose.position.x;
    _curr_pos.y = odom->pose.pose.position.y;

    tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, _curr_yaw);

}


void NAVIGATION::human_input(){
	string keyboard_input;
	cout << "Choose the tool:" << endl;
	cout<<"[1] for hammer" << endl << "[2] for drill" << endl << "[3] for screwdriver" << endl;
	getline(cin, keyboard_input);
	
	if( keyboard_input == "1") //2 room
		_requested_marker = 25;
	else if( keyboard_input == "2") //1 room
		_requested_marker = 26;
	else if( keyboard_input == "3") //3 room
		_requested_marker = 24;
	
}


bool NAVIGATION::navigation(float x_goal, float y_goal){
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.pose.position.x = x_goal;
	goal.target_pose.pose.position.y = y_goal;
	if(_last_room){
		goal.target_pose.pose.orientation.w = sqrt(2)/2;
		goal.target_pose.pose.orientation.z = sqrt(2)/2;
	}
	else {
		goal.target_pose.pose.orientation.w = 0;
		goal.target_pose.pose.orientation.z = -1;
	}
	
	_ac.sendGoal(goal);

	bool done = false;
	ros::Rate r(10);
	while(!done){
		if(_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || _ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
			done = true;
			cout << "Room reached" <<endl;
			return done;
		}
		r.sleep();
	}
	return false;
}


void NAVIGATION::ctrl_loop(){
	ros::Rate rate(10);

	double cumulative_yaw;
	double previous_curr_yaw;

	geometry_msgs::Twist cmd;

	geometry_msgs::Point room1, room2, room3, room4;
    room1.x = -3.5; room1.y = 2.5;
	room2.x = -3.5; room2.y = 5.5;
	room3.x = -3.5; room3.y = 8.5;
	room4.x = 2; 	room4.y = 4;

    _waypoints.push_back(room1);
    _waypoints.push_back(room2);
    _waypoints.push_back(room3);
    _waypoints.push_back(room4);
	_wp_index = 0;

	human_input();
		while(!_marker_match && _wp_index <= _waypoints.size()-2 ) {
			while(!navigation(_waypoints[_wp_index].x, _waypoints[_wp_index].y)){
				rate.sleep();
			}

			if(!_marker_match){ //if marker was not found right away, rotate on the spot
				cout<<"Tool not found. Rotating on the spot..." << endl;
				cumulative_yaw = 0;
				previous_curr_yaw = _curr_yaw;	

				cmd.angular.z = 0.3;
				_cmd_vel_pub.publish(cmd);

				while( !_marker_match && fabs(cumulative_yaw) < 2*M_PI ){
					cumulative_yaw = cumulative_yaw + fabs( fabs(_curr_yaw) - fabs(previous_curr_yaw) );
					previous_curr_yaw = _curr_yaw;
					rate.sleep();
				}

				cmd.angular.z = 0.0;
				_cmd_vel_pub.publish(cmd);

				if(!_marker_match){ //if marker was not found after the rotation, move to the next room
					cout << "Tool not found. Trying in another room..." << endl;
					_wp_index++;
				}
				else { //marker was found while rotating
					cout << "Tool found while rotating. Sleeping for 1 second..." << endl;
					ros::Duration(1).sleep();
					cout << "Awake" << endl;
				}
			}

			else { //ho trovato il marker
				cout <<"Tool found. Sleeping for 1 second..." << endl;
				ros::Duration(1).sleep();
				cout << "Awake" << endl;
			}
		}

		// go into Kuka's room
		cout << "Reaching kuka" << endl;
		_last_room = true;
		while(!navigation( _waypoints[_waypoints.size()-1].x, _waypoints[_waypoints.size()-1].y )){
			rate.sleep();
		}
		_task_completed = true;
		cout <<"Task completed." << endl;

	
}


void NAVIGATION::diff_drive_loop(){
	ros::Rate rate(10);
	std_msgs::Int32 marker, uncompleted;
	uncompleted.data = -1;

	while(ros::ok()){
		if(_task_completed){
			marker.data = _requested_marker;
			_diff_drive_pub.publish(marker);
		}
		else{
			_diff_drive_pub.publish(uncompleted);
		}
		rate.sleep();
	}

}


void NAVIGATION::run(){
    boost::thread ctrl_loop_t( &NAVIGATION::ctrl_loop, this);
	boost::thread diff_drive_loop_t( &NAVIGATION::diff_drive_loop, this);
    ros::spin();
}


int main(int argc, char **argv) {

	ros::init(argc, argv,"move_base_client");
	NAVIGATION nav;
	nav.run();

	return 0;
}
