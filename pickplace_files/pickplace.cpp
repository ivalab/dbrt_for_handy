/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */
/* Modified by: Fu-Jen Chu, Ruinian Xu, Michael E. Cao*/

#include <ros/ros.h>
#include <ros/duration.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <dynamixel_msgs/JointState.h>

#include <time.h>
#include <unistd.h>
#include <math.h>
#include <unistd.h>
#include <cstdlib> 
#include <iostream>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions> 
#include <tf/tf.h>
#include <string> //for instantiating CORRECTION publishers

// include headers for moveit group
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

double time_pick = 0.0;
double time_place = 0.0;

void getCommandFromMeta(std::string &command, std::string &x, std::string &y);
void getCommandFromMeta_commandOnly(std::string &command);
void getCommandFromMeta_locationOnly(std::string &x, std::string &y);
std::vector<double> current_joint_values;
std::vector<double> estimated_joint_values; // CORRECTIONS
std::vector<double> desired_correction; // CORRECTIONS
bool mutex = 0;
double load_8 = 0;

const double pi = std::acos(-1);
//const double pi = 3.1415926535897;
const double gripper_open_value = -0.7; //open value of 8th motor
const double gripper_close_value = 0.2; //close value of 8th motor
const double paw_torque_compensate = 0.1;
const double slark_2nd_motor = 0.420755383;

void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg);
void est_joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg); // updating estimated pose
void execute_with_correction(moveit::planning_interface::MoveGroup::Plan plan, std::vector<ros::Publisher> &pub_1to6, ros::Publisher &pub_7, double correct_coeff); // execute trajectory with correction
void traj_end_mutex(const std_msgs::Bool::ConstPtr& msg);
void update_state(const dynamixel_msgs::JointState::ConstPtr& msg);
void mutex_traj();
void mutex_rotate(double target_angle);
void openGrabber(ros::Publisher &pub_8, ros::Publisher &pub_9);
void closeGrabber(ros::Publisher &pub_8, ros::Publisher &pub_9);
double rotateGripper(ros::Publisher &pub_7, double angle);
void pickup_place(moveit::planning_interface::MoveGroup &group, ros::Publisher &pub_7, ros::Publisher &pub_8, ros::Publisher &pub_9, bool time_count, bool do_correction, double correction_coeff, std::vector<ros::Publisher> &pub_1to6);
const std::vector<std::string> listNamedPoses(moveit::planning_interface::MoveGroup &group);
void gotoNamedTarget(moveit::planning_interface::MoveGroup &group, std::string target, bool constraint_on);
void addObject2Scene(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher);
void getConstraint(moveit::planning_interface::MoveGroup &group, std::string target);
void adjust(geometry_msgs::Pose &target_pose);

int main(int argc, char **argv)
{

    /*****************************************************************
    *                         Arm initialization                     *
    *****************************************************************/
    // ros initialization
    ros::init(argc, argv, "arm_kinematics");
    ros::NodeHandle node_handle;  
    ros::Publisher pub_7 = node_handle.advertise<std_msgs::Float64>("/finalarm_position_controller_7/command", 1, true);
    ros::Publisher pub_8 = node_handle.advertise<std_msgs::Float64>("/finalarm_position_controller_8/command", 1, true);
    ros::Publisher pub_9 = node_handle.advertise<std_msgs::Float64>("/finalarm_position_controller_9/command", 1, true);
    ros::Subscriber sub_multi_joint_states = node_handle.subscribe<sensor_msgs::JointState>("/joint_states", 1, joint_state_handler);
    ros::Subscriber sub_est_joint_states = node_handle.subscribe<sensor_msgs::JointState>("/estimated/joint_states", 1, est_joint_state_handler);
    ros::Subscriber sub_endtime = node_handle.subscribe<std_msgs::Bool>("/finalarm_joint_trajectory_action_controller/mutex", 5, traj_end_mutex);
    ros::Subscriber sub_joint_8_state = node_handle.subscribe<dynamixel_msgs::JointState>("/finalarm_position_controller_8/state", 1, update_state);


    ros::AsyncSpinner spinner(1);
    spinner.start();

    // kinematic_state & kinematic_model loading & planner
    moveit::planning_interface::MoveGroup group_arm("arm");
    // moveit::planning_interface::MoveGroup group_actuator("arm4_full");
    group_arm.setPlannerId("BKPIECEkConfigDefault");//ForageRRTkConfigDefault//LBKPIECEkConfigDefault//RRTstarkConfigDefault//BKPIECEkConfigDefault//RRTstarkConfigDefault
    // relationship is as follow
    // robot_state::RobotStatePtr kinematic_state:       group.getCurrentState() 
    // robot_model::RobotModelPtr kinematic_model:       group.getCurrentState()->getRobotModel()
    // robot_state::JointModelGroup* joint_model_group:  group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName())
    
    // We will use the planning_scene_interface class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 

    // clear commands from META
    //std::ofstream myfileOut ("/home/fujenchu/projects/robotArm/socket/commands.txt", std::ios::trunc);
    //myfileOut << ""; myfileOut.close();

    /*****************************************************************
    *                         ROS info output                        *
    *****************************************************************/
    // get the joint names to verify if loaded successfully 
    const std::vector<std::string> &joint_names = group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName())->getJointModelNames();
    //std::cout << "Loading model..  joints in arm model:" << std::endl; 
    //for (size_t i = 0; i < 8 ; i++) std::cout << joint_names[i] << std::endl;

    const std::vector<std::string> &link_names = group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName())->getLinkModelNames();
    //for (size_t i = 0; i < 8 ; i++) std::cout << link_names[i] << std::endl;

    ROS_INFO("Model frame: %s", group_arm.getCurrentState()->getRobotModel()->getModelFrame().c_str());
    ROS_INFO("Reference Planning frame: %s", group_arm.getPlanningFrame().c_str());
    ROS_INFO("EndEffectorLink simulator: %s", group_arm.getEndEffectorLink().c_str());
    //ROS_INFO("EndEffectorLink actuator: %s", group_actuator.getEndEffectorLink().c_str());
    //ROS_INFO("EndEffector: %s", group.getEndEffector().c_str());

    /*****************************************************************
    *                       Visualization setup                      *
    *****************************************************************/
    // for visualization
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    /*****************************************************************
    *                     Adding objects to world                    *
    *****************************************************************/
    /* First put an object into the scene*/
    /* Advertise the collision object message publisher*/
    ros::Publisher collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    while(collision_object_publisher.getNumSubscribers() < 1) {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }
    addObject2Scene(group_arm, planning_scene_interface ,collision_object_publisher);
  
    /*****************************************************************
    *                        List stored poses                       *
    *****************************************************************/
    // list the pre-defined pose in SRDF file
    const std::vector<std::string> namedTargets = listNamedPoses(group_arm);

    /*****************************************************************
    *                      Specify initial pose                      *
    *****************************************************************/
    // namedTargets stores the names of pre-defined poses(joint values)
    // select the name (ex. "home"), gotoNamedTarget function will find plan to the specific pose

    std::string target = ""; 
    int targetNum = 0;
    std::cout<<"select target pose above: "; std::cin >> targetNum;
    if(targetNum == 0) target = "Home";
    else target = "Home";
    gotoNamedTarget(group_arm, target, 0);

    /*
    std::string command = "";
    std::string pos_x = "";
    std::string pos_y = "";
    std::cout<< "Handy is waiting for command! (you might need to run the server)"<<std::endl;
    */

    bool time_count = false;
    std::cout<<"Would you like to count the time for pick and place? Type:0(No) 1(Yes)"<<std::endl;
    std::cin>>time_count;

    // CORRECTION STUFF
    std::vector<ros::Publisher> pub_1to6;
    bool correct = false;
    std::cout<<"Would you like to apply corrections for pick and place? Type:0(No) 1(Yes)"<<std::endl;
    std::cin>>correct;
    
    double correction_coeff = 0.0;
    if(correct){
        // if correction desired, get coefficient
        std::cout<<"Enter the desired correction coefficient:"<<std::endl;
    	std::cin>>correction_coeff;
    }
    for(int i = 1; i <= 6; i++){
        pub_1to6.push_back(node_handle.advertise<std_msgs::Float64>("/finalarm_position_controller_" + std::to_string(i) + "/command", 1, true));
    }
    // END CORRECTION STUFF
    
    int flag = 1;
    while (flag) {
    
        /*****************************************************************
        *                 Attempt to pick up and place                   *
        *****************************************************************/
        // specify the target_pose
        // the robot will attempt to:
        // 1. go above the object
        // 2. open gripper
        // 3. rotate gripper
        // 4. go down
        // 5. close gripper 
        // 6. lift gripper a little bit
        // 7. go to place the object
        // 8. go back to home position
        pickup_place(group_arm, pub_7, pub_8, pub_9, time_count, correct, correction_coeff, pub_1to6);

        gotoNamedTarget(group_arm, target, 0);

        std::cout<<"Would you like to continue picking and placing items? 0.No 1.Yes"<<std::endl;
        std::cin >> flag;
    }

    ros::spin();
    return 0;
}

void getCommandFromMeta(std::string &command, std::string &x, std::string &y){
    std::ifstream myfileIn ("/home/fujenchu/projects/robotArm/socket/commands.txt");
    getline (myfileIn,command);

    if(command != ""){ // got sth
        getline (myfileIn,x);
        getline (myfileIn,y);
        std::ofstream myfileOut ("/home/fujenchu/projects/robotArm/socket/commands.txt", std::ios::trunc);
        myfileOut << "";
        myfileOut.close();
        std::cout << "receive a command = " << command << std::endl; 
        std::cout<<"x = "<<x<<std::endl;
        std::cout<<"y = "<<y<<std::endl;
    }
    myfileIn.close();
}

void getCommandFromMeta_commandOnly(std::string &command){
    std::ifstream myfileIn ("/home/fujenchu/projects/robotArm/socket/commands.txt");
    getline (myfileIn,command);

    if(command != ""){ // got sth
        std::cout << "receive a command = " << command << std::endl;
        std::ofstream myfileOut ("/home/fujenchu/projects/robotArm/socket/commands.txt", std::ios::trunc);
        myfileOut << "";
        myfileOut.close();
    }
    myfileIn.close();
}

void getCommandFromMeta_locationOnly(std::string &x, std::string &y){
    std::ifstream myfileIn ("/home/fujenchu/projects/robotArm/toy-opencv-mat-socket-server-master_pcl/bbs/locations.txt");

    getline (myfileIn,x);
    getline (myfileIn,y);
    std::cout<<"x = "<<x<<std::endl;
    std::cout<<"y = "<<y<<std::endl;
    
    myfileIn.close();
}

void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg)
{
    current_joint_values.clear();
    for(std::size_t i = 0; i < msg->position.size(); ++i) {
        current_joint_values.push_back(msg->position[i]);
    }
}

// CORRECTION STUFF
void est_joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg)
{
    estimated_joint_values.clear();
    // the first six joints may be camera state estimation, which is not needed for the correction
    // If camera is being estimated, initialize i = 6, otherwise i = 0
    for(std::size_t i = 6; i < msg->position.size(); ++i) { 
        estimated_joint_values.push_back(msg->position[i]);
    }
}
// END CORRECTION STUFF

void traj_end_mutex(const std_msgs::Bool::ConstPtr& msg){
    if (msg->data){
        mutex = 1;
    }
}

void update_state(const dynamixel_msgs::JointState::ConstPtr& msg){
    //update load for 8th motor
    load_8 = msg->load;
}

void mutex_traj(){
    clock_t start_time = clock();
    while(1){
        if(mutex){
            break;
        }
        else if ((clock() - start_time) / CLOCKS_PER_SEC > 6.0)
            break;
    }
    mutex = 0;
}

void mutex_rotate(double target_angle){
    clock_t start_time = clock();
    while(1){
        if (std::abs(target_angle - current_joint_values[6]) < 0.01)
            break;
        else if ((clock() - start_time) / CLOCKS_PER_SEC > 4.0)
            break;
    }
}

void openGrabber(ros::Publisher &pub_8, ros::Publisher &pub_9){
    std_msgs::Float64 val_8;
    std_msgs::Float64 val_9;
    val_8.data = gripper_open_value;
    val_9.data = -gripper_open_value;
    pub_8.publish(val_8);
    pub_9.publish(val_9);
    sleep(1.5);
}

double rotateGripper(ros::Publisher &pub_7, double angle) {
    std_msgs::Float64 val_7;

    std::cout<<" current value of seventh joint: "<<current_joint_values[6]<<std::endl;
    std::cout<<" compensate value: "<<std::abs(angle - 180.00) / 180.00 * pi<<std::endl;
    double compensation = 0.0;

    if ((current_joint_values[6] + (angle / 90.00 * pi / 2)) > 1.8){
        compensation = -(std::abs(angle - 180.00) / 180.00 * pi);
    }
    else if ((current_joint_values[6] + (angle / 90.00 * pi / 2)) < -1.8){
        compensation = (std::abs(angle - 180.00) / 180.00 * pi);
    }
    else{
        compensation = (angle / 90.00 * 1.8);
    } 
    val_7.data = current_joint_values[6] + compensation;
    pub_7.publish(val_7);
    mutex_rotate(val_7.data);
    return compensation;
}

void closeGrabber(ros::Publisher &pub_8, ros::Publisher &pub_9){
    std_msgs::Float64 val_8 ;
    val_8.data = gripper_close_value;
    pub_8.publish(val_8);

    std_msgs::Float64 val_9 ;
    val_9.data = gripper_close_value;
    pub_9.publish(val_9);
    
    //double sleep_time = std::abs(angle - current_joint_values.back()) / 0.85 * 4.0;
    sleep(5);
    
    /*
    while(1){
        if(load_8 > 0.4){
            val_8.data = current_joint_values.back() - 0.1;
            pub_8.publish(val_8);
            break;
        }
    }
    */
    /*
    while(1){
        if((std::abs(load_8) > 0.05)){ //previously 0.4
            val_8.data = current_joint_values[7] + paw_torque_compensate;
            val_9.data = -val_8.data;
            pub_8.publish(val_8);
            pub_9.publish(val_9);
            break;
        }
    }
    */
}

// NEW METHOD (CURRENTLY WORK IN PROGRESS)
/*
void correct_pose(std::vector<ros::Publisher> &pub_1to6, double coeff){
    double cur_joint_value = 0.0;
    double new_joint_value = 0.0;
    double adjustment = 0.0;
    std::cout<<"Correcting Joints..."<<std::endl;
    desired_correction.clear();
    // first lock in the desired pose (aka the pose that the encoders think we are in)
    for(int i = 0; i < 6 ; i++) { 
	cur_joint_value = current_joint_values[i];
	adjustment = (cur_joint_value - estimated_joint_values[i]) * coeff;
	new_joint_value = cur_joint_value + adjustment;
        desired_correction.push_back(new_joint_value);
        std::cout<<"Joint "<< i+1 << ": " << cur_joint_value << " + " << adjustment << " = " << new_joint_value << std::endl;
    }
    int num_correct = 0;
    double threshold = 0.01;
    while(num_correct < 6){
        num_correct = 0;
    	for(int i = 0; i < 6 ; i++){
      	    pub_1to6[i].publish(new_joint_value);
    	}
        sleep(1); //for now, sleep for a second to allow correction to complete
        for(int i = 0; i < 6 ; i++){
      	    if((desired_correction[i] - estimated_joint_values[i]) <= threshold){ num_correct++;}
    	}
        std::cout<<"Joints in place:" <<num_correct << std::endl;
    }
    std::cout<<"Correction Complete!" << std::endl;
    
}*/

// CORRECTION BASED ON VISUAL POSE ESTIMATION
void correct_pose(std::vector<ros::Publisher> &pub_1to6, double coeff){
    std::cout<<"Correcting Joints..."<<std::endl;
	
    double cur_joint_value = 0.0;
    double new_joint_value = 0.0;
    double adjustment = 0.0;
    for(int i = 0; i < 6 ; i++){
        cur_joint_value = current_joint_values[i];
        adjustment = (cur_joint_value - estimated_joint_values[i]) * coeff;
        new_joint_value = cur_joint_value + adjustment;
        std::cout<<"Joint "<< i+1 << ": " << cur_joint_value << " + " << adjustment << " = " << new_joint_value << std::endl;
        pub_1to6[i].publish(new_joint_value);
    }
    sleep(3); //for now, sleep for a second to allow correction to complete
}


void execute_with_correction(moveit::planning_interface::MoveGroup::Plan plan, std::vector<ros::Publisher> &pub_1to6, ros::Publisher &pub_7, double correct_coeff){
    // DEBUG: print trajectory 
    // std::cout<<plan.trajectory_.joint_trajectory.points[0]<<std::endl;
    clock_t start_clock = clock();
    double time_passed = 0.0;
    double next_stage = 0.0;
    double target_pos = 0.0;
    std::vector<double> adjustments{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //std::vector<double> adjustment_factor{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; //currently multiplicative, could be additive
    double cur_joint_value = 0.0;
    for(int p = 1; p < 10; p++){
        next_stage = plan.trajectory_.joint_trajectory.points[p].time_from_start.toSec();
        std::cout<<"GOAL UNTIL "<<next_stage<<std::endl;
        for(int j = 0; j < 7; j++){
            target_pos = plan.trajectory_.joint_trajectory.points[p].positions[j];
            std::cout<<target_pos;
            if(correct_coeff > 0.0){
                cur_joint_value = current_joint_values[j];
                adjustments[j] = (cur_joint_value - estimated_joint_values[j]) * correct_coeff;
                target_pos += adjustments[j] * adjustment_factor[j]; 
                std::cout<<" + "<<adjustments[j]<<" * "<<adjustment_factor[j]<<" = "<<target_pos;
            }
            std::cout<<std::endl;
            if(j < 6){
                pub_1to6[j].publish(target_pos);
            } else {
                pub_7.publish(target_pos);
            }
        }
        while(time_passed < next_stage){
            time_passed = ((double)(clock() - start_clock))/CLOCKS_PER_SEC;
        }
        //calculate adjustment factor if correction desired (changes based on how close the joint ended up to the target)
        //if (correct_coeff > 0.0){
        //    double diff = 0.0;
        //    for(int j = 0; j < 7; j++){
        //        if(adjustments[j] > 0.0){
        //            target_pos = plan.trajectory_.joint_trajectory.points[p].positions[j];
        //            diff = target_pos - estimated_joint_values[j];
        //            adjustment_factor[j] = (adjustments[j] + diff)/adjustments[j]; // if more adjustment needed this should be > 1
        //            std::cout<<adjustment_factor[j]<<std::endl;
        //        }
        //    }
        //    
        //}
    }
    sleep(2); //sleep to allow trajectory to finish
    //correct at the end based on encoder error
    //double adjustment = 0.0;
    //std::cout<<"ENCODER CORRECTION"<<std::endl;
    //for(int j = 0; j < 7; j++){
    //    target_pos = plan.trajectory_.joint_trajectory.points[9].positions[j];
    //    std::cout<<target_pos;
    //    cur_joint_value = current_joint_values[j];
    //    adjustment = target_pos - cur_joint_value;
    //    target_pos += adjustment; 
    //    std::cout<<" + "<<adjustment<<" = "<<target_pos;
    //    std::cout<<std::endl;
    //    if(j < 6){
    //        pub_1to6[j].publish(target_pos);
    //    } else {
    //        pub_7.publish(target_pos);
    //    }
    //}
    sleep(2); //sleep to allow correction to finish
}

void pickup_place(moveit::planning_interface::MoveGroup &group, ros::Publisher &pub_7,ros::Publisher &pub_8, ros::Publisher &pub_9, bool time_count, bool do_correction, double correction_coeff, std::vector<ros::Publisher> &pub_1to6){

    /*****************************************************************
    *                       Specify target pose                      *
    *****************************************************************/
    ROS_INFO("Setup target_pose for pickup");
    geometry_msgs::Pose target_pose_pickup;

    // target pose constraints 
    double x, y, z;
    
    std::cout<<"input x (ex. 0.4):"; 
    std::cin>>x;
    std::cout<<"input y (ex. 0.2):";
    std::cin>>y;
    std::cout<<"input z (ex. 0.1 or 0.0):"; 
    std::cin>>z;

    clock_t begin_pick = clock();

    double APPROACH_Z = 0.12;
    double GRIPPER_BUFFER = 0.03;
    target_pose_pickup.position.x = x;//0.4;
    target_pose_pickup.position.y = y;//0.0;
    target_pose_pickup.position.z = z + APPROACH_Z + GRIPPER_BUFFER;//0.2;, input actual Z of object instead of approach
    std::cout<<M_PI<<std::endl;
    tf::Quaternion qat_pick = tf::createQuaternionFromRPY(0, -M_PI, 0);
    qat_pick.normalize();
    std::cout<<qat_pick<<std::endl;
    target_pose_pickup.orientation.x = qat_pick.x();//0.577;//0.49; // two-sided gribber
    target_pose_pickup.orientation.y = qat_pick.y();//0.577;//0.49; // two-sided gribber
    target_pose_pickup.orientation.z = qat_pick.z();//0.577;//0.49;
    target_pose_pickup.orientation.w = qat_pick.w(); 

    ROS_INFO("Go to target_pose for pickup");
    // set target pose
    group.setPoseTarget(target_pose_pickup);
    // plan
    moveit::planning_interface::MoveGroup::Plan my_plan;
    // bool success = group.plan(my_plan); MOVEIT_EDIT
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    //compensate_slark(my_plan); //compensate the slark for 2nd motor
    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    // execute
    std::cout<<my_plan.trajectory_.joint_trajectory<<std::endl;
    //if(success) group.execute(my_plan);
    //mutex_traj();
    if(success) execute_with_correction(my_plan, pub_1to6, pub_7, correction_coeff);
    // CORRECTION STUFF
    /****************************************************************
    *                          Correct Pose (Pre-Lower)             *
    ****************************************************************/
    //if(do_correction) correct_pose(pub_1to6, correction_coeff);

    /****************************************************************
    *                          Rotate wrist                         *
    ****************************************************************/
    double angle;
    std::cout<<"Input the desired rotation angle(Positive(Clockwise)): ";std::cin>>angle;
    angle = rotateGripper(pub_7, angle);
    std::cout<<angle<<std::endl;

    /*****************************************************************
    *                          Open grabber                          *
    *****************************************************************/
    ROS_INFO("open grabber");
    openGrabber(pub_8, pub_9);

    /****************************************************************
    *                Move down to pick up by jacobian               *
    *****************************************************************/
    robot_state::RobotStatePtr kinematic_state = group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName());
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
    const Eigen::Affine3d &temp = kinematic_state->getGlobalLinkTransform("link_eef");

    Eigen::Vector3d Trans;
    Eigen::Matrix3d Rot;
    Trans = temp.translation();
    std::cout<<"the current position: \n"<<Trans<<std::endl;
    target_pose_pickup.position.z = Trans(2);
    // Eigen::Quaterniond q(temp.rotation());
    tf::Quaternion qat_down = tf::createQuaternionFromRPY(0, -M_PI, -angle);
    qat_down.normalize();
    target_pose_pickup.orientation.x = qat_down.x();
    target_pose_pickup.orientation.y = qat_down.y();
    target_pose_pickup.orientation.z = qat_down.z();
    target_pose_pickup.orientation.w = qat_down.w();

    target_pose_pickup.position.z = z + GRIPPER_BUFFER;

    group.setPoseTarget(target_pose_pickup);
    // plan
    success = group.plan(my_plan);
    //compensate_slark(my_plan);
    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
    // execute
    //if(success) group.execute(my_plan);
    //mutex_traj();
    if(success) execute_with_correction(my_plan, pub_1to6, pub_7, correction_coeff);


    //Jacobian_hybrid(group, target_pose1, 0.03);
    

    // CORRECTION STUFF
    /****************************************************************
    *                          Correct Pose  (Pre-Grasp)            *
    ****************************************************************/
    //if(do_correction) correct_pose(pub_1to6, correction_coeff);
    //ARUCO TAG CORRECTION
    ROS_INFO("Specify current position detected by Aruco Tags");

    // target pose constraints 
    double x_ar, y_ar, z_ar;
    
    std::cout<<"input x:"; 
    std::cin>>x_ar;
    std::cout<<"input y:";
    std::cin>>y_ar;
    std::cout<<"input z:"; 
    std::cin>>z_ar;

    target_pose_pickup.position.x = x + (x - x_ar);//0.4;
    target_pose_pickup.position.y = y + (y - y_ar);//0.0;
    target_pose_pickup.position.z = z + (z - z_ar) + GRIPPER_BUFFER;//0.2;

    group.setPoseTarget(target_pose_pickup);
    // plan
    success = group.plan(my_plan);
    // compensate_slark(my_plan);
    // visualization
    ROS_INFO("Visualizing ArUco Correction %s",success?"":"FAILED");    
    // execute
    //if(success) group.execute(my_plan);
    //mutex_traj();
    if(success) execute_with_correction(my_plan, pub_1to6, pub_7, correction_coeff);

    /*****************************************************************
    *                          Close grabber                          *
    *****************************************************************/
    
    ROS_INFO("close grabber");
    closeGrabber(pub_8, pub_9);
    clock_t end_pick = clock();
    time_pick = time_pick + ((double)(end_pick - begin_pick))/CLOCKS_PER_SEC;


    /*****************************************************************
    *                  Lift gribber a little bit                     *
    *****************************************************************/
    clock_t begin_place = clock();

    kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
    const Eigen::Affine3d &temp1 = kinematic_state->getGlobalLinkTransform("link_eef");

    Eigen::Vector3d Trans1;
    Trans1 = temp.translation();
    std::cout<<"the current position: \n"<<Trans1<<std::endl;
    target_pose_pickup.position.z = Trans1(2);
    /*Eigen::Quaterniond q1(temp1.rotation());
    target_pose_pickup.orientation.x = 1.0;
    target_pose_pickup.orientation.y = 0.0;
    target_pose_pickup.orientation.z = 0.0;
    target_pose_pickup.orientation.w = 0.0;
    target_pose_pickup.position.z = 0.0;*/
    tf::Quaternion qat_up = tf::createQuaternionFromRPY(0, -M_PI, 0);
    qat_down.normalize();
    target_pose_pickup.orientation.x = qat_up.x();
    target_pose_pickup.orientation.y = qat_up.y();
    target_pose_pickup.orientation.z = qat_up.z();
    target_pose_pickup.orientation.w = qat_up.w();

    target_pose_pickup.position.z = 0.1;
    std::cout<<"Height: "<<target_pose_pickup.position.z<<std::endl;
    
    group.setPoseTarget(target_pose_pickup);
    // plan
    success = group.plan(my_plan);
    //compensate_slark(my_plan);
    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
    // sleep(1.5);
    // execute
    ROS_INFO("Lift the gripper");
    if(success) group.execute(my_plan);
    mutex_traj();
    
    //Jacobian_hybrid(group,target_pose1,0.03);

    /******************************************************************
    *                              Place                              *
    *******************************************************************/
    /*ROS_INFO("Setup target_pose for place");
    geometry_msgs::Pose target_pose_place;
    std::cout<<"input x (ex. 0.4):"; std::cin>>x;
    std::cout<<"input y (ex. 0.2):"; std::cin>>y;
    std::cout<<"input z (ex. 0.1 or 0.0):"; std::cin>>z;*/

    /*target_pose_place.position.x = x;//0.4;
    target_pose_place.position.y = y;//0.0;
    target_pose_place.position.z = z;//0.2;*/

    /*
    target_pose_place.orientation.x = 0.49;//0.577;//0.49; // two-sided gribber
    target_pose_place.orientation.y = 0.49;//0.577;//0.49; // two-sided gribber
    target_pose_place.orientation.z = 0.49;//0.577;//0.49;
    target_pose_place.orientation.w = sqrt(1 - target_pose_place.orientation.x * target_pose_place.orientation.x - 
        target_pose_place.orientation.y * target_pose_place.orientation.y - target_pose_place.orientation.z * target_pose_place.orientation.z);
    */
    /*tf::Quaternion qat_place = tf::createQuaternionFromRPY(0, -M_PI, 0);
    qat_place.normalize();
    std::cout<<qat_place<<std::endl;
    target_pose_place.orientation.x = qat_place.x();//0.577;//0.49; // two-sided gribber
    target_pose_place.orientation.y = qat_place.y();//0.577;//0.49; // two-sided gribber
    target_pose_place.orientation.z = qat_place.z();//0.577;//0.49;
    target_pose_place.orientation.w = qat_place.w(); */
    
    /*ROS_INFO("Go to target_pose for place");
    // set target pose
    group.setPoseTarget(target_pose_place);
    // plan
    success = group.plan(my_plan);
    //compensate_slark(my_plan);
    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
    // sleep(1.5);
    // execute
    if(success) group.execute(my_plan);
    mutex_traj();*/

    // CORRECTION STUFF
    /****************************************************************
    *                          Correct Pose (Pre-Lower)             *
    ****************************************************************/
    //if(do_correction) correct_pose(pub_1to6, correction_coeff);

    /****************************************************************
    *                Move down to place by jacobian               *
    *****************************************************************/
    /*robot_state::RobotStatePtr kinematic_state_place = group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group_place = group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName());
    const std::vector<std::string> &joint_names_place = joint_model_group_place->getJointModelNames();
    kinematic_state_place->setJointGroupPositions(joint_model_group_place, current_joint_values);
    const Eigen::Affine3d &temp_place = kinematic_state_place->getGlobalLinkTransform("link_eef");*/

    /*Eigen::Vector3d Trans_place;
    Eigen::Matrix3d Rot_place;
    Trans_place = temp_place.translation();
    std::cout<<"the current position: \n"<<Trans_place<<std::endl;
    target_pose_place.position.z = Trans_place(2);
    // Eigen::Quaterniond q(temp.rotation());
    tf::Quaternion qat_down_place = tf::createQuaternionFromRPY(0, -M_PI, 0);
    qat_down_place.normalize();
    target_pose_place.orientation.x = qat_down_place.x();
    target_pose_place.orientation.y = qat_down_place.y();
    target_pose_place.orientation.z = qat_down_place.z();
    target_pose_place.orientation.w = qat_down_place.w();*/
    
    //target_pose_place.position.z = OBJ_Z; //-0s.01;//-0.05;//target_pose1.position.z - 0.06;

    /*
    group.setPoseTarget(target_pose_place);
    // plan
    success = group.plan(my_plan);
    //compensate_slark(my_plan);
    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
    // execute
    if(success) group.execute(my_plan);
    mutex_traj();*/

    //END MOVE DOWN
    //open gripper
    openGrabber(pub_8, pub_9);

    //lift a little bit
    /*target_pose_place.position.z = target_pose_place.position.z + 0.05;
    // set target pose
    group.setPoseTarget(target_pose_place);
    // plan
    success = group.plan(my_plan);
    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
    // sleep(1.5);
    // execute
    ROS_INFO("Lift the gripper");
    if(success) group.execute(my_plan);
    mutex_traj();*/

    /*clock_t end_place = clock();
    time_place = time_place + ((double)(end_place - begin_place))/CLOCKS_PER_SEC;*/

    if(time_count)
    {
        std::cout<<"Time for picking object: "<<time_pick<<"s"<<std::endl;
        //std::cout<<"Time for placing object: "<<time_place<<"s"<<std::endl;
    }


}

const std::vector<std::string> listNamedPoses(moveit::planning_interface::MoveGroup &group){
    // list of all stored pose name in SRDF 
    const std::vector<std::string> namedTargets = group.getNamedTargets();
    std::cout<<"stored position in SRDF: "<<std::endl;
    for(int i = 0; i < namedTargets.size(); i++){
        std::cout<<i<<": ";
        std::cout<<namedTargets[i]<<std::endl;
    }

    return namedTargets;
}

void gotoNamedTarget(moveit::planning_interface::MoveGroup &group, std::string target, bool constraint_on){
    ROS_INFO("TASK: Go to %s pose", target.c_str());

    // get joint values of stored pose by name
    std::vector<double> group_variable_values;
    std::map<std::string, double> positions = group.getNamedTargetValues(target);
    for (std::map<std::string, double> ::iterator it=positions.begin(); it!=positions.end(); ++it){
      std::cout << it->first << " => " << it->second << '\n';
      group_variable_values.push_back(it->second);
    }
 
    // get constraints
    if(constraint_on) getConstraint(group, target);

    // plan
    moveit::planning_interface::MoveGroup::Plan my_plan;
    group.setJointValueTarget(group_variable_values);
    // bool success = group.plan(my_plan); MOVEIT_EDIT
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    //compensate_slark(my_plan);
    /*
    sleep(5);

    ros::NodeHandle node_handle;
    ros::Publisher pub_8 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_8/command", 1, true);
    closeGrabber(pub_8,gripper_close_value_paw);
    sleep(5);
    */

    // set planning time to default
    group.setPlanningTime(20.0);
    group.clearPathConstraints();

    // visualization
    ROS_INFO("Visualizing plan home (joint space goal) %s",success?"":"FAILED");
    //sleep(5.0);    
 
    // execution
    ROS_INFO("Execution if plan successfully");
    if(success) group.execute(my_plan);
    mutex_traj();
    
    // show current Joints (the real robot joints, not the planned joints)
    std::vector<double> currentJoints = group.getCurrentJointValues();
    /*
    std::cout<< "current joint values:"<<std::endl;
    for(size_t i = 0; i<currentJoints.size(); i++) std::cout<< currentJoints[i]<<" ";
    std::cout<<std::endl;
    */

}

void addObject2Scene(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher){
    /* Define the object message */
    moveit_msgs::CollisionObject object;

    /* The header must contain a valid TF frame */
    object.header.frame_id = group.getPlanningFrame();
    /* The id of the object */
    object.id = "box";

    /* A default pose */
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x =  0.5;
    pose.position.y =  0.5;
    pose.position.z = -0.165;//-0.13;//-0.1;//-0.07

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.9;
    primitive.dimensions[1] = 0.9;
    primitive.dimensions[2] = 0.05;

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);


    /* An attach operation requires an ADD */
    object.operation = object.ADD;

    /* Publish and sleep (to view the visualized results) */
    collision_object_publisher.publish(object);

    std::vector<moveit_msgs::CollisionObject> collision_objects;  
    collision_objects.push_back(object);  

    // Now, let's add the collision object into the world
    ROS_INFO("Add an object into the world");  
    planning_scene_interface.addCollisionObjects(collision_objects);

    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();

}

void getConstraint(moveit::planning_interface::MoveGroup &group, std::string target){
    ROS_INFO("get constraints");

    // switch wont take string
    int targetNum = 0;
    if(target == "home") targetNum = 0;
    else if(target == "holding") targetNum = 1;
    else targetNum = -1;

    // avoid cross init
    moveit_msgs::OrientationConstraint ocm; 
    moveit_msgs::Constraints test_constraints;
    switch(targetNum){
        case 0: // no constraints
          break;

        case 1: // hoding position  
          ocm.link_name = "link_8";  
          ocm.header.frame_id = "base_link";
          ocm.orientation.z = 1.0;
          ocm.absolute_x_axis_tolerance = 0.1; //0.1
          ocm.absolute_y_axis_tolerance = 0.1; //0.1
          ocm.absolute_z_axis_tolerance = 0.5; //0.1
          ocm.weight = 1.0;

          test_constraints.orientation_constraints.push_back(ocm);  
          group.setPathConstraints(test_constraints);
          group.setPlanningTime(20.0);
          break;

        default: // no constraints
          break;
    }
}

void adjust(geometry_msgs::Pose &target_pose){

    // get target and grabber from vision
    std::string target_x, target_y, grabber_x, grabber_y;
    double target_x_val, target_y_val, grabber_x_val, grabber_y_val;
    std::ifstream myfileIn ("/home/fujenchu/Projects/ARM_project/shareFromWin/detection_results.txt");
    if(true){ // got sth
        getline (myfileIn,target_x);
        getline (myfileIn,target_y);
        getline (myfileIn,grabber_x);
        getline (myfileIn,grabber_y);
        
        std::ofstream myfileOut ("/home/fujenchu/Projects/ARM_project/shareFromWin/detection_results.txt", std::ios::trunc);
        myfileOut << "";
        myfileOut.close();
     
        std::cout<<"x = "<<target_x<<std::endl;
        std::cout<<"y = "<<target_y<<std::endl;
        std::cout<<"x = "<<grabber_x<<std::endl;
        std::cout<<"y = "<<grabber_y<<std::endl;
    }
    myfileIn.close();

    target_x_val = std::atof(target_x.c_str());
    target_y_val = std::atof(target_y.c_str());
    grabber_x_val = std::atof(grabber_x.c_str());
    grabber_y_val = std::atof(grabber_y.c_str());

    // adject based on current difference
    target_pose.position.x = target_pose.position.x + (target_x_val - grabber_x_val);
    target_pose.position.y = target_pose.position.y + (target_y_val - grabber_y_val);

}
