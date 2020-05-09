//
// Created by zeid on 2/27/20.
//
#include "robot_controller.h"

/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */
RobotController::RobotController(std::string arm_id) :
robot_controller_nh_("/ariac/"+arm_id),
robot_controller_options("manipulator",
        "/ariac/"+arm_id+"/robot_description",
        robot_controller_nh_),
robot_move_group_(robot_controller_options) {
    ROS_WARN(">>>>> RobotController");

    robot_move_group_.setPlanningTime(15);
    robot_move_group_.setNumPlanningAttempts(12);
    robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
    robot_move_group_.setMaxVelocityScalingFactor(0.8);
    robot_move_group_.setMaxAccelerationScalingFactor(0.8);
    // robot_move_group_.setEndEffector("moveit_ee");
    robot_move_group_.allowReplanning(true);


    //--These are joint positions used for the home position

    // home_joint_pose_ = {-0.15, 3.14, -2.43, -1.63, -0.63, 1.51, 0.88};  //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
    initial_home_joint_pose_ = {-0.15, 3.14, -2.43, -1.63, -0.63, 1.51, 0.88};  
    home_joint_pose_ = {0, 3.14, -2, 2.6, 4.1, 4.7, 0};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}   //bin position

    if(arm_id=="arm1") agv_drop_position_ = {1.00, 1.13, -1.34, 2.0, 4.12, 4.67, -4.27};  
    else agv_drop_position_ = {-1.08, 4.65, -1.51, 1.76, 4.4, -1.51, 0};
    //-- offset used for picking up parts
    //-- For the pulley_part, the offset is different since the pulley is thicker
    offset_ = 0.0231;

    //--topic used to get the status of the gripper
    gripper_subscriber_ = gripper_nh_.subscribe(
            "/ariac/"+arm_id+"/gripper/state", 10, &RobotController::GripperCallback, this);

    robot_joint_state_sub = robot_controller_nh_.subscribe(
            "/ariac/"+arm_id+"/joint_states", 10, &RobotController::JointStatesCallback, this);


    ROS_INFO_STREAM("Sending robot "<< arm_id << " home");
    if(arm_id=="arm1") {
        SendRobotHome1();
    }

    if(arm_id=="arm2") {
        SendRobotHome1();
    }
    //SendRobotHome();

    robot_tf_listener_.waitForTransform(arm_id+"_linear_arm_actuator", arm_id+"_ee_link",
                                            ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/"+arm_id+"_linear_arm_actuator", "/"+arm_id+"_ee_link",
                                           ros::Time(0), robot_tf_transform_);


    fixed_orientation_.x = robot_tf_transform_.getRotation().x();
    fixed_orientation_.y = robot_tf_transform_.getRotation().y();
    fixed_orientation_.z = robot_tf_transform_.getRotation().z();
    fixed_orientation_.w = robot_tf_transform_.getRotation().w();

    tf::quaternionMsgToTF(fixed_orientation_,q);
    tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);


    end_position_ = home_joint_pose_;
    end_position_ = {-1.1, 3.14, -2, 2.6, 3.9, 4.7, 0};   //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
    // end_position_[0] 0.2;      
    // end_position_[1] = 4.5;
    // end_position_[2] = 1.2;


    robot_tf_listener_.waitForTransform("world", arm_id+"_ee_link", ros::Time(0),
                                            ros::Duration(10));
    robot_tf_listener_.lookupTransform("/world", "/"+arm_id+"_ee_link", ros::Time(0),
                                           robot_tf_transform_);

    home_cart_pose_.position.x = robot_tf_transform_.getOrigin().x();
    home_cart_pose_.position.y = robot_tf_transform_.getOrigin().y();
    home_cart_pose_.position.z = robot_tf_transform_.getOrigin().z();
    home_cart_pose_.orientation.x = robot_tf_transform_.getRotation().x();
    home_cart_pose_.orientation.y = robot_tf_transform_.getRotation().y();
    home_cart_pose_.orientation.z = robot_tf_transform_.getRotation().z();
    home_cart_pose_.orientation.w = robot_tf_transform_.getRotation().w();

    agv_tf_listener_.waitForTransform("world", "kit_tray_1",
                                      ros::Time(0), ros::Duration(10));
    agv_tf_listener_.lookupTransform("/world", "/kit_tray_1",
                                     ros::Time(0), agv_tf_transform_);
    agv_position_.position.x = agv_tf_transform_.getOrigin().x();
    agv_position_.position.y = agv_tf_transform_.getOrigin().y();
    agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;

    gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
            "/ariac/"+arm_id+"/gripper/control");
    counter_ = 0;
    drop_flag_ = false;
}

RobotController::~RobotController() {}

/**
 *
 * @return
 */
bool RobotController::Planner() {
    ROS_INFO_STREAM("Planning started...");
    if (robot_move_group_.plan(robot_planner_) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        plan_success_ = true;
        ROS_INFO_STREAM("Planner succeeded!");
    } else {
        plan_success_ = false;
        ROS_WARN_STREAM("Planner failed!");
    }

    return plan_success_;
}


void RobotController::Execute() {
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.0).sleep();
    }
}

void RobotController::GoToTarget(const geometry_msgs::Pose& pose) {
    ROS_INFO_STREAM("Going to target!");
    target_pose_.orientation = fixed_orientation_;
    target_pose_.position = pose.position;
    ros::AsyncSpinner spinner(4);
    robot_move_group_.setPoseTarget(target_pose_);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.1).sleep();
    }
    ROS_INFO_STREAM("Point reached...");
}

void RobotController::GoToTarget(const geometry_msgs::Pose& pose, float delay) {
    ROS_INFO_STREAM("Delayed start...");
    //ros::Duration(2.48).sleep();
    ROS_INFO_STREAM("Going to target!");
    target_pose_.orientation = fixed_orientation_;
    target_pose_.position = pose.position;
    ros::AsyncSpinner spinner(4);
    robot_move_group_.setPoseTarget(target_pose_);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO_STREAM("Point reached...");
}

void RobotController::GoToTarget(
        std::initializer_list<geometry_msgs::Pose> list) {
    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::vector<geometry_msgs::Pose> waypoints;
    for (auto i : list) {
        i.orientation.x = fixed_orientation_.x;
        i.orientation.y = fixed_orientation_.y;
        i.orientation.z = fixed_orientation_.z;
        i.orientation.w = fixed_orientation_.w;
        waypoints.emplace_back(i);
    }

    moveit_msgs::RobotTrajectory traj;
    auto fraction =
            robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

    ROS_WARN_STREAM("Fraction: " << fraction * 100);
    ros::Duration(1.0).sleep();

    robot_planner_.trajectory_ = traj;

    //if (fraction >= 0.3) {
    // ROS_INFO_STREAM("Delay");
    robot_move_group_.execute(robot_planner_);
    ros::Duration(1.0).sleep();
//    } else {
//        ROS_ERROR_STREAM("Safe Trajectory not found!");
//    }
}

void RobotController::SendRobotHome() {
    // ros::Duration(2.0).sleep();
    robot_move_group_.setJointValueTarget(home_joint_pose_);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.0).sleep();
    }

     ros::Duration(1.0).sleep();
     
}
void RobotController::SendRobotHome1() {
    // ros::Duration(2.0).sleep();
    robot_move_group_.setJointValueTarget(initial_home_joint_pose_);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.0).sleep();
    }

     ros::Duration(2.0).sleep();
     
}

void RobotController::SendRobotEnd() {
    robot_move_group_.setJointValueTarget(end_position_);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(3.0).sleep();
    }

     ros::Duration(2.0).sleep();

}


void RobotController::SendRobotToJointValues(std::vector<double> joint_values) {
    robot_move_group_.setJointValueTarget(joint_values);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        ROS_INFO_STREAM("Going to joint value...");
        robot_move_group_.move();
        ros::Duration(2.0).sleep();
    }  
}

void RobotController::GripperToggle(const bool& state) {
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    ros::Duration(1.0).sleep();
    // if (gripper_client_.call(gripper_service_)) {
    if (gripper_service_.response.success) {
        ROS_INFO_STREAM("Gripper activated!");
    } else {
        ROS_WARN_STREAM("Gripper activation failed!");
    }
}


bool RobotController::DropPartExchange(geometry_msgs::Pose part_pose) {
    ROS_INFO_STREAM("Dropping part at the exchange point");

    if(gripper_state_) {
        ros::Duration(1.5).sleep();
        ros::spinOnce();
        this->GoToTarget(part_pose);
        ros::Duration(1.0).sleep();
        this->GripperToggle(false);
        this->SendRobotHome();
    }

    return gripper_state_;
}

bool RobotController::DropPart(geometry_msgs::Pose part_pose) {
    

    drop_flag_ = true;
    float delay=1.8;
    ros::spinOnce();
    ROS_INFO_STREAM("Placing phase activated...");

    if (gripper_state_){//--while the part is still attached to the gripper
        //--move the robot to the end of the rail
        ROS_INFO_STREAM("Moving towards AGV...");
        
        //ros::Duration(1.0).sleep();
        this->SendRobotToJointValues(agv_drop_position_);
        ros::Duration(1.2).sleep();

        this->GoToTarget(part_pose);
        ros::Duration(1.0).sleep();
        if(!gripper_state_) {
            ROS_WARN_STREAM("Part dropped...picking up from the dropped position and picking...");
            return !gripper_state_;   // true
        }
        // this->GoToTarget(part_pose);
        ros::Duration(1.0).sleep();
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(false);
        ros::spinOnce();
        auto temp_pose = part_pose;

       if (!gripper_state_) {
            // temp_pose.position.z+=0.6;
            // this->GoToTarget(temp_pose);
            // ros::Duration(0.5).sleep();
            ROS_INFO_STREAM("Going to home position...");
            this->SendRobotHome();
            ros::Duration(0.1).sleep();
       }
    }

    drop_flag_ = false;
    return gripper_state_;
}

bool RobotController::DropPartFlipped(geometry_msgs::Pose part_pose) {

    auto temp_pose_1 = part_pose;
    temp_pose_1.position.z += 0.3;
    ROS_INFO_STREAM("Actuating the gripper!");
    this->GripperToggle(true);
    ROS_INFO_STREAM("Going to waypoint...");
    
    this->GoToTarget(part_pose);
    ros::Duration(0.1).sleep();
    this->GoToTarget(temp_pose_1);
    // if(product_type=="piston_rod_part") this->GoToTarget(part_pose, delay);    // For conveyor belt pick-ups
    // else this->GoToTarget(part_pose);
    ros::spinOnce();
    while (!gripper_state_) {
        part_pose.position.z -= 0.007;
        this->GoToTarget(part_pose);
        ros::Duration(0.3).sleep();
        this->GoToTarget(temp_pose_1);
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(true);
        ros::spinOnce();
    }

    drop_flag_ = true;
    float delay=1.8;
    ros::spinOnce();
    ROS_INFO_STREAM("Flipping part...");

    if (gripper_state_){//--while the part is still attached to the gripper
        
        // ROS_INFO_STREAM("Moving towards bin to place it vertically...");
        
        target_pose_.position = part_pose.position;
        target_pose_.orientation = part_pose.orientation;
        ros::AsyncSpinner spinner(4);
        // target_pose_.position.y -= 0.1;
        target_pose_.position.x += 0.04197;
        target_pose_.position.z += 0.035;
        robot_move_group_.setPoseTarget(target_pose_);
        spinner.start();
        if (this->Planner()) {
            robot_move_group_.move();
            ros::Duration(1.1).sleep();
        }
        ROS_INFO_STREAM("Placed vertically reached...");
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(false);
        ros::spinOnce();
    }
    //After made vertically stand moving the arm behind the part with below distance
    target_pose_.position.x -= 0.35;
    target_pose_.position.y -= 0.009;
    target_pose_.position.z += 0.3;
    this->GoToTarget(target_pose_);
    //Moving infront of the part
    target_pose_.position.x += 0.525;
    this->GoToTarget(target_pose_);
    //Pushing the part from front
    //target_pose_.position.y += 0.012;
    target_pose_.position.z -= 0.3;
    this->GoToTarget(target_pose_);
    //Reaching to pick the flipped part 
    target_pose_.position.x -= 0.28;
    target_pose_.position.y -= 0.03;
    target_pose_.position.z += 0.07;
    this->GoToTarget(target_pose_);         
    std::string product_type = "pulley_part";
    

    drop_flag_ = false;
    return gripper_state_;
}



bool RobotController::DiscardPart(geometry_msgs::Pose part_pose) {
    
    ROS_INFO_STREAM("Discarding part...");
    bool discard_flag = false;
    ROS_INFO_STREAM("Moving towards AGV...");
    this->SendRobotToJointValues(agv_drop_position_);
    ROS_INFO_STREAM("Actuating the gripper...");
    this->GripperToggle(true);
    auto temp_pose = part_pose;
    temp_pose.position.z += 0.15;
    part_pose.position.z += 0.02;
    this->GoToTarget({part_pose, temp_pose});
    ros::Duration(0.5).sleep();
    // this->GoToTarget(temp_pose);
    ros::spinOnce();
    while (!gripper_state_) {
        part_pose.position.z -= 0.007;
        this->GoToTarget({part_pose, temp_pose});
        ros::Duration(0.3).sleep();
        // this->GoToTarget(temp_pose);
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(true);
        ros::spinOnce();
    }
    auto drop_pose = part_pose;
    drop_pose.position.x -= 0.52;
    if((part_pose.position.y)<-1.5){drop_pose.position.y += 0.5;}
    else if((part_pose.position.y)>1.5){drop_pose.position.y -= 0.5;}
    drop_pose.position.z += 0.3;
    this->GoToTarget(drop_pose);
    ros::Duration(2.0).sleep();
    this->GripperToggle(false);
    ros::spinOnce();
    discard_flag = true;
    // this->SendRobotToJointValues(agv_drop_position_);
    ROS_WARN_STREAM("Discarded Part!");
    this->SendRobotHome();
    return discard_flag;
}

bool RobotController::PickAndPlaceUpdated(geometry_msgs::Pose pick_pose, geometry_msgs::Pose place_pose) {
    
    ROS_INFO_STREAM("Updating part pose...");
    bool update_flag = false;

    ROS_INFO_STREAM("Moving towards AGV...");
    this->SendRobotToJointValues(agv_drop_position_);
    auto temp_pose = pick_pose;
    temp_pose.position.z += 0.15;
    ROS_INFO_STREAM("Actuating the gripper...");
    this->GripperToggle(true);
    pick_pose.position.z += 0.02;
    this->GoToTarget({pick_pose, temp_pose});
    ros::Duration(1.0).sleep();
    while (!gripper_state_) {
        pick_pose.position.z -= 0.007;
        this->GoToTarget({pick_pose, temp_pose});
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(true);
        ros::spinOnce();
    }
    ros::spinOnce();
    auto drop_pose = place_pose;
    drop_pose.position.z += 0.1;
    this->GoToTarget(drop_pose);
    ros::Duration(2.0).sleep();
    this->GripperToggle(false);
    ros::spinOnce();
    update_flag = true;
    this->SendRobotToJointValues(agv_drop_position_);


    ROS_WARN_STREAM("Updated Part Pose!");
    return update_flag;
}


void RobotController::GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}

void RobotController::JointStatesCallback(
    const sensor_msgs::JointState::ConstPtr& state) {
    arm_joint_state_ = state->position;
}

std::vector<double> RobotController::getJointStates() {
    return arm_joint_state_;
}


bool RobotController::PickPart(geometry_msgs::Pose& part_pose, std::string product_type,int OnConveyor) {
    float delay=1.8;
    ROS_INFO_STREAM("Moving to part...");

    if (OnConveyor==1)
    {
        if(product_type=="gear_part")
            {
                part_pose.position.z -= 0.0105; 
                part_pose.position.y -= 0.8; 
            }
        else if (product_type=="piston_rod_part")
            {
                part_pose.position.z -= 0.01575;
                part_pose.position.y -= 0.8; 
            }
        else if (product_type=="disk_part")
            {
                //part_pose.position.z += 0.04;
                part_pose.position.y -= 0.8;            // Generalize
            }
        else if (product_type=="gasket_part")
            {
                //part_pose.position.z -= 0.001;
                part_pose.position.y -= 0.8;            // Generalize
            }
    }
    else
    {
        if(product_type=="gear_part")
            {
                part_pose.position.z += 0.028;
            }
        else if (product_type=="piston_rod_part")
            {
                part_pose.position.z += 0.0178;
            }
        else if (product_type=="disk_part")
            {
                part_pose.position.z += 0.04;
                //part_pose.position.y -= 0.8;            // Generalize
            }
        // else if (product_type=="gasket_part")
        //     {
        //         part_pose.position.z += 0.08;
        //         //part_pose.position.y -= 0.8;            // Generalize
        //     }
    }
    
    
    auto temp_pose_1 = part_pose;
    temp_pose_1.position.z += 0.15;
    ROS_INFO_STREAM("Actuating the gripper!");
    this->GripperToggle(true);

    ros::Duration(0.1).sleep();
    ROS_INFO_STREAM("Going to waypoint...");
    ROS_WARN_STREAM("Part_Pose Final"<<part_pose);
    this->GoToTarget(part_pose);
    ros::Duration(0.1).sleep();
    this->GoToTarget(temp_pose_1);
    // if(product_type=="piston_rod_part") this->GoToTarget(part_pose, delay);    // For conveyor belt pick-ups
    // else this->GoToTarget(part_pose);
    ros::spinOnce();
    while (!gripper_state_) {
        //part_pose.position.z -= 0.007;
        //ros::Duration(0.1).sleep();
        this->GoToTarget(part_pose);
        ros::Duration(0.1).sleep();
        this->GoToTarget(temp_pose_1);
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(true);
        //ros::spinOnce();
    }
    ros::Duration(1.0).sleep();
    ROS_INFO_STREAM("Moving to home position with part!");
    if(OnConveyor!=1){
    this->SendRobotHome();}
    else{
        if(product_type=="piston_rod_part") {
        part_pose.position.z += 0.0157;}
        else if(product_type=="gear_part") {
        part_pose.position.z += 0.007;
    }}

    // if(product_type=="piston_rod_part") this->SendRobotToJointValues(home_joint_pose_);
    // else this->SendRobotToJointValues(bin_position_);

    // this->GoToTarget(temp_pose_1);
    ros::Duration(1.5).sleep();



    return gripper_state_;
}