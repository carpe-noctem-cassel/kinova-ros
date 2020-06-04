#include <signal.h>
#include <pick_place.h>
#include <ros/console.h>
#include <string>
#include "object_factory.h"
#include <iostream>

#include <tf_conversions/tf_eigen.h>

const double FINGER_MAX = 6400;

using namespace kinova;

void ctlc_handler(int sig){
    exit(1);
}

// TODO: Existiert eine umwandlung von Zxz nach Quaternion?
tf::Quaternion EulerZYZ_to_Quaternion(double tz1, double ty, double tz2) {
    tf::Quaternion q;
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();

    rot_temp.setEulerYPR(tz1, 0.0, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(0.0, ty, 0.0); // Dritter WErt für zxz
    rot *= rot_temp;
    rot_temp.setEulerYPR(tz2, 0.0, 0.0);
    rot *= rot_temp;
    rot.getRotation(q);
    return q;
}

PickPlace::PickPlace(ros::NodeHandle &nh) : nh_(nh) {
    std::cout << "PP Constructor called\n";

    ros::NodeHandle pn("~");

    nh_.param<std::string>("/robot_type", robot_type_, "j2n6s300");
    nh_.param<bool>("/robot_connected", robot_connected_, true);

    if (robot_connected_) {
        // sub_joint_ =
        // nh_.subscribe<sensor_msgs::JointState>("/j2s7s300_driver/out/joint_state",
        // 1, &PickPlace::get_current_state, this);
        sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/" + robot_type_ + "_driver/out/tool_pose", 1,
                                                              &PickPlace::get_current_pose, this);
    }

    // Before we can load the planner, we need two objects, a RobotModel and a
    // PlanningScene.
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();

    // construct a `PlanningScene` that maintains the state of the world
    // (including the robot).
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    //    //  every time need retrive current robot state, do the following.
    //    robot_state::RobotState& robot_state =
    //    planning_scene_->getCurrentStateNonConst(); const
    //    robot_state::JointModelGroup *joint_model_group =
    //    robot_state.getJointModelGroup("arm");

    group_ = new moveit::planning_interface::MoveGroupInterface("arm");
    gripper_group_ = new moveit::planning_interface::MoveGroupInterface("gripper");

    group_->setEndEffectorLink(robot_type_ + "_end_effector");
    finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>(
            "/" + robot_type_ + "_driver/fingers_action/finger_positions", false);
    while (robot_connected_ && !finger_client_->waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the finger action server to come up");
    }

    pub_co_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
    pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    int arm_joint_num = robot_type_[3] - '0';
    joint_names_.resize(arm_joint_num);
    joint_values_.resize(joint_names_.size());
    for (uint i = 0; i < joint_names_.size(); i++) {
        joint_names_[i] = robot_type_ + "_joint_" + boost::lexical_cast<std::string>(i + 1);
    }

    // set pre-defined joint and pose values.
    define_cartesian_pose();
    define_joint_values();

    // pick process
    result_ = false;
    my_pick();
}

PickPlace::~PickPlace() {
    // shut down pub and subs
    pub_co_.shutdown();
    pub_aco_.shutdown();
    pub_planning_scene_diff_.shutdown();

    // release memory
    delete group_;
    delete gripper_group_;
    delete finger_client_;
}

void PickPlace::get_current_state(const sensor_msgs::JointStateConstPtr &msg) {
    boost::mutex::scoped_lock lock(mutex_state_);
    current_state_ = *msg;
}

void PickPlace::get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg) {
    boost::mutex::scoped_lock lock(mutex_pose_);
    current_pose_ = *msg;
}

void replaceTable(moveit_msgs::CollisionObject &collisionObject, ros::Publisher &pub, moveit_msgs::PlanningScene &planningScene, std::string id = "table", double x = 0, double y = 0.9, double edgeLength = 1, double height = 0.55, double topThickness = 0.1, double legRadius = 0.05, double legInset = 0.1, double shieldThickness = 0.01){
    replaceObject(collisionObject, pub, planningScene, id + "Top", shape_msgs::SolidPrimitive::BOX, Position(x, y, height - topThickness / 2), {edgeLength, edgeLength, topThickness});
    replaceObject(collisionObject, pub, planningScene, id + "Leg1", shape_msgs::SolidPrimitive::CYLINDER, Position(x + (edgeLength/2 - legInset), y + (edgeLength/2 - legInset), (height - topThickness) / 2), {height - topThickness, legRadius});
    replaceObject(collisionObject, pub, planningScene, id + "Leg2", shape_msgs::SolidPrimitive::CYLINDER, Position(x + (edgeLength/2 - legInset), y - (edgeLength/2 - legInset), (height - topThickness) / 2), {height - topThickness, legRadius});
    replaceObject(collisionObject, pub, planningScene, id + "Leg3", shape_msgs::SolidPrimitive::CYLINDER, Position(x - (edgeLength/2 - legInset), y + (edgeLength/2 - legInset), (height - topThickness) / 2), {height - topThickness, legRadius});
    replaceObject(collisionObject, pub, planningScene, id + "Leg4", shape_msgs::SolidPrimitive::CYLINDER, Position(x - (edgeLength/2 - legInset), y - (edgeLength/2 - legInset), (height - topThickness) / 2), {height - topThickness, legRadius});
    replaceObject(collisionObject, pub, planningScene, id + "Shield", shape_msgs::SolidPrimitive::BOX, Position(x, y - edgeLength/2 + shieldThickness/2, (height - topThickness) / 2), {edgeLength - 2*legInset - 4*legRadius - 0.01, shieldThickness, (height - topThickness - 0.01)});
}

/**
 * @brief PickPlace::gripper_action
 * @param gripper_rad close for 6400 and open for 0.0
 * @return true is gripper motion reaches the goal
 */
bool PickPlace::gripper_action(double finger_turn) {
    if (robot_connected_ == false) {
        if (finger_turn > 0.5 * FINGER_MAX) {
            gripper_group_->setNamedTarget("Close");
        } else {
            gripper_group_->setNamedTarget("Open");
        }
        gripper_group_->move();
        return true;
    }

    if (finger_turn < 0) {
        finger_turn = 0.0;
    } else {
        finger_turn = std::min(finger_turn, FINGER_MAX);
    }

    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = finger_turn;
    goal.fingers.finger2 = goal.fingers.finger1;
    goal.fingers.finger3 = goal.fingers.finger1;
    finger_client_->sendGoal(goal);

    if (finger_client_->waitForResult(ros::Duration(5.0))) {
        finger_client_->getResult();
        return true;
    } else {
        finger_client_->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
}

void PickPlace::clear_workscene() {
    // remove table
    co_.id = "table";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    // remove target
    co_.id = "target_cylinder";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    // remove attached target
    aco_.object.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_aco_.publish(aco_);

    planning_scene_msg_.world.collision_objects.clear();
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);

    clear_obstacle();
}

void PickPlace::build_workscene() {
    std::cout << "Constructing workscene...\n";
    co_.header.frame_id = "world";
    co_.header.stamp = ros::Time::now();

    replaceObject(co_, pub_co_, planning_scene_msg_, "floor", shape_msgs::SolidPrimitive::BOX, Position(0, 0, -0.03 / 2.0), {2.4, 2.4, 0.03});

    replaceTable(co_, pub_co_, planning_scene_msg_);

    replaceObject(co_, pub_co_, planning_scene_msg_, "laser", shape_msgs::SolidPrimitive::CYLINDER, Position(-0.05, 0, 0.3), {0.2, 0.1});

    std::cout << "Cup placement(x y): ";
    std::cin >> cup_x >> cup_y;

    replaceObject(co_, pub_co_, planning_scene_msg_, "cup", shape_msgs::SolidPrimitive::CYLINDER, Position(cup_x, 0.65 + cup_y, 0.6), {0.1, 0.01});


    ros::WallDuration(0.1).sleep();
    std::cout << "Setup COMPLETE!\n";
}

void PickPlace::clear_obstacle() {
    co_.id = "pole";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    co_.id = "bot_obstacle";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    co_.id = "top_obstacle";
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ":
    //      remove pole "); std::cin >> pause_;
}

// TODO: best way to attach?
void PickPlace::add_attached_obstacle() {
     //once the object is know to be grasped
     //we remove obstacle from work scene
    co_.id = "cup";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    // and then we declare it as an attached obstacle
    aco_.object.id = "cup";
    aco_.object.operation = moveit_msgs::CollisionObject::ADD;
    aco_.link_name = robot_type_ + "_end_effector";
    aco_.touch_links.push_back(robot_type_ + "_end_effector");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_1");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_2");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_3");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_tip_1");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_tip_2");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_tip_3");
    pub_aco_.publish(aco_);
}

// TODO: Helferfunktion für Poses
void PickPlace::define_cartesian_pose() {
    tf::Quaternion q;

    // define grasp pose
    grasp_pose.header.frame_id = "root";
    grasp_pose.header.stamp = ros::Time::now();

    // Euler_ZYZ (-M_PI/4, M_PI/2, M_PI/2)
    grasp_pose.pose.position.x = 0.0  + cup_x;
    grasp_pose.pose.position.y = 0.65 + cup_y;
    grasp_pose.pose.position.z = 0.6;

    transport_pose.header.frame_id = "root";
    transport_pose.header.stamp = ros::Time::now();
    transport_pose.pose.position.x = 0;
    transport_pose.pose.position.y = 0.55;
    transport_pose.pose.position.z = 0.40;

    q = EulerZYZ_to_Quaternion(M_PI / 4, M_PI / 2, M_PI / 2);
    transport_pose.pose.orientation.x = q.x();
    transport_pose.pose.orientation.y = q.y();
    transport_pose.pose.orientation.z = q.z();
    transport_pose.pose.orientation.w = q.w();

    grasp_pose = generate_gripper_align_pose(grasp_pose, 0.04, M_PI / 4, M_PI / 2, M_PI / 2);
    pregrasp_pose = generate_gripper_align_pose(grasp_pose, 0.1, M_PI / 4, M_PI / 2, M_PI / 2);
    postgrasp_pose = grasp_pose;
    postgrasp_pose.pose.position.z = grasp_pose.pose.position.z + 0.15;
}

void PickPlace::define_joint_values() {
    start_joint.resize(joint_names_.size());
    start_joint[0] = M_PI * 1.0 / 2.0;
    start_joint[1] = M_PI * 1.0 / 3.0; // pi = gerade ausstrecken
    start_joint[2] = M_PI * 1.0 / 2.0; // pi = gerade ausstrecken
    start_joint[3] = 0;
    start_joint[4] = M_PI;
    start_joint[5] = M_PI;
}

/**
 * @brief PickPlace::generate_gripper_align_pose
 * @param targetpose_msg pick/place pose (object location): where gripper
 * close/open the fingers (grasp/release the object). Only position information
 * is used.
 * @param dist distance of returned pose to targetpose
 * @param azimuth an angle measured from the x-axis in the xy-plane in spherical
 * coordinates, denoted theta (0<= theta < 2pi ).
 * @param polar also named zenith, colatitude, denoted phi (0<=phi<=pi). It is
 * the angle from the positive z-axis to the vector.  phi= pi/2 - delta where
 * delta is the latitude.
 * @param rot_gripper_z rotation along the z axis of the gripper reference frame
 * (last joint rotation)
 * @return a pose defined in a spherical coordinates where origin is located at
 * the target pose. Normally it is a pre_grasp/post_realease pose, where gripper
 * axis (last joint axis) is pointing to the object (target_pose).
 */
 // TODO: inhalt prüfen
geometry_msgs::PoseStamped PickPlace::generate_gripper_align_pose(geometry_msgs::PoseStamped targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z) {
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.frame_id = "root";

    // computer pregrasp position w.r.t. location of grasp_pose in spherical
    // coordinate. Orientation is w.r.t. fixed world (root) reference frame.
    double delta_x = -dist * cos(azimuth) * sin(polar);
    double delta_y = -dist * sin(azimuth) * sin(polar);
    double delta_z = -dist * cos(polar);

    // computer the orientation of gripper w.r.t. fixed world (root) reference
    // frame. The gripper (z axis) should point(open) to the grasp_pose.
    tf::Quaternion q = EulerZYZ_to_Quaternion(azimuth, polar, rot_gripper_z); // TODO: convert to zxz

    pose_msg.pose.position.x = targetpose_msg.pose.position.x + delta_x;
    pose_msg.pose.position.y = targetpose_msg.pose.position.y + delta_y;
    pose_msg.pose.position.z = targetpose_msg.pose.position.z + delta_z;
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": "
                                         << "pose_msg: x "
                                         << pose_msg.pose.position.x << ", y "
                                         << pose_msg.pose.position.y << ", z "
                                         << pose_msg.pose.position.z << ", qx "
                                         << pose_msg.pose.orientation.x << ", qy "
                                         << pose_msg.pose.orientation.y << ", qz "
                                         << pose_msg.pose.orientation.z << ", qw "
                                         << pose_msg.pose.orientation.w);

    return pose_msg;
}

void PickPlace::evaluate_plan(moveit::planning_interface::MoveGroupInterface &group) {
    int count = 0;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    result_ = false;

    // try to find a success plan.
    double plan_time = 60;
    ROS_INFO("Setting plan time to %f sec", plan_time);
    group.setPlanningTime(plan_time);
    result_ = (group.plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
    ros::WallDuration(0.1).sleep();

    if (result_ == true) {
        group.execute(my_plan);
        ros::WallDuration(1.0).sleep();
    } else {
        std::cerr << "Exit since plan failed" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void PickPlace::setup_orientation_constraint(geometry_msgs::Pose target){
    moveit_msgs::Constraints grasp_constrains;
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = robot_type_ + "_end_effector";
    ocm.header.frame_id = "root";
    ocm.orientation = target.orientation;
    ocm.absolute_x_axis_tolerance = M_PI/10;
    ocm.absolute_y_axis_tolerance = M_PI/10;
    ocm.absolute_z_axis_tolerance = 2*M_PI;
    ocm.weight = 0.5;
    grasp_constrains.orientation_constraints.push_back(ocm);
    group_->setPathConstraints(grasp_constrains);
}

bool PickPlace::my_pick() {
    clear_workscene();
    ros::WallDuration(1.0).sleep();
    gripper_group_->setNamedTarget("Open");
    gripper_group_->move();

    group_->setJointValueTarget(start_joint);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Motion planning in cartesian space without obstacles ...");
    build_workscene();
    ros::WallDuration(0.1).sleep();
    ROS_INFO_STREAM("Planning to go to pre-grasp position ...");
    group_->setPoseTarget(pregrasp_pose);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Approaching grasp position ...");
    group_->setPoseTarget(grasp_pose);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Grasping ...");
    add_attached_obstacle();
    gripper_action(0.75 * FINGER_MAX); // partially close

    replaceTable(co_, pub_co_, planning_scene_msg_,"table", 0, 1.5);

    setup_orientation_constraint(postgrasp_pose.pose);
    ROS_INFO_STREAM("Planning to return to post grasp  ...");
    group_->setPoseTarget(postgrasp_pose);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Retruning to transport pose ...");
    setup_orientation_constraint(transport_pose.pose);
    group_->setPoseTarget(transport_pose);
//    group_->setJointValueTarget(start_joint);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Releasing gripper ...");
    gripper_action(0.0);

    clear_workscene();
    return true;
}

int main(int argc, char **argv) {

    signal(SIGINT, ctlc_handler);

    ros::init(argc, argv, "pick_place_demo");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    kinova::PickPlace pick_place(node);

    ros::spin();
    return 0;
}
