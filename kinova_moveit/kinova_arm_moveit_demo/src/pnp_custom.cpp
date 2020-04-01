#include <pick_place.h>
#include <ros/console.h>
#include <string>
#include "object_factory.h"

#include <tf_conversions/tf_eigen.h>

const double FINGER_MAX = 6400;

using namespace kinova;

tf::Quaternion EulerZYZ_to_Quaternion(double tz1, double ty, double tz2) {
    tf::Quaternion q;
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();

    rot_temp.setEulerYPR(tz1, 0.0, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(0.0, ty, 0.0);
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
    // sub_joint_.shutdown();
    // sub_pose_.shutdown();
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

    replaceObject(co_, pub_co_, planning_scene_msg_, "cup", shape_msgs::SolidPrimitive::CYLINDER, Position(0, 0.65, 0.6), {0.1, 0.04});

//    can_pose_.pose.position.x = co_.primitive_poses[0].position.x;
//    can_pose_.pose.position.y = co_.primitive_poses[0].position.y;
//    can_pose_.pose.position.z = co_.primitive_poses[0].position.z;

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

void PickPlace::check_collision() {
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    collision_request.max_contacts = 1000;

    collision_result.clear();
    planning_scene_->checkCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " collision");

    collision_request.group_name = "arm";
    collision_result.clear();
    planning_scene_->checkCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 3: Current state is " << (collision_result.collision ? "in" : "not in") << " collision");

    // check contact
    planning_scene_->checkCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 4: Current state is " << (collision_result.collision ? "in" : "not in") << " collision");
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for (it = collision_result.contacts.begin();
         it != collision_result.contacts.end(); ++it) {
        ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
    }

    // allowed collision matrix
    collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene_->getCurrentState();

    collision_detection::CollisionResult::ContactMap::const_iterator it2;
    for (it2 = collision_result.contacts.begin();
         it2 != collision_result.contacts.end(); ++it2) {
        acm.setEntry(it2->first.first, it2->first.second, true);
    }
    collision_result.clear();
    planning_scene_->checkCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 5: Current state is " << (collision_result.collision ? "in" : "not in") << " collision");
}

void PickPlace::define_cartesian_pose() {
    tf::Quaternion q;

    // define start pose before grasp
    start_pose_.header.frame_id = "root";
    start_pose_.header.stamp = ros::Time::now();
    start_pose_.pose.position.x = 0.5;
    start_pose_.pose.position.y = -0.5;
    start_pose_.pose.position.z = 0.5;

    q = EulerZYZ_to_Quaternion(-M_PI / 4, M_PI / 2, M_PI);
    start_pose_.pose.orientation.x = q.x();
    start_pose_.pose.orientation.y = q.y();
    start_pose_.pose.orientation.z = q.z();
    start_pose_.pose.orientation.w = q.w();

    // define grasp pose
    grasp_pose_.header.frame_id = "root";
    grasp_pose_.header.stamp = ros::Time::now();

    // Euler_ZYZ (-M_PI/4, M_PI/2, M_PI/2)
    grasp_pose_.pose.position.x = 0.0;
    grasp_pose_.pose.position.y = 0.65;
    grasp_pose_.pose.position.z = 0.6;

    transport_pose_.header.frame_id = "root";
    transport_pose_.header.stamp = ros::Time::now();
    transport_pose_.pose.position.x = 0.0;
    transport_pose_.pose.position.y = 0.65;
    transport_pose_.pose.position.z = 0.75;

    q = EulerZYZ_to_Quaternion(M_PI / 4, M_PI / 2, -M_PI / 2);
    transport_pose_.pose.orientation.x = q.x();
    transport_pose_.pose.orientation.y = q.y();
    transport_pose_.pose.orientation.z = q.z();
    transport_pose_.pose.orientation.w = q.w();

    // generate_pregrasp_pose(double dist, double azimuth, double polar, double
    // rot_gripper_z)
    grasp_pose_ = generate_gripper_align_pose(grasp_pose_, 0.04, M_PI / 4, M_PI / 2, -M_PI / 2);
    pregrasp_pose_ = generate_gripper_align_pose(grasp_pose_, 0.1, M_PI / 4, M_PI / 2, -M_PI / 2);
    postgrasp_pose_ = grasp_pose_;
    postgrasp_pose_.pose.position.z = grasp_pose_.pose.position.z + 0.05;
}

void PickPlace::define_joint_values() {
    start_joint_.resize(joint_names_.size());
    //    getInvK(start_pose_, start_joint_);
    start_joint_[0] = M_PI * 1.0 / 2.0;
    start_joint_[1] = M_PI * 1.0 / 3.0; // pi = gerade ausstrecken
    start_joint_[2] = M_PI * 1.0 / 3.0; // pi = gerade ausstrecken
    start_joint_[3] = M_PI * -3.0 / 2.0;
    start_joint_[4] = M_PI * 1.0 / 2.0;
    start_joint_[5] = M_PI * 1.0 / 2.0;

    grasp_joint_.resize(joint_names_.size());
    //    getInvK(grasp_pose, grasp_joint_);
    grasp_joint_[0] = 144.0 * M_PI / 180.0;
    grasp_joint_[1] = 256.5 * M_PI / 180.0;
    grasp_joint_[2] = 91.3 * M_PI / 180.0;
    grasp_joint_[3] = 163.8 * M_PI / 180.0;
    grasp_joint_[4] = 88.5 * M_PI / 180.0;
    grasp_joint_[5] = 151.3 * M_PI / 180.0;

    pregrasp_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, pregrasp_joint_);
    pregrasp_joint_[0] = 145.4 * M_PI / 180.0;
    pregrasp_joint_[1] = 253.7 * M_PI / 180.0;
    pregrasp_joint_[2] = 67.0 * M_PI / 180.0;
    pregrasp_joint_[3] = 151.0 * M_PI / 180.0;
    pregrasp_joint_[4] = 118.5 * M_PI / 180.0;
    pregrasp_joint_[5] = 141.7 * M_PI / 180.0;

    //    postgrasp_joint_ = pregrasp_joint_;
    postgrasp_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, postdefinegrasp_joint_);
    postgrasp_joint_[0] = 144 * M_PI / 180.0;
    postgrasp_joint_[1] = 249 * M_PI / 180.0;
    postgrasp_joint_[2] = 88 * M_PI / 180.0;
    postgrasp_joint_[3] = 165 * M_PI / 180.0;
    postgrasp_joint_[4] = 83 * M_PI / 180.0;
    postgrasp_joint_[5] = 151 * M_PI / 180.0;
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
geometry_msgs::PoseStamped PickPlace::generate_gripper_align_pose(
        geometry_msgs::PoseStamped targetpose_msg, double dist, double azimuth,
        double polar, double rot_gripper_z) {
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.frame_id = "root";

    // computer pregrasp position w.r.t. location of grasp_pose in spherical
    // coordinate. Orientation is w.r.t. fixed world (root) reference frame.
    double delta_x = -dist * cos(azimuth) * sin(polar);
    double delta_y = -dist * sin(azimuth) * sin(polar);
    double delta_z = -dist * cos(polar);

    // computer the orientation of gripper w.r.t. fixed world (root) reference
    // frame. The gripper (z axis) should point(open) to the grasp_pose.
    tf::Quaternion q = EulerZYZ_to_Quaternion(azimuth, polar, rot_gripper_z);

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

void PickPlace::check_constrain() {
    moveit_msgs::Constraints grasp_constrains = group_->getPathConstraints();
    bool has_constrain = false;
    ROS_INFO("check constrain result: ");
    if (!grasp_constrains.orientation_constraints.empty()) {
        has_constrain = true;
        ROS_INFO("Has orientation constrain. ");
    }
    if (!grasp_constrains.position_constraints.empty()) {
        has_constrain = true;
        ROS_INFO("Has position constrain. ");
    }
    if (has_constrain == false) {
        ROS_INFO("No constrain. ");
    }
}

void PickPlace::evaluate_plan(moveit::planning_interface::MoveGroupInterface &group) {
    bool replan = true;
    int count = 0;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    while (replan == true && ros::ok()) {
        // reset flag for replan
        count = 0;
        result_ = false;

        // try to find a success plan.
        double plan_time;
        while (result_ == false && count < 5) {
            count++;
            plan_time = 20 + count * 10;
            ROS_INFO("Setting plan time to %f sec", plan_time);
            group.setPlanningTime(plan_time);
            result_ = (group.plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
            std::cout << "at attemp: " << count << std::endl;
            ros::WallDuration(0.1).sleep();
        }

        // found a plan
        if (result_ == true) {
            std::cout << "plan success at attemp: " << count << std::endl;

            replan = false;
            std::cout << "please input e to execute the plan, r to replan, q to quit, others to skip: ";
            std::cin >> pause_;
            ros::WallDuration(0.5).sleep();

            if (pause_ == "r" || pause_ == "R") {
                replan = true;
            } else {
                replan = false;
            }

            if (pause_ == "q" || pause_ == "Q") {
                exit(0);
            }
        } else // not found
        {
            std::cout << "Exit since plan failed until reach maximum attempt: " << count << std::endl;
            replan = false;
            break;
        }
    }

    if (result_ == true) {
        if (pause_ == "e" || pause_ == "E") {
            group.execute(my_plan);
        }
    }
    ros::WallDuration(1.0).sleep();
}

void PickPlace::setup_orientation_constraint(geometry_msgs::Pose target){
    moveit_msgs::Constraints grasp_constrains;
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = robot_type_ + "_end_effector";
    ocm.header.frame_id = "root";
    ocm.orientation = target.orientation;
    ocm.absolute_x_axis_tolerance = 2*M_PI;
    ocm.absolute_y_axis_tolerance = 2*M_PI;
    ocm.absolute_z_axis_tolerance = M_PI/10;
    ocm.weight = 0.5;
    grasp_constrains.orientation_constraints.push_back(ocm);
    group_->setPathConstraints(grasp_constrains);
}

bool PickPlace::my_pick() {
    clear_workscene();
    ros::WallDuration(1.0).sleep();
    gripper_group_->setNamedTarget("Open");
    gripper_group_->move();

    group_->setJointValueTarget(start_joint_);
    evaluate_plan(*group_);
    ROS_INFO_STREAM("Motion planning in cartesian space without obstacles ...");
    clear_workscene();
    build_workscene();
    ros::WallDuration(0.1).sleep();
    ROS_INFO_STREAM("Planning to go to pre-grasp position ...");
    group_->setPoseTarget(pregrasp_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Approaching grasp position ...");
    group_->setPoseTarget(grasp_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Grasping ...");
    add_attached_obstacle();
    gripper_action(0.75 * FINGER_MAX); // partially close

//    setup_orientation_constraint(postgrasp_pose_.pose);

    ROS_INFO_STREAM("Planning to return to start position  ...");
    group_->setPoseTarget(postgrasp_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Retruning to start pose ...");
//    setup_orientation_constraint(transport_pose_.pose);
    group_->setPoseTarget(transport_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Releasing gripper ...");
    gripper_action(0.0);

    clear_workscene();

    ROS_INFO_STREAM("Press any key to quit ...");
    std::cin >> pause_;
    return true;
}

void PickPlace::getInvK(geometry_msgs::Pose &eef_pose,
                        std::vector<double> &joint_value) {
    // TODO: transform cartesian command to joint space, and alway motion plan in
    // joint space.
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_place_demo");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    kinova::PickPlace pick_place(node);

    ros::spin();
    return 0;
}
