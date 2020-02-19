#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <kinova_driver/kinova_ros_types.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_plan");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(10.0);

  // Setup
  moveit::planning_interface::MoveGroupInterface group("arm");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

//  tf::Pose Home;
//  Home.setOrigin(tf::Vector3(1.6, 1.0, 1.2));
//  Home.setRotation(kinova::EulerXYZ2Quaternion(4.7, 3.7, 1.2));
  std::map<std::string, double> joints;
  joints["j2n6s300_joint_1"] = 1.6;
  joints["j2n6s300_joint_2"] = 1.0;
  joints["j2n6s300_joint_3"] = 1.2;
  joints["j2n6s300_joint_4"] = 4.7;
  joints["j2n6s300_joint_5"] = 3.7;
  joints["j2n6s300_joint_6"] = 1.2;
  joints["j2n6s300_joint_finger_1"] = 0.02;
  joints["j2n6s300_joint_finger_2"] = 0.02;
  joints["j2n6s300_joint_finger_3"] = 0.02;
  group.setJointValueTarget(joints);

  group.move();
  group.move();
  group.move();

  std::map<std::string, double> joints2;
  joints2["j2n6s300_joint_1"] = 1.6;
  joints2["j2n6s300_joint_2"] = 1.6;
  joints2["j2n6s300_joint_3"] = 1.8;
  joints2["j2n6s300_joint_4"] = 2.2;
  joints2["j2n6s300_joint_5"] = 0.4;
  joints2["j2n6s300_joint_6"] = 0.6;
  joints2["j2n6s300_joint_finger_1"] = 0.02;
  joints2["j2n6s300_joint_finger_2"] = 0.02;
  joints2["j2n6s300_joint_finger_3"] = 0.02;
  group.setJointValueTarget(joints2);
//  tf::Pose random_pose;
//  random_pose.setOrigin(tf::Vector3(0.54882, -0.30854,  0.65841));
//  random_pose.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));

  // Planning to a Pose goal
//  geometry_msgs::Pose target_pose1;
//  tf::poseTFToMsg(random_pose, target_pose1);
//  group.setPoseTarget(target_pose1);

//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//  auto success = group.plan(my_plan);

//  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);

  ROS_INFO("Visualizing plan 1 (again)");
//  display_trajectory.trajectory_start = my_plan.start_state_;
//  display_trajectory.trajectory.push_back(my_plan.trajectory_);
//  display_publisher.publish(display_trajectory);
  /* Sleep to give Rviz time to visualize the plan. */
  std::cout << "waiting . . ." << std::endl;
  sleep(5.0);

  // Moving to a pose goal
  std::cout << "Starting to move . . ." << std::endl;
//  robot_state::RobotState start_state(*group.getCurrentState());
  group.move();
  group.move();
  group.move();
  std::cout << "Finisched moving!" << std::endl;

//  // Planning with Path Constraints
//
//  // We will reuse the old goal that we had and plan to it.
//  // Note that this will only work if the current state already
//  // satisfies the path constraints. So, we need to set the start
//  // state to a new pose.
//  robot_state::RobotState start_state(*group.getCurrentState());
//  std::cout << "Goten current state!" << std::endl;
//  geometry_msgs::Pose start_pose2; // start from Home pose of j2n6
//  tf::poseTFToMsg(Home, start_pose2);
//
//  const robot_state::JointModelGroup *joint_model_group =
//      start_state.getJointModelGroup(group.getName());
//  start_state.setFromIK(joint_model_group, start_pose2);
//  group.setStartState(start_state);
//  ROS_WARN("start_state after set: ");
//  start_state.printStatePositions();
//
//  // Path constraints can easily be specified for a link on the robot.
//  // Let's specify a path constraint and a pose goal for our group.
//  // First define the path constraint.
//  moveit_msgs::OrientationConstraint ocm;
//  ocm.link_name = "j2n6s300_end_effector";
//  ocm.header.frame_id = "j2n6s300_link_base";
//  ocm.orientation.x = 0.68463;
//  ocm.orientation.y = -0.22436;
//  ocm.orientation.z = 0.68808;
//  ocm.orientation.w = 0.086576;
//
//  ocm.absolute_x_axis_tolerance = 0.1;
//  ocm.absolute_y_axis_tolerance = 0.1;
//  ocm.absolute_z_axis_tolerance = 0.1;
//  ocm.weight = 1.0;
//
//  // Now, set it as the path constraint for the group.
//  moveit_msgs::Constraints test_constraints;
//  test_constraints.orientation_constraints.push_back(ocm);
//  group.setPathConstraints(test_constraints);
//
//  // Now we will plan to the earlier pose target from the new
//  // start state that we have just created.
//  group.setPoseTarget(target_pose1);
//  success = group.plan(my_plan);
//
//  ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
//  /* Sleep to give Rviz time to visualize the plan. */
//  sleep(10.0);
//
//  // When done with the path constraint be sure to clear it.
//  group.clearPathConstraints();
//
//  // Cartesian Paths
//  // ^^^^^^^^^^^^^^^
//  // You can plan a cartesian path directly by specifying a list of waypoints
//  // for the end-effector to go through. Note that we are starting
//  // from the new start state above.  The initial pose (start state) does not
//  // need to be added to the waypoint list.
//  std::vector<geometry_msgs::Pose> waypoints;
//
//  geometry_msgs::Pose target_pose3 = start_pose2;
//  target_pose3.position.x += 0.02;
//  target_pose3.position.z += 0.02;
//  waypoints.push_back(target_pose3);  // up and out
//
//  target_pose3.position.y -= 0.02;
//  waypoints.push_back(target_pose3);  // left
//
//  target_pose3.position.z -= 0.02;
//  target_pose3.position.y += 0.02;
//  target_pose3.position.x -= 0.02;
//  waypoints.push_back(target_pose3);  // down and right (back to start)
//
//  // We want the cartesian path to be interpolated at a resolution of 1 cm
//  // which is why we will specify 0.01 as the max step in cartesian
//  // translation.  We will specify the jump threshold as 0.0, effectively
//  // disabling it.
//  moveit_msgs::RobotTrajectory trajectory;
//  double fraction = group.computeCartesianPath(waypoints,
//                                               0.01,  // eef_step
//                                               0.0,   // jump_threshold
//                                               trajectory);
//
//  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
//           fraction * 100.0);
//  /* Sleep to give Rviz time to visualize the plan. */
//  sleep(15.0);
//
//
//  // Adding/Removing Objects and Attaching/Detaching Objects
//  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//  // First, we will define the collision object message.
//  moveit_msgs::CollisionObject collision_object;
//  collision_object.header.frame_id = group.getPlanningFrame();
//
//  /* The id of the object is used to identify it. */
//  collision_object.id = "box1";
//
//  /* Define a box to add to the world. */
//  shape_msgs::SolidPrimitive primitive;
//  primitive.type = primitive.BOX;
//  primitive.dimensions.resize(3);
//  // dimention less than the distance between start point and goal point
//  primitive.dimensions[0] = 0.1;
//  primitive.dimensions[1] = 0.02;
//  primitive.dimensions[2] = 0.05;
//
//  /* A pose for the box (specified relative to frame_id) */
//  geometry_msgs::Pose box_pose;
//  box_pose.orientation.w = 1.0;
//  // place between start point and goal point.
//  box_pose.position.x =  0.37;
//  box_pose.position.y = -0.27;
//  box_pose.position.z =  0.57;
//
//  collision_object.primitives.push_back(primitive);
//  collision_object.primitive_poses.push_back(box_pose);
//  collision_object.operation = collision_object.ADD;
//
//  std::vector<moveit_msgs::CollisionObject> collision_objects;
//  collision_objects.push_back(collision_object);
//
//  // Now, let's add the collision object into the world
//  ROS_INFO("Add an object into the world");
//  planning_scene_interface.addCollisionObjects(collision_objects);
//
//  /* Sleep so we have time to see the object in RViz */
//  sleep(2.0);
//
//  // Planning with collision detection can be slow.  Lets set the planning time
//  // to be sure the planner has enough time to plan around the box.  10 seconds
//  // should be plenty.
//  group.setPlanningTime(10.0);
//
//
//  // Now when we plan a trajectory it will avoid the obstacle
//  group.setStartState(*group.getCurrentState());
//  group.setPoseTarget(target_pose1);
//  success = group.plan(my_plan);
//
//  ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
//           success?"":"FAILED");
//  /* Sleep to give Rviz time to visualize the plan. */
//  sleep(10.0);
//
//
//  // Now, let's attach the collision object to the robot.
//  ROS_INFO("Attach the object to the robot");
//  group.attachObject(collision_object.id);
//  /* Sleep to give Rviz time to show the object attached (different color). */
//  sleep(4.0);
//
//  // Now, let's detach the collision object from the robot.
//  ROS_INFO("Detach the object from the robot");
//  group.detachObject(collision_object.id);
//  /* Sleep to give Rviz time to show the object detached. */
//  sleep(4.0);
//
//  // Now, let's remove the collision object from the world.
//  ROS_INFO("Remove the object from the world");
//  std::vector<std::string> object_ids;
//  object_ids.push_back(collision_object.id);
//  planning_scene_interface.removeCollisionObjects(object_ids);
//  /* Sleep to give Rviz time to show the object is no longer there. */
//  sleep(4.0);

  ros::shutdown();
  return 0;
}
