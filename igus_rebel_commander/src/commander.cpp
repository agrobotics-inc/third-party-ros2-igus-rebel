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

// Author: Simone Giampà
// University: Politecnico di Milano
// Master Degree: Computer Science Engineering
// Laboratory: Artificial Intelligence and Robotics Laboratory (AIRLab)

// Code from the move group interface tutorial
// https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html
// other useful links:
// https://moveit.picknik.ai/main/doc/examples/motion_planning_pipeline/motion_planning_pipeline_tutorial.html
// https://moveit.picknik.ai/main/doc/examples/motion_planning_api/motion_planning_api_tutorial.html

#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("igus_rebel::commander");

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto commander_node = rclcpp::Node::make_shared("commander_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(commander_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    RCLCPP_INFO(LOGGER, "Starting Commander V2 with motion control");

    // BEGIN_TUTORIAL
    //
    // Setting up to start using a planning pipeline is pretty easy. Before we can load the planner, we need two objects,
    // a RobotModel and a PlanningScene.
    //
    // We will start by instantiating a RobotModelLoader object, which will look up the robot description on the ROS
    // parameter server and construct a RobotModel for us to use.
    const std::string PLANNING_GROUP = "rebel_arm";  // "rebel_arm" for simple robot movement
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(
        new robot_model_loader::RobotModelLoader(commander_node, "robot_description"));

    // move_group interface, handling everything for us
    moveit::planning_interface::MoveGroupInterface move_group(commander_node, PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // We can print the name of the reference frame for this robot.
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    // "flange" for simple robot movement, "toucher" for robot arm + camera
    std::string end_effector_link = move_group.getEndEffectorLink();
    RCLCPP_INFO(LOGGER, "End effector link: %s", end_effector_link.c_str());

    // We can get a list of all the groups in the robot:
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    /* We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information */
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader->getModel();

    // Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));

    // Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
    // group is useful for dealing with one set of joints at a time such as a left arm or a end effector
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    // Using the RobotModel we can construct a PlanningScene that maintains the state of the world (including the robot).
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // We will now construct a loader to load a planner, by name.
    // Note that we are using the ROS pluginlib library here.
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    // We will get the name of planning plugin we want to load
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.
    if (!commander_node->get_parameter("planning_plugin", planner_plugin_name))
        RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");
    try {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
    } catch (pluginlib::PluginlibException& ex) {
        RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
    }
    try {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, commander_node, commander_node->get_namespace())) {
            RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
        }
        RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
    } catch (pluginlib::PluginlibException& ex) {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (const auto& cls : classes)
            ss << cls << " ";
        RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
                     ex.what(), ss.str().c_str());
    }

    // Visualization
    // ^^^^^^^^^^^^^
    // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
    namespace rvt = rviz_visual_tools;
    // @param node - base frame - markers topic - robot model
    moveit_visual_tools::MoveItVisualTools visual_tools(commander_node, "base_link", "/rviz_visual_tools", move_group.getRobotModel());

    // extra options
    visual_tools.setPlanningSceneTopic("/move_group/monitored_planning_scene");
    visual_tools.loadPlanningSceneMonitor();
    visual_tools.enableBatchPublishing();

    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.25;
    visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XXLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // We can also use visual_tools to wait for user input
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // ------------------------------------------------ Pose Goal
    // ^^^^^^^^^
    // We will now create a motion plan request for the right arm of the igus rebel
    // specifying the desired pose of the end-effector as input.
    planning_interface::MotionPlanResponse res;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "igus_rebel_base_link";
    pose.pose.position.x = 0.1;
    pose.pose.position.y = -0.1;
    pose.pose.position.z = 0.7;
    // pose.pose.orientation.x = 0.47927573323249817;
    // pose.pose.orientation.y = -0.021937279030680656;
    // pose.pose.orientation.z = 0.8773083090782166;
    // pose.pose.orientation.w = 0.011984390206634998;
    pose.pose.orientation.x = -0.182926;
    pose.pose.orientation.y = -0.565131;
    pose.pose.orientation.z = -0.501989;
    pose.pose.orientation.w = 0.628627;

    // publish a coordinate axis corresponding to the pose with rviz visual tools
    visual_tools.publishAxisLabeled(pose.pose, "target_1");
    visual_tools.trigger();

    move_group.setStartState(*robot_state);
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.001);
    move_group.setPoseTarget(pose, end_effector_link);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(1.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::MoveItErrorCode response = move_group.plan(my_plan);

    moveit_msgs::msg::RobotTrajectory trajectory = my_plan.trajectory_;

    // show output of planned movement
    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal): result = %s", moveit::core::error_code_to_string(response).c_str());

    // Visualize the result
    // NOTE: display trajectory not working at the moment
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher =
        commander_node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(trajectory);
    display_publisher->publish(display_trajectory);

    /* Visualize the trajectory */
    RCLCPP_INFO(LOGGER, "Visualizing the trajectory");

    visual_tools.publishTrajectoryPath(trajectory, robot_state);
    visual_tools.trigger();
    // Wait for user input
    visual_tools.prompt("Press 'next' to move the robot");

    if (bool(response)) {
        RCLCPP_INFO(LOGGER, "moving the robot with cartesian space goal");
        move_group.execute(my_plan);
    } else {
        RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
    }

    // Wait for user input
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // ----------------------------------------------------- Second movement --------------------------------------

    // Joint Space Goals
    // ^^^^^^^^^^^^^^^^^
    // First, set the state in the planning scene to the final state of the last plan
    robot_state = move_group.getCurrentState();
    move_group.setStartState(*robot_state);

    robot_state->setJointGroupPositions(joint_model_group, trajectory.joint_trajectory.points.back().positions);

    // Now, setup a joint space goal
    moveit::core::RobotState goal_state(*robot_state);
    std::vector<double> joint_values = {0.5, 0.2, 0.1, 1.5, -0.7, 2.0};  // radians
    goal_state.setJointGroupPositions(joint_model_group, joint_values);

    bool valid_motion = move_group.setJointValueTarget(goal_state);
    if (!valid_motion) RCLCPP_ERROR(LOGGER, "Target joints outside their phyisical limits");

    // Get the pose of a specific link (e.g., the end-effector link).
    const Eigen::Isometry3d goal_pose = goal_state.getGlobalLinkTransform(end_effector_link);

    // use rviz visual tools to publish a coordinate axis corresponding to the goal pose defined
    visual_tools.publishAxisLabeled(goal_pose, "target_2");
    visual_tools.trigger();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_2;
    // optionally limit accelerations and velocity scaling
    // move_group.setMaxVelocityScalingFactor(0.05);
    // move_group.setMaxAccelerationScalingFactor(0.05);
    response = move_group.plan(my_plan_2);

    // visualizing the trajectory
    RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint goal): result = %s", moveit::core::error_code_to_string(response).c_str());


    visual_tools.publishTrajectoryPath(my_plan_2.trajectory_, my_plan_2.start_state_);
	visual_tools.trigger();

    visual_tools.prompt("Press 'next' to move the robot");

    if (bool(response)) {
        RCLCPP_INFO(LOGGER, "moving the robot with joint space goal");
        move_group.execute(my_plan_2);
    } else
        RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");

    // Wait for user input
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // ----------------------------------------------------- Third movement --------------------------------------
    // The objective is now to move the robot while taking into account the obstacles in the environment.
    // We will first define the obstacle, then plan to a new goal location and visualize the plan.
    // When the plan is visualized in RViz, you will see the robot move around the obstacle.

    // Now, let's define a collision object ROS message for the robot to avoid.
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "box1";

    // Define a box to add to the world.
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.05;
    primitive.dimensions[primitive.BOX_Y] = 0.9;
    primitive.dimensions[primitive.BOX_Z] = 0.4;

    // Define a pose for the box (specified relative to frame_id).
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.48;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    // (using a vector that could contain additional objects)
    RCLCPP_INFO(LOGGER, "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
    visual_tools.publishText(text_pose, "Add_object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' to plan the movement to the new goal position");

    // generate new goal pose for the robot
    pose.header.frame_id = "base_link";
    pose.pose.position.x = 0.2;
    pose.pose.position.y = -0.2;
    pose.pose.position.z = 0.2;
    pose.pose.orientation.x = 0.000000;
    pose.pose.orientation.y = 0.000000;
    pose.pose.orientation.z = 0.000000;
    pose.pose.orientation.w = 1.000000;

    // set robot state at the current position
    robot_state = move_group.getCurrentState();
    move_group.setStartState(*robot_state);

	// same parameters as the first position goal
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.001);
    move_group.setPoseTarget(pose, end_effector_link);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(1.0);

    // publish a coordinate axis corresponding to the pose with rviz visual tools
    visual_tools.publishAxisLabeled(pose.pose, "target_3");
    visual_tools.trigger();

    // Now, when we plan a trajectory it will avoid the obstacle
    // planning in cartesian space
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_3;
    response = move_group.plan(my_plan_3);
    bool success = (response == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Plan 3 result = %s", moveit::core::error_code_to_string(response).c_str());
    RCLCPP_INFO(LOGGER, "Visualizing plan 3 (pose goal move around cuboid) %s", success ? "" : "FAILED");

    visual_tools.publishText(text_pose, "Obstacle_Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryPath(my_plan_3.trajectory_, *robot_state);
    visual_tools.trigger();

    // Wait for user input
    visual_tools.prompt("Press 'next' to move the robot");

    if (success) {
        RCLCPP_INFO(LOGGER, "moving the robot with cartesian space goal");
        move_group.execute(my_plan_3);
    } else {
        RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
    }

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the movement is completed to end the simulation");

    RCLCPP_INFO(LOGGER, "Demo completed!");

    rclcpp::shutdown();
    return 0;
}