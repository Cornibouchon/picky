//
// Created by patrice on 17.02.21.

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// Used for the convertion from euler to quaternion see: http://wiki.ros.org/tf2/Tutorials/Quaternions
#include <tf2/LinearMath/Quaternion.h>
// Used to  transform between tf2_quaternion and geometry_msgs quaternion
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sstream>
#include <iostream>
#include <math.h>
using namespace std;
#include <stdio.h>
#define PI 3.14159265

//Used to import the custom message from marker1
#include "std_msgs/String.h"
//Include the custom message point_msg.cpp
#include <marker1/point_msg.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_controller");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Set the planning Group
    static const std::string PLANNING_GROUP = "gripx_arm";

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Getting Basic Information
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));


    //Set a different planner id:
    // move_group.setPlannerId("RRTConnectkConfigDefault");


    //Define a collision box to ensure no other strawberries are harmed during retrieval
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "box1";

    //Define the box
    shape_msgs::SolidPrimitive primitive, primitive1, primitive2;
    primitive.type = primitive.BOX;
    primitive1.type = primitive1.BOX;
    primitive2.type = primitive1.BOX;
    primitive.dimensions.resize(3);
    primitive1.dimensions.resize(3);
    primitive2.dimensions.resize(3);

    double x_offset =0.7;
    // Define the box dimensions
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 2;
    primitive.dimensions[2] = 1.5;

    primitive1.dimensions[0] = 2*x_offset - primitive.dimensions[0];
    primitive1.dimensions[1] = 0.1;
    primitive1.dimensions[2] = 1.5;

    primitive2.dimensions[0] = 1;
    primitive2.dimensions[1] = 1;
    primitive2.dimensions[2] = 0.1;

    //Define the box pose relative to frame id
    geometry_msgs::Pose box_pose, box_pose1, box_pose2, box_pose3, box_pose4;
    box_pose.orientation.w = 1;
    box_pose.position.x = x_offset;
    box_pose.position.y = 0;
    box_pose.position.z = primitive.dimensions[2]/2;

    box_pose1.orientation.w = 1;
    box_pose1.position.x = -x_offset;
    box_pose1.position.y = 0;
    box_pose1.position.z = primitive.dimensions[2]/2;

    box_pose2.orientation.w = 1;
    box_pose2.position.x = 0;
    box_pose2.position.y = -0.15;
    box_pose2.position.z = primitive1.dimensions[2]/2;

    box_pose3.orientation.w = 1;
    box_pose3.position.x = 0.4 + primitive2.dimensions[1]/2;
    box_pose3.position.y = 0;
    box_pose3.position.z = 0.6 + primitive2.dimensions[2]/2;

    box_pose4.orientation.w = 1;
    box_pose4.position.x = -(0.4 + primitive2.dimensions[1]/2);
    box_pose4.position.y = 0;
    box_pose4.position.z = 0.6 + primitive2.dimensions[2]/2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose1);
    collision_object.operation = collision_object.ADD;
    collision_object.primitives.push_back(primitive1);
    collision_object.primitive_poses.push_back(box_pose2);
    collision_object.primitives.push_back(primitive2);
    collision_object.primitive_poses.push_back(box_pose3);
    collision_object.primitives.push_back(primitive2);
    collision_object.primitive_poses.push_back(box_pose4);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    //Display the box
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    ros::shutdown();
    return 0;
}
