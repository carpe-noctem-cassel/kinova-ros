//
// Created by cn on 11.03.20.
//

#ifndef KINOVA_ARM_MOVEIT_DEMO_OBJECT_FACTORY_H
#define KINOVA_ARM_MOVEIT_DEMO_OBJECT_FACTORY_H

#include <moveit_msgs/PlanningScene.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <ros/ros.h>

struct Position {
    double x, y, z;

    Position(double x_, double y_, double z_) {
        x = x_;
        y = y_;
        z = z_;
    }
};

moveit_msgs::CollisionObject replaceObject(moveit_msgs::CollisionObject &collisionObject, ros::Publisher &pub, moveit_msgs::PlanningScene &planningScene, std::string name, unsigned char objectType, Position pos, std::vector<double> dimensions);

moveit_msgs::CollisionObject addObject(moveit_msgs::CollisionObject &collisionObject, ros::Publisher &pub, moveit_msgs::PlanningScene &planningScene, std::string name, unsigned char objectType, Position pos, std::vector<double> dimensions);

moveit_msgs::CollisionObject removeObject(moveit_msgs::CollisionObject &collisionObject, ros::Publisher &pub, moveit_msgs::PlanningScene &planningScene, std::string name);

#endif //KINOVA_ARM_MOVEIT_DEMO_OBJECT_FACTORY_H
