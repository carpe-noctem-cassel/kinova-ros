#include "object_factory.h"

moveit_msgs::CollisionObject replaceObject(moveit_msgs::CollisionObject &collisionObject, ros::Publisher &pub, moveit_msgs::PlanningScene &planningScene, std::string name, unsigned char objectType, Position pos, std::vector<double> dimensions) {
    removeObject(collisionObject, pub, planningScene, name);
    return addObject(collisionObject, pub, planningScene, name, objectType, pos, dimensions);
}

moveit_msgs::CollisionObject addObject(moveit_msgs::CollisionObject &collisionObject, ros::Publisher &pub, moveit_msgs::PlanningScene &planningScene, std::string name, unsigned char objectType, Position pos, std::vector<double> dimensions) {
    collisionObject.primitives.resize(1);
    collisionObject.primitive_poses.resize(1);
    collisionObject.primitives[0].type = objectType;
    collisionObject.operation = moveit_msgs::CollisionObject::ADD;

    switch (objectType) {
        case shape_msgs::SolidPrimitive::BOX:
            collisionObject.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
            collisionObject.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = dimensions[0];
            collisionObject.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = dimensions[1];
            collisionObject.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = dimensions[2];
            break;

        case shape_msgs::SolidPrimitive::CYLINDER:
            collisionObject.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
            collisionObject.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = dimensions[0];
            collisionObject.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = dimensions[1];
            break;

        default:
            std::cerr << objectType << "? what?" << std::endl;
    }

    collisionObject.primitive_poses[0].position.x = pos.x;
    collisionObject.primitive_poses[0].position.y = pos.y;
    collisionObject.primitive_poses[0].position.z = pos.z;
    pub.publish(collisionObject);
    planningScene.world.collision_objects.push_back(collisionObject);
    planningScene.is_diff = true;
    return collisionObject;
}

moveit_msgs::CollisionObject removeObject(moveit_msgs::CollisionObject &collisionObject, ros::Publisher &pub, moveit_msgs::PlanningScene &planningScene, std::string name) {
    collisionObject.id = name;
    collisionObject.operation = moveit_msgs::CollisionObject::REMOVE;
    pub.publish(collisionObject);
    planningScene.world.collision_objects.push_back(collisionObject);
    planningScene.is_diff = true;
    return collisionObject;
}

