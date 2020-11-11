#!/usr/bin/env python  
import roslib
roslib.load_manifest('kinova_arm_moveit_demo')
import rospy
import math
import tf
import geometry_msgs.msg
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState


def euler_to_quaternion(roll, pitch, yaw):

        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]


if __name__ == '__main__':
    print "Hello World"
    rospy.init_node('camera_tf_listener')

    listener = tf.TransformListener()
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    rate = rospy.Rate(100.0)

    # math.pi / -2     math.pi / 2    math.pi / -4
    transform_quaternion = euler_to_quaternion(0, 0, (math.pi / -15.0))

    rospy.wait_for_service('/gazebo/set_model_state')
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/root', '/j2n6s300_link_4', rospy.Time(0))
        except tf.LookupException:
          print "No lookup possible"
          continue
        except tf.ConnectivityException:
          print "Bad connection"
          continue
        except tf.ExtrapolationException:
          print "fail"
          continue

        state_msg = ModelState()
        state_msg.model_name = 'fakeBot'
        state_msg.pose.position.x = trans[0]
        state_msg.pose.position.y = trans[1]
        state_msg.pose.position.z = trans[2]
        state_msg.pose.orientation.x = rot[0] * transform_quaternion[0]
        state_msg.pose.orientation.y = rot[1] * transform_quaternion[1]
        state_msg.pose.orientation.z = rot[2] * transform_quaternion[2]
        state_msg.pose.orientation.w = rot[3] * transform_quaternion[3]
        print("Name:{}: {} {} {} | {} {} {} {}".format(state_msg.model_name,
        state_msg.pose.position.x,
        state_msg.pose.position.y,
        state_msg.pose.position.z,
        state_msg.pose.orientation.x,
        state_msg.pose.orientation.y,
        state_msg.pose.orientation.z,
        state_msg.pose.orientation.w))

        try:
          resp = set_state( state_msg )
          # print resp
        except:
          print "no state set"

        # print "Tralslation ", trans
        # print "Rotation    ", rot

        rate.sleep()