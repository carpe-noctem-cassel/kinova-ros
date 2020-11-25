#!/usr/bin/env python2
import roslib
roslib.load_manifest('kinova_arm_moveit_demo')
import rospy
import math
import tf
import geometry_msgs.msg
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import numpy as np
import quaternion
import scipy.spatial.transform.rotation


if __name__ == '__main__':
    rospy.init_node('camera_tf_listener')

    listener = tf.TransformListener()
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    rate = rospy.Rate(100.0)

    rospy.wait_for_service('/gazebo/set_model_state')
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/root', '/j2n6s300_link_5', rospy.Time(0))
        except tf.LookupException:
          continue
        except tf.ConnectivityException:
          print("Bad connection")
          continue
        except tf.ExtrapolationException:
          print("fail")
          continue

        # Match arm and camera
        rot = np.quaternion(rot[0], rot[1], rot[2], rot[3])
        tmp = scipy.spatial.transform.rotation.Rotation.from_euler("xyz", [np.pi, - np.pi / 4, 0], degrees=False).as_quat()
        tmp = np.quaternion(tmp[0], tmp[1], tmp[2], tmp[3])
        tmp = tmp * rot
        tmp = quaternion.as_float_array(tmp)


        print(rot, tmp)

        state_msg = ModelState()
        state_msg.model_name = 'fakeBot'
        state_msg.pose.position.x = trans[0]
        state_msg.pose.position.y = trans[1]
        state_msg.pose.position.z = trans[2]
        state_msg.pose.orientation.x = tmp[0]
        state_msg.pose.orientation.y = tmp[1]
        state_msg.pose.orientation.z = tmp[2]
        state_msg.pose.orientation.w = tmp[3]
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
          print("no state set")

        # print "Tralslation ", trans
        # print "Rotation    ", rot

        rate.sleep()
