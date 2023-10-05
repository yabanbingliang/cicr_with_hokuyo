#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
import numpy as np
import math
import tf.transformations as tft


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return [x, y, z, w]





def main():
    rospy.init_node('vec_test')

    model_odom_pub = rospy.Publisher('/position_control',Odometry,queue_size=10)
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    body_pose_msg = ModelState() #uav body pose
    last_body_pose_msg = ModelState() #
    odom = Odometry()
    rospy.logwarn("Vec_test Load Sucessfully")


    while not rospy.is_shutdown():
        drone_state = get_model_state('ardrone','world')
        rospy.sleep(0.01)

        linear_vec_x = 0.6

        # orientation ---> euler
        quaternion_uav = (drone_state.pose.orientation.w, drone_state.pose.orientation.x, drone_state.pose.orientation.y, drone_state.pose.orientation.z)
        euler_angles = tft.euler_from_quaternion(quaternion_uav, 'sxyz') # yaw
        if(euler_angles[0]>=0):
            uav_yaw = math.pi - euler_angles[0]
        if(euler_angles[0]<0):
            uav_yaw = -math.pi - euler_angles[0]

        # body ---> world
        rotation_matrix_z = np.matrix([[math.cos(uav_yaw),-math.sin(uav_yaw),0],[math.sin(uav_yaw),math.cos(uav_yaw),0],[0,0,1]])
        body_vel_matrix = np.matrix([linear_vec_x, 0, 0])
        body_vel_matrix_trans = body_vel_matrix.T
        world_vel_matrix = rotation_matrix_z.dot(body_vel_matrix_trans)




        odom.pose.pose.position.x = drone_state.pose.position.x
        odom.pose.pose.position.y = drone_state.pose.position.y
        odom.pose.pose.position.z = 5

        odom.pose.pose.orientation.w = drone_state.pose.orientation.w
        odom.pose.pose.orientation.x = drone_state.pose.orientation.x
        odom.pose.pose.orientation.y = drone_state.pose.orientation.y
        odom.pose.pose.orientation.z = drone_state.pose.orientation.z


        odom.twist.twist.linear.x = world_vel_matrix[0]
        odom.twist.twist.linear.y = world_vel_matrix[1]
        odom.twist.twist.linear.z = world_vel_matrix[2]


        odom.twist.twist.angular.x = 0
        odom.twist.twist.angular.y = 0
        # odom.twist.twist.angular.z = 0.6
        odom.twist.twist.angular.z = 0.6


        model_odom_pub.publish(odom)




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


