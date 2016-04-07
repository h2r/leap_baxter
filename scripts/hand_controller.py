#!/usr/bin/env python

import argparse
import sys
import os

import rospy
import baxter_interface
from copy import deepcopy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

from leap_client.msg import HandInfoList

is_init = False
still_count = 0
start_position = {'right_s0': 0.9223059476623536, 'right_s1': -0.728640873413086, 'right_w0': 0.041033986029052734, 'right_w1': 0.8291166149047852, 'right_w2': -0.03067961572265625, 'right_e0': -0.07708253450317383, 'right_e1': 1.145883647241211}
global last_time
global base_hand_pose
global base_robot_pose
global right_limb

def ik_solve(limb, pos, orient):
    #~ rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    #print "iksvc: ", iksvc
    #print "ikreq: ", ikreq
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        str(limb): PoseStamped(header=hdr,
            pose=Pose(position=pos, orientation=orient))}
    #print poses
    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        #print limb_joints
        return limb_joints
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return -1

def pose_difference(pose1, pose2):
    
    return_pose = deepcopy(pose1)

    return_pose.position.x = - pose1.position.z + pose2.position.z
    return_pose.position.y = - pose1.position.x + pose2.position.x
    return_pose.position.z = pose1.position.y - pose2.position.y

    return_pose.orientation.x = pose1.orientation.x - pose2.orientation.x
    return_pose.orientation.y = pose1.orientation.y - pose2.orientation.y
    return_pose.orientation.z = pose1.orientation.z - pose2.orientation.z
    return_pose.orientation.w = pose1.orientation.w - pose2.orientation.w

    return return_pose

def arm_to_pose_sum(arm, pose):

    return_pose = deepcopy(pose)

    return_pose.position.x = pose.position.x + arm['position'].x
    return_pose.position.y = pose.position.y + arm['position'].y
    return_pose.position.z = pose.position.z + arm['position'].z
    #print pose.position.x
    #print arm['position'].x
    #print return_pose.position.x


    return_pose.orientation.x = pose.orientation.x + arm['orientation'].x
    return_pose.orientation.y = pose.orientation.y + arm['orientation'].y
    return_pose.orientation.z = pose.orientation.z + arm['orientation'].z
    return_pose.orientation.w = pose.orientation.w + arm['orientation'].w

    return return_pose

def convert_from_mm_to_m(pose):
    pose.position.x /= 1000
    pose.position.y /= 1000
    pose.position.z /= 1000

    return pose

def hand_control(data):
    global is_init
    global last_time
    global base_hand_pose
    global still_count

    if not is_init:
        # check tht the velocity of the hand is near zero
        is_still = pow(data.hands[0].velocity.x,2) + \
            pow(data.hands[0].velocity.y,2) + pow(data.hands[0].velocity.z,2) < 1
        if is_still:
            still_count += 1
            if still_count > 20:
                # record the base positon of the hand at init time
                print "init is TRUE"
                is_init = True
                base_hand_pose = data.hands[0].pose
                still_count = 0
        
    else: 
        # first check that the arm isn't coming from out of range
        cur_time = rospy.get_time()
        if cur_time - last_time > 1:
            is_init = False
            last_time = cur_time
            print "init is FALSE"
            right_limb.move_to_joint_positions(start_position)
            return
        # do this only every 100 ms
        if int(cur_time*10) % 10 == 0:


            # calculate the difference between the current location and the base location
            difference = pose_difference(data.hands[0].pose, base_hand_pose)
            print "difference"
            print difference.position
            print "base hand pose"
            print base_hand_pose.position
            print "cur hand pose"
            print data.hands[0].pose.position

            new_arm_pose = arm_to_pose_sum(base_robot_pose, convert_from_mm_to_m(difference))
            joint_angles = ik_solve('right',new_arm_pose.position, base_robot_pose["orientation"])
            if joint_angles != -1:
                right_limb.move_to_joint_positions(joint_angles)

    last_time = rospy.get_time()

def hand_listener():
    #rospy.init_node("hand_listener")
    rospy.Subscriber("/leap/hand_info", HandInfoList, hand_control)
    rospy.spin()

def main():
    global base_robot_pose
    global right_limb

    os.system("rosrun baxter_tools enable_robot.py -e")

    rospy.init_node('leap_baxter_interface')

    right_limb = baxter_interface.Limb('right')

    
    right_limb.move_to_joint_positions(start_position)
    base_robot_pose = right_limb.endpoint_pose()
    print base_robot_pose['position'].x
    print base_robot_pose['position'].y
    print base_robot_pose['position'].z
    
    hand_listener()


    

if __name__ == "__main__":
    main()


    
