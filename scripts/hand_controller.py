#!/usr/bin/env python

import argparse
import sys
import os

import rospy
import baxter_interface

import numpy

from copy import deepcopy

import tf

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)
from std_msgs.msg import (
    Header,
    String
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

from leap_client.msg import HandInfoList

# global variables 
global last_time
global base_hand_pose
global base_robot_pose
global right_limb
global fx,fy,fz
global fqx, fqy, fqz, fqw
global right_gripper
global einPub
is_init = False
still_count = 0
cur_grip = False
last_switch_time = 0


# parameters
start_position = {
 'right_s0': 0.713684560748291,
 'right_s1': -0.728640873413086,
 'right_w0': 0.041033986029052734,
 'right_w1': 0.8291166149047852,
 'right_w2': -0.03067961572265625,
 'right_e0': -0.07708253450317383,
 'right_e1': 1.145883647241211}

start_ee_pose = (0.8018358404663602,
    -0.33435461249148715,
    0.17227074304924156,
    0.05420534191803939,
    0.9839693463837491,
    -0.028193083856791017,
    0.16754478895904237)

start_pose_msg = "%f %f %f %f %f %f %f moveToEEPose" % start_ee_pose

#start_position = {'right_s0': 0.713684560748291,
# 'right_s1': -1.0787719878479005,
# 'right_w0': -0.08245146725463867,
# 'right_w1': -0.7784952489624024,
# 'right_w2': 0.03911651004638672,
# 'right_e0': 0.07708253450317383,
# 'right_e1': 1.948155598388672}
t_decay = 0.92
q_decay = 0.92
scale_factor = 1.7

using_ein = False

def ik_solve(limb, pos, orient):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        str(limb): PoseStamped(header=hdr,
            pose=Pose(position=pos, orientation=orient))}
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
        return limb_joints
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return -1

def pose_difference(pose1, pose2):
    
    return_pose = deepcopy(pose1)

    return_pose.position.x = - pose1.position.z + pose2.position.z
    return_pose.position.y = - pose1.position.x + pose2.position.x
    return_pose.position.z = pose1.position.y - pose2.position.y


    xyzw_array = lambda o: numpy.array([o.x, o.y, o.z, o.w])

    return_pose.orientation = Quaternion(*tf.transformations.quaternion_inverse(
        tf.transformations.quaternion_multiply(xyzw_array(pose1.orientation),
            tf.transformations.quaternion_inverse(xyzw_array(pose2.orientation)))))

    return return_pose

def arm_to_pose_sum(arm, pose):

    return_pose = deepcopy(pose)

    return_pose.position.x = pose.position.x + arm['position'].x
    return_pose.position.y = pose.position.y + arm['position'].y
    return_pose.position.z = pose.position.z + arm['position'].z

    xyzw_array = lambda o: numpy.array([o.x, o.y, o.z, o.w])

    return_pose.orientation = Quaternion(*tf.transformations.quaternion_multiply(
        xyzw_array(pose.orientation),xyzw_array(arm["orientation"])))

    return return_pose

def convert_from_mm_to_m(pose):
    
    scale = 1000 / scale_factor

    pose.position.x /= scale
    pose.position.y /= scale
    pose.position.z /= scale

    return pose

def hand_control(data):
    global is_init
    global last_time
    global base_hand_pose
    global still_count
    global fx, fy, fz
    global fqx, fqy, fqz, fqw
    global cur_grip
    global last_switch_time
    global base_robot_pose

    if not is_init:
        if using_ein:
            einPub.publish(start_pose_msg)
            einPub.publish("shiftIntoGraspGear1")
        else:
            right_limb.move_to_joint_positions(start_position)

        # check tht the velocity of the hand is near zero
        is_still = pow(data.hands[0].velocity.x,2) + \
            pow(data.hands[0].velocity.y,2) + pow(data.hands[0].velocity.z,2) < 16
        if is_still:
            still_count += 1
            if still_count > 5:
                # record the base positon of the hand at init time
                print "init is TRUE"
                is_init = True
                base_hand_pose = data.hands[0].pose
                still_count = 0
        last_time = rospy.get_time()

        base_robot_pose = right_limb.endpoint_pose()
        fx, fy, fz = base_robot_pose["position"]
        fqx, fqy, fqz, fqw = base_robot_pose["orientation"]

        
    else: 
        # first check that the arm isn't coming from out of range
        cur_time = rospy.get_time()
        if cur_time - last_time > 2:
            is_init = False
            last_time = cur_time
            print "init is FALSE"
            return

        if cur_time - last_time > 0.01:

            if hand_is_fist(data.hands[0]) and False:
                base_hand_pose = data.hands[0].pose
                #prev_q = right_limb.endpoint_pose()["orientation"]
                base_robot_pose = right_limb.endpoint_pose()
                #base_robot_pose["orientation"] = prev_q
                fx, fy, fz = base_robot_pose["position"]
                fqx, fqy, fqz, fqw = base_robot_pose["orientation"]
                last_time = rospy.get_time()
                print "HAND IS IN FIST"
                return    
            # calculate the difference between the current location and the base location
            difference = pose_difference(data.hands[0].pose, base_hand_pose)

            # scale difference
            scaled_difference = convert_from_mm_to_m(difference)

            # calculate the new arm location relative to the base arm location
            new_arm_pose = arm_to_pose_sum(base_robot_pose, scaled_difference)
            
            # make new pose object
            filtered_pose = deepcopy(new_arm_pose)

            # lerp the positions
            fx = t_decay * fx + (1.0 - t_decay) * new_arm_pose.position.x
            fy = t_decay * fy + (1.0 - t_decay) * new_arm_pose.position.y
            fz = t_decay * fz + (1.0 - t_decay) * new_arm_pose.position.z


            # add the position data into pose object
            filtered_pose.position.x = fx
            filtered_pose.position.y = fy
            filtered_pose.position.z = fz

            # load orientation data into pose object
            filtered_pose.orientation.x = fqx
            filtered_pose.orientation.y = fqy
            filtered_pose.orientation.z = fqz
            filtered_pose.orientation.w = fqw
            
            xyzw_array = lambda o: numpy.array([o.x, o.y, o.z, o.w])

            #slerp orientation
            filtered_pose.orientation = Quaternion(*tf.transformations.quaternion_slerp(
                xyzw_array(filtered_pose.orientation),
                xyzw_array(new_arm_pose.orientation), 1 - q_decay))

            # save orientation in global variables
            fqx = filtered_pose.orientation.x
            fqy = filtered_pose.orientation.y
            fqz = filtered_pose.orientation.z
            fqw = filtered_pose.orientation.w
            
            if using_ein:
                msg = "%f %f %f %f %f %f %f moveToEEPose" % (fx, fy, fz, fqx, fqy, fqz, fqw)
                einPub.publish(msg)
            else:
                joint_angles = ik_solve('right',filtered_pose.position, filtered_pose.orientation)
                if joint_angles != -1:
                    right_limb.set_joint_positions(joint_angles)

            # get thumb and index fingertip position
            thumb_pos = data.hands[0].fingers[0].tip_position
            index_pos = data.hands[0].fingers[1].tip_position

            if fingers_touching(thumb_pos, index_pos):
                if cur_time - last_switch_time > 0.5:
                    if cur_grip == True:
                        right_gripper.open(block=True)
                        cur_grip = False
                        print "opening"
                    else:
                        right_gripper.close(block=True)
                        cur_grip = True
                        print "closing"
                last_switch_time = cur_time


            last_time = rospy.get_time()

def hand_is_fist(hand):
    in_fist = False
    avg_finger_dist = 0
    num_fingers = len(hand.fingers)-1
    for finger in hand.fingers[1:]:
        avg_finger_dist = avg_finger_dist + finger_distance(finger.tip_position, hand.pose.position)
    avg_finger_dist /= num_fingers
    if avg_finger_dist < 1500*num_fingers:
        in_fist = True
    #print in_fist
    return in_fist

def finger_distance(finger1, finger2):
    dist = (finger1.x - finger2.x)**2 + (finger1.y - finger2.y)**2 + (finger1.z - finger2.z)**2
    return dist

def fingers_touching(finger1,finger2):
    #print finger_distance(finger1,finger2)
    if finger_distance(finger1,finger2) < 2000:
        return True
    else:
        return False


def hand_listener():
    rospy.Subscriber("/leap/hand_info", HandInfoList, hand_control)
    rospy.spin()

def main():
    global base_robot_pose
    global right_limb
    global right_gripper
    global einPub

    os.system("rosrun baxter_tools enable_robot.py -e")

    rospy.init_node('leap_baxter_interface')

    right_limb = baxter_interface.Limb('right')
    right_gripper = baxter_interface.Gripper('right')

    # move limb to start position

    if using_ein:
        einPub = rospy.Publisher("/ein/right/forth_commands", String, queue_size = 0)
        rospy.sleep(1) # wait for einPub to boot up
        msg = "0.5 setSpeed 0.01 setGridSize 1 setCurrentIKFastMode ikModeService"
        einPub.publish(msg)
        einPub.publish(start_pose_msg)
        einPub.publish("shiftIntoGraspGear1")

    else:
        right_limb.move_to_joint_positions(start_position)
    
    base_robot_pose = right_limb.endpoint_pose()

    # gripper calibrating 
    if not right_gripper.calibrated():
        right_gripper.calibrate()
    right_gripper.open()
   

    print "Ready to pair with hand"
    
    hand_listener()


    

if __name__ == "__main__":
    main()
