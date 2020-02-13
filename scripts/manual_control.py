#!/usr/bin/env python

import argparse
import intera_interface
import rospy


parser = argparse.ArgumentParser()
parser.add_argument("-sa", "--angles", type=float,
        nargs='+', default=None,
        help="Set joint angles as order [J0...J6]")
parser.add_argument("-sp", "--position", type=float,
        nargs='+', default=[0.50335, 0.046513, 0.06],
        help="Set end-effector to position: x, y, z")
parser.add_argument("-so", "--orientation", type=float,
        nargs='+', default=[-0.00142460053167, 0.99999999999, -0.00177030764765, 0.00253311793936],
        help="Set end-effector to orientation: x, y, z, w")
parser.add_argument("-ga", "--get_angles", action='store_true',
        help="Get current angle of joints as order [J0...J6]")
parser.add_argument("-gp", "--get_pose", action='store_true',
        help="Get current pose of end-effector as [position, quarternion]")
parser.add_argument("-n", "--neutral", action='store_true',
        help="Move to neutral")
        
args = parser.parse_args()

def main():
    timeout = 5.0 # seconds
    
    rospy.init_node("move_joint")       # Init ROS node
    limb = intera_interface.Limb()      # Create Limb object from intera library
    gripper = intera_interface.Gripper()    # Create Gripper object from intera library
    
    joint_names = limb.joint_names()
    
    # Get pose
    if args.get_pose:
        current_pose = limb.endpoint_pose()
        print("[INFO] Current pose: [pos, ori] = [{}, {}, {}, {}, {}, {}, {}]".format(
            current_pose['position'].x, current_pose['position'].y, current_pose['position'].z,
            current_pose['orientation'].x, current_pose['orientation'].y, current_pose['orientation'].z, current_pose['orientation'].w))
    # Get angles
    if args.get_angles:
        current_angles = limb.joint_angles()
        print("[INFO] Current angles: [J0...J6] = [{}, {}, {}, {}, {}, {}, {}]".format(
            current_angles['right_j0'], current_angles['right_j1'], current_angles['right_j2'], current_angles['right_j3'],
            current_angles['right_j4'], current_angles['right_j5'], current_angles['right_j6']))
    
    # Set to neutral
    if args.neutral:
        print("Moving to neutral...")
        limb.move_to_neutral()
        exit()
        
    # Set angles
    if args.angles is not None:
        if len(args.angles) != len(joint_names):
            print('The number of joint_angles must be %d', len(joint_names))
            exit()
        else:
            joint = limb.joint_angles()
            i = 0
            for j in joint_names:
                joint[j] = float(args.angles[i])
                i += 1
        
        if not rospy.is_shutdown():
            print("Moving arm...")
            limb.move_to_joint_positions(joint, timeout=timeout)
        else:
            print('ROS is shutdown')

if __name__ == '__main__':
    main()
