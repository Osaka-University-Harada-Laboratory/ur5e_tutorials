#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Pose, Quaternion, Vector3
from moveit_commander import RobotCommander, MoveGroupCommander
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def euler_to_quaternion(euler):
    """Converts euler angles to quaternion.

    euler: geometry_msgs/Vector3
    quaternion: geometry_msgs/Quaternion
    """
    q = quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def quaternion_to_euler(quaternion):
    """Converts quaternion to euler angles.

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = euler_from_quaternion(
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])


def go_with_joint_values(mgc, jvs):
    """Executes the motions with joint values set."""
    mgc.set_joint_value_target(jvs)
    mgc.go()
    rospy.sleep(rospy.Duration.from_sec(1))


def go_with_pose(mgc, pose):
    """Executes the motions with pose set."""
    mgc.set_pose_target(pose)
    mgc.go()
    rospy.sleep(rospy.Duration.from_sec(1))


def pick_and_place():
    """Executes pick and place motion."""
    rospy.init_node("moveit_command_sender", disable_signals=True)
    robot = RobotCommander()
    manip = MoveGroupCommander("manipulator")

    # get current status
    manip_initial_pose = manip.get_current_pose().pose
    manip_initial_joint_values = manip.get_current_joint_values()

    # set maximum velocity and acceleration
    manip.set_max_velocity_scaling_factor(0.1)
    manip.set_max_acceleration_scaling_factor(0.1)

    # upright pose
    upright_jvs = [math.radians(90.0),
                   math.radians(-90.0),
                   math.radians(0.0),
                   math.radians(0.0),
                   math.radians(90.0),
                   math.radians(0.0)]
    upright_inv_jvs = [math.radians(-90.0),
                       math.radians(-90.0),
                       math.radians(0.0),
                       math.radians(0.0),
                       math.radians(90.0),
                       math.radians(0.0)]

    # generate grasp poses
    pregrasp_jvs = [math.radians(90.0),
                    math.radians(-50.0),
                    math.radians(25.0),
                    math.radians(115.0),
                    math.radians(92.0),
                    math.radians(0.0)]

    grasp_jvs = [math.radians(90.0),
                 math.radians(-27.0),
                 math.radians(15.0),
                 math.radians(101.0),
                 math.radians(92.0),
                 math.radians(0.0)]

    # way points
    way1_jvs = [math.radians(90.0),
                math.radians(-110.0),
                math.radians(80.0),
                math.radians(100.0),
                math.radians(90.0),
                math.radians(0.0)]

    way2_jvs = way1_jvs
    way2_jvs[0] = math.radians(-60.0)

    # generate release poses
    prerelease_jvs = [math.radians(-60.0),
                      math.radians(-70.0),
                      math.radians(85.0),
                      math.radians(77.0),
                      math.radians(92.0),
                      math.radians(0.0)]

    release_jvs = [math.radians(-60.0),
                   math.radians(-64.0),
                   math.radians(95.0),
                   math.radians(60.0),
                   math.radians(92.0),
                   math.radians(0.0)]

    # execute pick and place motion
    rospy.loginfo("Start pick and place motion...")
    go_with_joint_values(manip, upright_jvs)
    cnt = 0
    while cnt < 1:
        rospy.loginfo("Pregrasp.")
        go_with_joint_values(manip, pregrasp_jvs)
        rospy.loginfo("Grasp.")
        go_with_joint_values(manip, grasp_jvs)
        go_with_joint_values(manip, pregrasp_jvs)
        go_with_joint_values(manip, upright_jvs)
        go_with_joint_values(manip, upright_inv_jvs)
        go_with_joint_values(manip, way1_jvs)
        go_with_joint_values(manip, way2_jvs)
        rospy.loginfo("Prerelease.")
        go_with_joint_values(manip, prerelease_jvs)
        rospy.loginfo("Release.")
        go_with_joint_values(manip, release_jvs)
        go_with_joint_values(manip, prerelease_jvs)
        cnt += 1

    # initialize joints
    go_with_joint_values(manip, upright_inv_jvs)
    go_with_joint_values(manip, upright_jvs)
    rospy.signal_shutdown("Finished.")


if __name__ == '__main__':
    try:
        pick_and_place()
    except rospy.ROSInterruptException:
        pass
