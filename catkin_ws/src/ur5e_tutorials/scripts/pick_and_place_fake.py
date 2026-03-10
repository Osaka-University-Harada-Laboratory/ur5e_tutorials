#!/usr/bin/env python

import math
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander

from robotiq import robotiq


def go_with_joint_values(mgc, jvs):
    """Executes the motions with joint values set."""
    mgc.set_joint_value_target(jvs)
    mgc.go()
    rospy.sleep(rospy.Duration.from_sec(1))


def pick_and_place():
    """Executes pick and place motion."""
    rospy.init_node("moveit_command_sender", disable_signals=True)
    robot = RobotCommander()
    arm = MoveGroupCommander("manipulator")

    # initial setting for arm
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(0.1)

    # generate grasp poses
    pregrasp_jvs = [math.radians(90.0),
                    math.radians(-50.0),
                    math.radians(25.0),
                    math.radians(115.0),
                    math.radians(92.0),
                    math.radians(0.0)]
    grasp_jvs = [math.radians(90.0),
                 math.radians(-60.0),
                 math.radians(70.0),
                 math.radians(80.0),
                 math.radians(90.0),
                 math.radians(0.0)]

    # generate release poses
    prerelease_jvs = [math.radians(45.0),
                      math.radians(-70.0),
                      math.radians(65.0),
                      math.radians(95.0),
                      math.radians(90.0),
                      math.radians(0.0)]
    release_jvs = [math.radians(0.0),
                   math.radians(-70.0),
                   math.radians(85.0),
                   math.radians(75.0),
                   math.radians(90.0),
                   math.radians(0.0)]

    # execute pick and place motion
    rospy.loginfo("Start pick and place motion...")
    cnt = 0
    while cnt < 1:
        rospy.loginfo("Pregrasp.")
        go_with_joint_values(arm, pregrasp_jvs)
        rospy.loginfo("Grasp.")
        go_with_joint_values(arm, grasp_jvs)
        go_with_joint_values(arm, pregrasp_jvs)
        rospy.loginfo("Prerelease.")
        go_with_joint_values(arm, prerelease_jvs)
        rospy.loginfo("Release.")
        go_with_joint_values(arm, release_jvs)
        go_with_joint_values(arm, prerelease_jvs)
        cnt += 1

    # finalize
    rospy.signal_shutdown("Finished.")


if __name__ == '__main__':
    try:
        pick_and_place()
    except rospy.ROSInterruptException:
        pass
