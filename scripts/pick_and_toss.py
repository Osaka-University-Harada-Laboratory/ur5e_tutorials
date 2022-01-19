#!/usr/bin/env python

import math
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander


def go_with_joint_values(mgc, jvs):
    """Executes the motions with joint values set."""
    mgc.set_joint_value_target(jvs)
    mgc.go()
    rospy.sleep(rospy.Duration.from_sec(1))


def pick_and_toss():
    """Executes pick and toss motion."""
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
    upright_r_jvs = [math.radians(180.0),
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

    # generate toss poses
    toss_start_jvs = [math.radians(180.0),
                      math.radians(-120.0),
                      math.radians(-85.0),
                      math.radians(-95.0),
                      math.radians(90.0),
                      math.radians(0.0)]

    toss_end_jvs = [math.radians(180.0),
                    math.radians(-120.0),
                    math.radians(-25.0),
                    math.radians(-45.0),
                    math.radians(90.0),
                    math.radians(0.0)]

    # execute pick and toss motion
    rospy.loginfo("Start pick and toss motion...")
    go_with_joint_values(manip, upright_jvs)
    cnt = 0
    while cnt < 1:
        rospy.loginfo("Pregrasp.")
        go_with_joint_values(manip, pregrasp_jvs)
        rospy.loginfo("Grasp.")
        go_with_joint_values(manip, grasp_jvs)
        go_with_joint_values(manip, pregrasp_jvs)
        go_with_joint_values(manip, upright_jvs)
        go_with_joint_values(manip, upright_r_jvs)
        rospy.loginfo("Start.")
        go_with_joint_values(manip, toss_start_jvs)
        rospy.loginfo("Release.")
        go_with_joint_values(manip, toss_end_jvs)
        cnt += 1

    # initialize joints
    go_with_joint_values(manip, upright_r_jvs)
    go_with_joint_values(manip, upright_jvs)
    rospy.signal_shutdown("Finished.")


if __name__ == '__main__':
    try:
        pick_and_toss()
    except rospy.ROSInterruptException:
        pass
