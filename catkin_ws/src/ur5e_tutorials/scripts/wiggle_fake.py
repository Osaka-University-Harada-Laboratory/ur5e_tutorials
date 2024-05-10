#!/usr/bin/env python

import math
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander


def show_joint_values(mgc):
    """Shows joint values with rospy log info."""
    rjs = mgc.get_current_joint_values()
    rospy.loginfo("Current joint angles.")
    rospy.loginfo("j1: %f", rjs[0])
    rospy.loginfo("j2: %f", rjs[1])
    rospy.loginfo("j3: %f", rjs[2])
    rospy.loginfo("j4: %f", rjs[3])
    rospy.loginfo("j5: %f", rjs[4])
    rospy.loginfo("j6: %f", rjs[5])


def go_with_joint_values(mgc, jvs):
    """Executes the motions with joint values set."""
    mgc.set_joint_value_target(jvs)
    mgc.go()
    rospy.sleep(rospy.Duration.from_sec(1))


def wiggle():
    """Executes wiggle motions."""
    rospy.init_node("moveit_command_sender", disable_signals=True)
    robot = RobotCommander()
    manip = MoveGroupCommander("manipulator")

    # set maximum velocity and acceleration
    manip.set_max_velocity_scaling_factor(0.3)
    manip.set_max_acceleration_scaling_factor(0.3)

    # generate joint values
    forward_jvs = [math.radians(90.0),
                   math.radians(-60.0),
                   math.radians(-30.0),
                   math.radians(-30.0),
                   math.radians(90.0),
                   math.radians(0.0)]
    backward_jvs = [math.radians(90.0),
                    math.radians(-120.0),
                    math.radians(30.0),
                    math.radians(30.0),
                    math.radians(90.0),
                    math.radians(0.0)]
    middle_jvs = [math.radians(90.0),
                  math.radians(-90.0),
                  math.radians(0.0),
                  math.radians(0.0),
                  math.radians(90.0),
                  math.radians(0.0)]

    # execute wiggle motions two times
    rospy.loginfo("Start wiggle motions.")

    rospy.loginfo("Moving to middle point...")
    go_with_joint_values(manip, middle_jvs)
    show_joint_values(manip)
    cnt = 0
    while cnt < 2:
        rospy.loginfo("Moving forward...")
        go_with_joint_values(manip, forward_jvs)
        show_joint_values(manip)
        rospy.loginfo("Moving backward...")
        go_with_joint_values(manip, backward_jvs)
        show_joint_values(manip)
        cnt += 1

    # initialize joints
    rospy.loginfo("Moving to middle point...")
    go_with_joint_values(manip, middle_jvs)
    show_joint_values(manip)
    rospy.signal_shutdown("Finished.")


if __name__ == '__main__':
    try:
        wiggle()
    except rospy.ROSInterruptException:
        pass
