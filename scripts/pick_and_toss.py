#!/usr/bin/env python

import math
import rospy
import Queue as queue
from threading import Thread
from moveit_commander import RobotCommander, MoveGroupCommander

from robotiq import robotiq


class ThreadWithReturnValue(Thread):
    """Overwrites Tread class to get return values."""
    def __init__(self, group=None, target=None, name=None,
                args=(), kwargs={}, Verbose=None):
        Thread.__init__(self, group, target, name, args, kwargs, Verbose)
        self._return = None

    def run(self):
        if self._Thread__target is not None:
            self._return = self._Thread__target(
                *self._Thread__args, **self._Thread__kwargs)

    def join(self):
        Thread.join(self)
        return self._return


def go_with_joint_values(mgc, jvs):
    """Executes the motions with joint values set."""
    mgc.set_joint_value_target(jvs)
    mgc.go()
    rospy.sleep(rospy.Duration.from_sec(1))


def pick_and_toss():
    """Executes pick and toss motion."""
    rospy.init_node("moveit_command_sender", disable_signals=True)
    robot = RobotCommander()
    arm = MoveGroupCommander("manipulator")

    ## initial setting for arm
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(0.1)

    ## initial setting for gripper
    if use_gripper:
        gripper = robotiq()
        rospy.loginfo("Connect/Reset/Activate a connected gripper.")
        gripper.connect(ur5e_ip, tool_tcp_port)
        gripper.reset()
        gripper.activate()
        result = gripper.wait_activate_complete()
        if result != 0x31:
            rospy.loginfo("Failed open the gripper...")
            gripper.disconnect()
            return
        gripper.adjust()
        rospy.loginfo("Complete gripper's initialization.")

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

    # generate toss poses
    toss_start_jvs = [math.radians(90.0),
                      math.radians(-130.0),
                      math.radians(-100.0),
                      math.radians(-115.0),
                      math.radians(90.0),
                      math.radians(0.0)]
    toss_end_jvs = [math.radians(90.0),
                    math.radians(-110.0),
                    math.radians(-15.0),
                    math.radians(-35.0),
                    math.radians(90.0),
                    math.radians(0.0)]

    def arm_motions(is_grasp, is_toss):
        """Defines arm motions in a thread."""
        is_grasp.put(False)
        is_toss.put(False)

        rospy.loginfo("Start arm motions thread.")
        rospy.loginfo("Pregrasp.")
        go_with_joint_values(arm, pregrasp_jvs)
        rospy.loginfo("Grasp.")
        go_with_joint_values(arm, grasp_jvs)
        is_grasp.put(True)
        rospy.sleep(rospy.Duration.from_sec(1))
        go_with_joint_values(arm, pregrasp_jvs)

        rospy.loginfo("Start tossing.")
        go_with_joint_values(arm, toss_start_jvs)
        # arm.set_max_velocity_scaling_factor(1.0)
        # arm.set_max_acceleration_scaling_factor(1.0)
        rospy.sleep(rospy.Duration.from_sec(1))
        is_toss.put(True)
        rospy.loginfo("Release.")
        go_with_joint_values(arm, toss_end_jvs)
        rospy.loginfo("End arm motions thread.")

        # initialize joints
        # arm.set_max_velocity_scaling_factor(0.2)
        # arm.set_max_acceleration_scaling_factor(0.2)
        rospy.sleep(rospy.Duration.from_sec(1))
        return True

    def gripper_motions(is_grasp, is_toss):
        """Defines gripper motions in a thread."""
        rospy.loginfo("Start gripper motions thread.")

        # close at pregrasp
        while True:
            rospy.sleep(rospy.Duration.from_sec(0.1))
            if is_grasp.get():
                break
        gripper.move(255, 100, 50) # close
        (status, position, force) = gripper.wait_move_complete()

        # open after tossing start
        while True:
            rospy.sleep(rospy.Duration.from_sec(0.1))
            if is_toss.get():
                break
        rospy.sleep(rospy.Duration.from_sec(0.3))
        gripper.move(0, 255, 50)
        (status, position, force) = gripper.wait_move_complete()

        rospy.loginfo("End gripper motions thread.")
        gripper.disconnect()
        return True

    # initializing arm and gripper poses
    if use_gripper:
        gripper.move(0, 255, 0) # open fast
        (status, position, force) = gripper.wait_move_complete()

    is_grasp = queue.Queue()
    is_toss = queue.Queue()
    
    rospy.loginfo("Start pick and toss motion...")
    arm_thread = ThreadWithReturnValue(
        target=arm_motions, args=(is_grasp, is_toss))
    arm_thread.start()
    if use_gripper:
        gripper_thread = ThreadWithReturnValue(
            target=gripper_motions, args=(is_grasp, is_toss))
        gripper_thread.start()
    while True:
        rospy.sleep(rospy.Duration.from_sec(1))
        if arm_thread.join():
            if use_gripper:
                if gripper_thread.join():
                    break
            else:
                break

    # finalize
    rospy.signal_shutdown("Finished.")


if __name__ == '__main__':
    use_gripper = rospy.get_param('/pick_and_toss/use_gripper')
    ur5e_ip = rospy.get_param('/pick_and_toss/ur5e_ip')
    tool_tcp_port = rospy.get_param('/pick_and_toss/tool_tcp_port')

    try:
        pick_and_toss()
    except rospy.ROSInterruptException:
        pass
