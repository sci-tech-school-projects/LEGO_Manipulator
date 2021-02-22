#! /usr/bin/env python
import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy
import actionlib
import actionlib_tutorials.msg

class FibonacciAction(object):
    _feedback = []
    _result   = []

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                actionlib_tutorials.msg.FibonacciAction,
                                                execute_cb=self.execute_cb)
        self._as.start()

    def execute_cb(self, goal):
        r = rospy.Rate(1)
        success = True


        for i in xrange(1, goal.order):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._as.publish_feedback(self._feedback)
            r.sleep()

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('fibonacci')
    FibonacciAction(rospy.get_name())
    rospy.spin()

