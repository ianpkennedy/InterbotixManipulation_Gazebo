#!/usr/bin/env python

"""
Test for the step service. Implementing one "pass" and one "fail" for motion planning /px100/step service calls
"""
import rospy
import unittest 
from motion_plan.srv import Step, StepResponse

class StepTesting(unittest.TestCase):
    def __init__(self,*args):
        super(StepTesting, self).__init__(*args)


    def test_crash(self):
        rospy.wait_for_service("/px100/step")
        step=rospy.ServiceProxy("/px100/step", Step)
        result = step(0.2,0.0,-0.3,0)
        self.assertEquals(result.code,1)

    def test_success(self):
        rospy.wait_for_service("/px100/step")
        step=rospy.ServiceProxy("/px100/step", Step)
        result = step(0.2,0.0,0.2,0)
        self.assertEquals(result.code,1)

if __name__=="__main__":
    import rostest
    rostest.rosrun('motion_plan','step_testing',StepTesting)