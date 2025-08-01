#!/usr/bin/env python
import numpy as np
import roslib
import rospy
import tf
import actionlib
from sparrow_planner.msg import sparrowAction
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PoseStamped
import utils
from sparrow_planner.msg import grinding_path
class SparrowActionServer(object):
    def __init__(self):
        self.server = actionlib.SimpleActionServer('sparrow_action', sparrowAction, self.execute, False)
        self.server.start()
        self.path_pub = rospy.Publisher('/move_base/SparrowPlanner/grinding_path', grinding_path, queue_size=10, latch=True)
	rospy.Subscriber('/boundary', Polygon,self.boundaryCb)

    def makePose(self, point):
        print "construct pose from point", point
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = 0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

    def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
        self.server.set_succeeded()
    def boundaryCb(self, boundary):
        init_point = [.0,.0]
        points = np.array(map(lambda p: [p.x, p.y], boundary.points))
        print "The boundary is :", points
        points = utils.grinding_path(init_point, points)
        path = map(lambda p:self.makePose(p), points)
        print path
        msg = grinding_path()
        msg.spirial = path
        print 'begin----'
        self.path_pub.publish(msg)
        print 'end----'
        

if __name__ == '__main__':
    rospy.init_node('sparrow_action')
    server = SparrowActionServer()
    rospy.spin()
