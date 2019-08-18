#!/usr/bin/env python
import numpy as np
from collections import deque
import roslib
import rospy
import tf
from derobotee.msg import Elevation
from derobotee.msg import MotorStatusList
from derobotee.msg import MotorCmdList
from derobotee.msg import MotorCmd
from std_msgs.msg import Int32
class GrinderControl():
    ELEVATION_RECEIVED = 1
    ELEVATION_NOT_RECEIVED = 0
    FIND_HOME = 0x8000
    ELEVATION_NO_SIG_DURATION = 1
    Z_OFFSET = 8000

    def __init__(self):
#        self.z_motor_pub = rospy.Publisher('/move_base/SparrowPlanner/grinding_path', grinding_path, queue_size=10, latch=True)
#        self.grinding_motor_pub = rospy.Publisher('/move_base/SparrowPlanner/grinding_path', grinding_path, queue_size=10, latch=True)
#        self.base_location_pub = rospy.Publisher('/move_base/SparrowPlanner/grinding_path', grinding_path, queue_size=10, latch=True)
        self.z_motor_state = None
	self.elevation_state = None
	self.elevation_time = None
	self.motor_offset = 0
        self.working = False
        '''
        grinding base elevation value, will be set by ros topic from cmd
        '''
        self.base = 96
#	self.zero_point = 128000
	#self.zero_point = 93000
	self.zero_point = 70000
	self.elevations = deque(maxlen=10)
	self.high_freq_ele = None
	self.should_grinding = False
	self.base_show_rate = 0.0
	self.elevation_avg = None
        rospy.init_node('grinder_control')
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
	rospy.Subscriber('/derobotee/elevation', Elevation,self.receive_elevation)
#	rospy.Subscriber('/derobotee/elevation_base', :,self.receive_elevation)
	rospy.Subscriber('/motor/state', MotorStatusList,self.receive_motor_state)
	rospy.Subscriber('/motor/zero_point_offset', Int32 ,self.receive_motor_offset)
	rospy.Subscriber('/derobotee/working', Int32 ,self.working_cb)
        self.z_motor_pub = rospy.Publisher('/derobotee/cmd', MotorCmdList, queue_size=10)
    def working_cb(self, working):
        print "Received working value is ", working
        if working.data == 1:
            self.working = True
        else:
            self.working = False
        print "working value is ", self.working
    def receive_motor_offset(self, offset):
        self.motor_offset = offset.data
        print "motor offset is", self.motor_offset
    def receive_motor_state(self, states):
    #    print states.list
        self.z_motor_state = states.list[0]
    def receive_elevation(self, elevation):
        self.elevations.append(elevation.data)
	self.elevation_time = rospy.get_time()
        self.analysis_elevation_data()
#        print self.elevation
    def analysis_elevation_data(self):
	count = 0
	values = {}
        for e in self.elevations:
	    if values.has_key(e):
	        values[e] = values[e] + 1
            else:
	        values[e] = 1
	if values.has_key(self.base):
	    self.base_show_rate = 1.0*values[self.base]/len(self.elevations)
        self.elevation_avg = np.average(self.elevations)
    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
    def update(self):
        grind_cmd = MotorCmd()
        grind_cmd.slave = 2
        grind_cmd.value = 2000
	"""
	check motor state
	"""
        if not self.working:
            grind_cmd.value = 0
        if self.z_motor_state is None : return
        if (not self.z_motor_state.statusWord & self.FIND_HOME): return
	"""
	check elevation sensor state
	"""
	now = rospy.get_time()
	if (self.elevation_time is not None) and (now - self.elevation_time) < self.ELEVATION_NO_SIG_DURATION:
	    self.elevation_state = self.ELEVATION_RECEIVED
	else:
	    self.elevation_state = self.ELEVATION_NOT_RECEIVED
	    self.elevations.clear()
	"""
	check motor state
	"""
	cmd = MotorCmd()
	cmd.slave = 1
	target_position = self.zero_point 
	if self.elevation_state == self.ELEVATION_NOT_RECEIVED:
	    print 'elevation sensor has no data'
	    cmd.value = self.zero_point 
	else:
	    print 'elevation sensor has data', self.elevation_avg
	    print self.base_show_rate
	    if self.base_show_rate > 0.2: 
		print self.elevation_avg, self.base_show_rate
	        if self.elevation_avg >= self.base :
		    target_position = self.zero_point 
		elif self.elevation_avg < 0:
		    target_position = self.zero_point + self.Z_OFFSET + self.motor_offset
            elif self.elevation_avg < 0:
                target_position = self.zero_point + self.Z_OFFSET + self.motor_offset
	cmd.value = target_position
        if not self.working:
            cmd.value = 0
	cmds = MotorCmdList()
        cmds.list.append(cmd)
        cmds.list.append(grind_cmd)
	print cmds.list
        self.z_motor_pub.publish(cmds)
if __name__ == '__main__':
    rospy.init_node('grinder_control')
    try:
        ne = GrinderControl()
        ne.spin()
    except rospy.ROSInterruptException: pass
