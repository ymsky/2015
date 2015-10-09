#!/usr/bin/env python

import rospy
import urllib2
import time
from std_msgs.msg import UInt16

def callback(data):
	ts=time.time()
	ts1=ts*10000
	
	print data.data

	ipaddress_last=38
	command_index=data.data
	string='http://192.168.1.%d:81/decoder_control.cgi?loginuse=admin&loginpas=12345&command=%d&onestep=0&%d.3093554663937539' 	% (ipaddress_last,command_index,ts1)
	response = urllib2.urlopen(string)
	html = response.read()
	
	print html
	print string


def test():
	rospy.init_node("urlExcecute", anonymous=True)
	rospy.loginfo('urlExcecte.py is running')
	
	rospy.Subscriber("command_request", UInt16, callback)
	rospy.spin()
	
	
if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass
