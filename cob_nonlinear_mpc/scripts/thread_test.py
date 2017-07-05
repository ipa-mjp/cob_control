#!/usr/bin/python

import thread
import rospy

joint_pub = JointStatePublisher('/arm/joint_group_velocity_controller/command')
q = Float64MultiArray()

def loop():
    rate = rospy.Rate(100)  # 10hz
    joint_pub.open()
    while not rospy.is_shutdown():
        #joint_pub.publish(q)
        print("publising")
        rate.sleep()
    thread.exit_thread()

def loop2():
    rate = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        print('thread 2')
        rate.sleep()
    thread.exit_thread()

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    # Create two threads as follows
    try:
        thread.start_new_thread(loop, ())
        thread.start_new_thread(loop2, ())
    except:
        print "Error: unable to start thread"