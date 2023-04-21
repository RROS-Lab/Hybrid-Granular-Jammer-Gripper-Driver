#!/usr/bin/python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult
from serial import SerialException
from time import sleep
import multiprocessing
import sys
from std_msgs.msg import Int16
from rosserial_python import SerialClient, RosSerialServer
from serial import SerialException

class GripperActionServer(object):

    # Action Server /result
    _result   = FollowJointTrajectoryAction()
    def __init__(self):
        self.action_server = actionlib.SimpleActionServer('gripper_controller/gripper_action', FollowJointTrajectoryAction, self.gripper_action_execute, False)
        self.action_server.start()
        self.arduino_publisher = rospy.Publisher('/Robotiq_state_arduino', Int16, queue_size = 10)

    def gripper_action_execute(self, goal):
        rospy.loginfo('goal action')
        result = FollowJointTrajectoryActionResult()
        self.action_server.set_succeeded()
        result.status.status = 3
        self.pos_arduino = int(goal.trajectory.points[0].positions[0]*100)
        self.arduino_publisher.publish(gripper_action_server.pos_arduino)
    
    def arduino_interface(self):
        # Port - Write the port that Arduino is connected to
        port_name = rospy.get_param('~port', '/dev/ttyACM0')
        baud = int(rospy.get_param('~baud', '57600'))
        # for systems where pyserial yields errors in the fcntl.ioctl(self.fd, TIOCMBIS, \
        # TIOCM_DTR_str) line, which causes an IOError, when using simulated port
        fix_pyserial_for_test = rospy.get_param('~fix_pyserial_for_test', False)

        # Allows for assigning local parameters for tcp_port and fork_server with
        # global parameters as fallback to prevent breaking changes 
        if(rospy.has_param('~tcp_port')):
            tcp_portnum = int(rospy.get_param('~tcp_port'))
        else:
            tcp_portnum = int(rospy.get_param('/rosserial_embeddedlinux/tcp_port', '11411'))

        if(rospy.has_param('~fork_server')):
            fork_server = rospy.get_param('~fork_server')
        else:
            fork_server = rospy.get_param('/rosserial_embeddedlinux/fork_server', False)

        # TODO: do we really want command line params in addition to parameter server params?
        sys.argv = rospy.myargv(argv=sys.argv)
        if len(sys.argv) >= 2 :
            port_name  = sys.argv[1]
        if len(sys.argv) == 3 :
            tcp_portnum = int(sys.argv[2])

        if port_name == "tcp" :
            server = RosSerialServer(tcp_portnum, fork_server)
            rospy.loginfo("Waiting for socket connections on port %d" % tcp_portnum)
            try:
                server.listen()
            except KeyboardInterrupt:
                rospy.loginfo("got keyboard interrupt")
            finally:
                rospy.loginfo("Shutting down")
                for process in multiprocessing.active_children():
                    rospy.loginfo("Shutting down process %r", process)
                    process.terminate()
                    process.join()
                rospy.loginfo("All done")

        else :          # Use serial port
            while not rospy.is_shutdown():
                rospy.loginfo("Connecting to %s at %d baud" % (port_name,baud) )
                try:
                    client = SerialClient(port_name, baud, fix_pyserial_for_test=fix_pyserial_for_test)
                    client.run()
                except KeyboardInterrupt:
                    break
                except SerialException:
                    sleep(1.0)
                    continue
                except OSError:
                    sleep(1.0)

# Main Program
if __name__ == "__main__":
    rospy.init_node('robotiqGripper')
    rospy.loginfo("Started")
    gripper_action_server = GripperActionServer()
    gripper_action_server.arduino_interface()
    