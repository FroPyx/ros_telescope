#!/usr/bin/env python

import rospy
from rosserial_arduino import SerialClient
from serial import SerialException
from time import sleep
from std_msgs.msg import Bool

import sys

if __name__=="__main__":

    rospy.init_node("serial_node")

    port_name = rospy.get_param('~port','/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud','115200'))

    # Number of seconds of sync failure after which Arduino is auto-reset.
    # 0 = no timeout, auto-reset disabled
    auto_reset_timeout = int(rospy.get_param('~auto_reset_timeout','0'))

    # for systems where pyserial yields errors in the fcntl.ioctl(self.fd, TIOCMBIS, \
    # TIOCM_DTR_str) line, which causes an IOError, when using simulated port
    fix_pyserial_for_test = rospy.get_param('~fix_pyserial_for_test', False)

    # TODO: do we really want command line params in addition to parameter server params?
    sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 2 :
        port_name  = sys.argv[1]

    # Define the motors_connexion_status_publisher
    motors_connexion_status_publisher = rospy.Publisher('motors_connection_status', Bool, latch=True, queue_size=1)
    motors_connexion_status = Bool(False)
    motors_connexion_status_publisher.publish(motors_connexion_status)

    while not rospy.is_shutdown():
        rospy.loginfo("Connecting to %s at %d baud" % (port_name, baud))
        try:
            client = SerialClient(port_name, baud, fix_pyserial_for_test=fix_pyserial_for_test, auto_reset_timeout=auto_reset_timeout)
            # Send connection OK status
            motors_connexion_status.data = True
            motors_connexion_status_publisher.publish(motors_connexion_status)
            client.run()
        except KeyboardInterrupt:
            # Send connection OK status
            motors_connexion_status.data = False
            motors_connexion_status_publisher.publish(motors_connexion_status)
            break
        except SerialException:
            # Send connection OK status
            motors_connexion_status.data = False
            motors_connexion_status_publisher.publish(motors_connexion_status)
            sleep(1.0)
            continue
        except OSError:
            # Send connection OK status
            motors_connexion_status.data = False
            motors_connexion_status_publisher.publish(motors_connexion_status)
            sleep(1.0)
            continue
        except:
            # Send connection OK status
            motors_connexion_status.data = False
            motors_connexion_status_publisher.publish(motors_connexion_status)
            sleep(1.0)
            continue

