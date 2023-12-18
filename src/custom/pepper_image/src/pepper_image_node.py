#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
import qi
import sys
import time

def display_image(session, url):
    url = "http://198.18.0.1/apps/studyadvisor/qr.png"
    tablet_service = session.service("ALTabletService")
    print(url)
    result = tablet_service.showImageNoCache(url)
    print("RESUlT OF SHOW IS:", result)
    time.sleep(20)
    tablet_service.hideImage()

def callback(data):
    IP = rospy.get_param('~pepper_ip', '192.168.1.101')
    PORT = rospy.get_param('~pepper_port', 9559)
    session = qi.Session()
    try:
        session.connect("tcp://" + IP + ":" + str(PORT))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + IP + "\" on port " + str(PORT) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    display_image(session, data.data)


if __name__ == '__main__':
    rospy.init_node('pepper_image_display')
    rospy.Subscriber("image_url", String, callback)
    rospy.spin()