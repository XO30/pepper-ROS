#!/usr/bin/env python2
import rospy
from std_msgs.msg import Bool, String
from pepper_face_detection import PepperFaceDetection

def detection_callback(msg):
    if msg.data: 
        detected_face_id = fd.detect()
        face_id_pub.publish(detected_face_id)

if __name__ == '__main__':
    rospy.init_node('face_detection_node')
    IP = rospy.get_param('~pepper_ip', '192.168.1.101')
    PORT = rospy.get_param('~pepper_port', 9559)
    fd = PepperFaceDetection(IP, PORT)
    face_id_pub = rospy.Publisher('pepper_face_detection_id', String, queue_size=1)
    rospy.Subscriber('pepper_face_detection_state', Bool, detection_callback)
    rospy.spin()