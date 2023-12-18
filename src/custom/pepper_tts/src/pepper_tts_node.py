#!/usr/bin/env python2

import rospy
from std_msgs.msg import String, Bool
from naoqi import ALProxy

def callback(data):
    IP = rospy.get_param('~pepper_ip', '192.168.1.101')
    PORT = rospy.get_param('~pepper_port', 9559)
    LANGUEGE = rospy.get_param('~language', 'German')
    SPEED = rospy.get_param('~voice_speed', 400)
    PITCH = rospy.get_param('~voice_pitch', 1.1)
    tts = ALProxy("ALTextToSpeech", IP, PORT)
    tts.setParameter("speed", SPEED)
    tts.setParameter("pitchShift", PITCH)
    tts.setLanguage(LANGUEGE)

    # Set the flag to False when callback starts
    callback_finished = False

    # Perform the callback actions
    tts.say(data.data)

    # Set the flag to True after the speech is done
    callback_finished = True

    # Publish the flag
    finished_publisher.publish(callback_finished)

def listener():
    rospy.init_node('pepper_tts_node', anonymous=False)
    rospy.Subscriber('pepper_tts', String, callback)
    rospy.spin()

if __name__ == '__main__':
    finished_publisher = rospy.Publisher('pepper_tts_state', Bool, queue_size=1)
    listener()

