#!/usr/bin/env python2

import rospy
from std_msgs.msg import String, Bool
import qi
import sys

def play_sound(session, path):
    audio_player_service = session.service("ALAudioPlayer")
    audio_player_service.playFile(path)


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
    sound_path = data.data
    
    # Set the flag to False when callback starts
    callback_finished = False

    # Perform the callback actions
    play_sound(session=session, path=sound_path)

    # Set the flag to True after the speech is done
    callback_finished = True

    # Publish the flag
    finished_publisher.publish(callback_finished)

def listener():
    rospy.init_node('sound_path', anonymous=True)
    rospy.Subscriber('pepper_sound', String, callback)
    rospy.spin()

if __name__ == '__main__':
    finished_publisher = rospy.Publisher('pepper_sound_state', Bool, queue_size=1)
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
