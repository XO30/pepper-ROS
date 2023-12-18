from naoqi import ALProxy
import time
import uuid

class PepperFaceDetection:
    def __init__(self, pepper_ip, pepper_port):
        self.ip = pepper_ip
        self.port = pepper_port
        self.face_detection = ALProxy("ALFaceDetection", self.ip, self.port)
        self.memory = ALProxy("ALMemory", self.ip, self.port)
        self.face_detection.subscribe("FaceRecognitionApp")
    
    def detect(self):
        detected = None
        while not detected:
            time.sleep(2)
            val = self.memory.getData("FaceDetected")
            if val:
                face_info = val[1]
                if face_info:
                    recognized_face = face_info[0][1]
                    face_label = recognized_face[2]
                    if face_label:
                        detected = face_label
                    else:
                        self.learn()
        return detected

    def learn(self):
        new_label = str(uuid.uuid4())
        self.face_detection.learnFace(new_label)
        return new_label
