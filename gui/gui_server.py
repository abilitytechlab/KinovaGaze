import sys
import time
import cv2
# from flask_jsglue import JSGlue
from flask import Flask, render_template, Response
from flask_socketio import SocketIO

# The -1 should find at least one webcam, if you need to use a different webcam or if it doesn't find the webcam try changing this to 0 or higher.
# cv2.CAP_OPENCV_MJPEG specifies the OpenCV MotionJPEG codec, try in case of issues try changing this to CAP_V4L2 
videoCapture = cv2.VideoCapture(-1)
videoCapture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
#print(cv2.getBuildInformation())
#input("Press Enter to continue...")
videoCapture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920/3)
videoCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080/3)

# Calling this method will create a generator which yields a frame each time __next__() is called
# The frame is encoded in a string of bytes with text around it (HTTP arguments??)
def generate_frames():
    frame_output = ""
    error_count = 0
    while True:
        returnValue, image = videoCapture.read()        # Get a video frame
        if returnValue:
            ret, buffer = cv2.imencode('.jpg', image)   # Encode image into jpg
            frame = buffer.tobytes()                    # Turn buffer into byte array
            frame_output = b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
            yield(frame_output)  
        else:
            error_count += 1
            print(f"Error reading video feed, repeating last frame, error count: {error_count}")
            yield(frame_output)

class GUIServer():
    def __init__(self):
        self.app = Flask(__name__)
        # self.jsglue = JSGlue(self.app)
        self.app.route('/')(lambda: render_template('gui.html'))
        self.app.route('/old')(self.show_gui_old)
        self.app.route
        self.app.route('/video_feed')(self.video_feed)

    def show_gui_old(self):
        return render_template('gui_old.html')

    def video_feed(self):
        return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    server = GUIServer()
    server.app.run(host="0.0.0.0", debug=False)
