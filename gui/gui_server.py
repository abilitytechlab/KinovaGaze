import sys
import time
import cv2
import os
# from flask_jsglue import JSGlue
from flask import Flask, render_template, Response
from flask_socketio import SocketIO

# Calling this method will create a generator which yields a frame each time __next__() is called
# The frame is encoded in a string of bytes with text around it (HTTP arguments??)
def generate_frames(videoCapture):
    frame_output = ""
    error_count = 0
    while True:
        returnValue, image = None, None
        try:
            returnValue, image = videoCapture.read()        # Get a video frame
        except Exception:
            print("Fatal error reading video feed, closing stream.")
            videoCapture.release()
            return
        if returnValue is not None:
            if(error_count > 0):
                print("Video feed recovered, ")
            ret, buffer = cv2.imencode('.jpg', image)   # Encode image into jpg
            frame = buffer.tobytes()                    # Turn buffer into byte array
            frame_output = b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
            yield(frame_output) 
        else:
            if(error_count == 0):
                print("Error reading video feed, attempting to recover.")
            elif(error_count >= 100):
                print("Video feed did not recover, closing stream.")
                videoCapture.release()
                return
            error_count += 1
            
            yield(frame_output)

class GUIServer():
    videoCapture = None

    def __init__(self):
        print("Hello World!")
        self.app = Flask(__name__)
        # self.jsglue = JSGlue(self.app)
        self.app.route('/')(self.show_gui_start_stop)
        self.app.route('/old')(self.show_gui_old)
        self.app.route('/start-stop')(self.show_gui_start_stop)
        self.app.route('/research_start-stop')(self.show_gui_start_stop_research)
        self.app.route('/hold')(self.show_gui_hold)
        self.app.route('/research_hold')(self.show_gui_hold_research)
        self.app.route('/whack-a-button')(self.show_whack_a_button)
        self.app.route('/research_whack-a-button')(self.show_whack_a_button_research)
        
        self.app.route
        self.app.route('/video_feed')(self.video_feed)

    def show_gui_old(self):
        return render_template('gui_old.html')

    def show_gui_start_stop(self):
        return render_template('gui_start-stop.html')
    
    def show_gui_start_stop_research(self):
        return render_template('gui_start-stop_research.html')

    def show_gui_hold(self):
        return render_template('gui_hold.html')
    
    def show_gui_hold_research(self):
        return render_template('gui_hold_research.html')

    def show_whack_a_button(self):
        return render_template('whack-a-button.html')
    
    def show_whack_a_button_research(self):
        return render_template('whack-a-button_research.html')

    def video_feed(self):
        if not (os.environ.get('WERKZEUG_RUN_MAIN') or not os.environ.get("FLASK_ENV") == "development" or Flask.debug is False):
            return
        if(self.videoCapture is None or not self.videoCapture.isOpened()):
            # The -1 should find at least one webcam, if you need to use a different webcam or if it doesn't find the webcam try changing this to 0 or higher.
            # If your camera doesn't work, you might need to add the argument cv2.CAP_V4L2 or cv2.CAP_OPENCV_MJPEG and remove the first videoCapture.set() line.
            videoCapture = cv2.VideoCapture(-1)
            if(videoCapture is None and not videoCapture.isOpened()):
                return {"Retry-After": "10", "Cache-Control": "no-cache"}, 503
            
            # If a video encoder is specified in the cv2.VideoCapture initialisation, remove the next line.
            videoCapture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            #print(cv2.getBuildInformation())
            #input("Press Enter to continue...")
            videoCapture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920/3)
            videoCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080/3)
            self.videoCapture = videoCapture

        return Response(generate_frames(self.videoCapture), mimetype='multipart/x-mixed-replace; boundary=frame')
            


if __name__ == '__main__':
    server = GUIServer()
    server.app.run(host="0.0.0.0", debug=True)
