"""GUI Server

Creates a flask HTTP server which hosts the web interfaces, relevant files and camema feed.
The server opens on all available IP addresses.
"""


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
    """On every iteration, generates a video frame encoded as jpeg and packaged into a HTTP packet.

    Every iteration a video frame is read from the videoCapture, encoded into a string and passed encoded as a HTTP packet.
    If reading the feed results in an error, the stream is closed and the method returns.
    If reading the feed results in None, the method will yield the previous valid frame. 
    If the videoCapture returns None 100 times in a row, the stream is closed and the method returns.
    
    Parameters
    ----------
    videoCapture : cv2.VideoCapture
        The cv2 video capture object to use

    Yields
    ------
    String
        A video framed encoded as jpeg and packaged into
    """

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
                print("Video feed did not recover in time, closing stream.")
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
        self.app.route('/start-stop')(self.show_gui_start_stop)
        self.app.route('/research_start-stop')(self.show_gui_start_stop_research)
        self.app.route('/hold')(self.show_gui_hold)
        self.app.route('/research_hold')(self.show_gui_hold_research)
        self.app.route('/whack-a-button')(self.show_whack_a_button)
        self.app.route('/research_whack-a-button')(self.show_whack_a_button_research)
        self.app.route('/video_feed')(self.video_feed)

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
        """Starts an instance of generate_frames with a new cv2.VideoCaputre.

        Opens a new cv2.VideoCaputre and passes it to a new instance of generate_frames.
        Returns a HTTP package with generate_frames as well as a type indicator.
        Returns an existing videoCapture instance if available.
        Returns 503 if the videoCapture could not be opened
        Returns if the envcironment is a flask debug thread to prevent opening the camera twice when debug is enabled.

        Returns
        -------
        tuple of (dict, int)
            HTTP packet containing either video frame generator or error information
        """
        # The video feed will not run if the current environment is the debugging thread to avoid opening the video capture twice
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
    