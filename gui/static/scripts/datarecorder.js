{
    GazeControl.Recorder = class {
        recording = false
        writer = undefined
        fullFilename = ""

        constructor(filename = "recording") {
            this.filename = filename
        }

        /**
         * Initialises the recorder to capture a certain canvas, does not begin recording.
         *
         * @param {*} canvas
         */
        initialize(canvas) {
            this.videoRecorder = new p5.VideoRecorder(canvas);
            this.videoRecorder.onFileReady = this.saveVideo;
        }

        start(p, filename = this.filename) {
            if (!this.recording) {
                this.fullFilename = filename + "-" + Date.now()
                this.writer = p.createWriter(this.fullFilename, 'csv')
                this.videoRecorder.start()
                this.recording = true
                this.write("Beginning Recording")
            }
        }

        stop() {
            if (this.recording) {
                this.write("End Recording")
                this.writer.close()
                this.videoRecorder.stop()
                this.recording = false
            }
        }

        write(data) {
            if (this.recording) {
                this.writer.print([Date.now(), data])
            }
        }

        display(p) {
            p.textAlign(p.RIGHT, p.BOTTOM)
            // textSize(textSize/2)
            if (this.recording) {
                p.text("Recording: " + this.fullFilename, p.width, p.height)
            } else {
                p.text("Not recording (" + this.fullFilename + ")", p.width, p.height)
            }
        }

        saveVideo() {
            //  Download the recording
            recorder.videoRecorder.save(recorder.fullFilename);
        }
    }
}