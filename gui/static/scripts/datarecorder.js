{
    /** Recorder of p5.js canvas with accompanying .csv log file */
    GazeControl.Recorder = class {
        recording = false
        writer = undefined
        fullFilename = ""

        /**
         * @constructor
         * @param {string} [filename="recording"] - (optional) Filename of recording, defaults to "recording"
         */
        constructor(filename = "recording") {
            this.filename = filename
        }

        /**
         * Initialises the recorder to capture a certain canvas, does not begin recording.
         *
         * @param {object} canvas - Canvas to record
         */
        initialize(canvas) {
            this.videoRecorder = new p5.VideoRecorder(canvas);
            this.videoRecorder.onFileReady = this.saveVideo;
        }

        /**
         * Start recording
         *
         * @param {object} p - p5.js instance
         * @param {string} [filename=this.filename] - (optional) Filename of output, defaults to name set in constructor or "recording"
         */
        start(p, filename = this.filename) {
            if (!this.recording) {
                this.fullFilename = filename + "-" + Date.now()
                this.writer = p.createWriter(this.fullFilename, 'csv')
                this.videoRecorder.start()
                this.recording = true
                this.write("Beginning Recording")
            }
        }

        /**
         * Stops recording and downloads output files
         */
        stop() {
            if (this.recording) {
                this.write("End Recording")
                this.writer.close()
                this.videoRecorder.stop()
                this.recording = false
            }
        }

        /**
         * Write output to log, accepts either a primite variabe (including string) or an array
         *
         * @param {*} data - data to write, either a primite variabe (including string) or an array
         */
        write(data) {
            if (this.recording) {
                this.writer.print([Date.now(), data])
            }
        }

        /**
         * Display a simple status text with the name of current or last filename and whether the recorder is currently recording
         *
         * @param {object} p - p5.js instance
         */
        display(p) {
            p.textAlign(p.RIGHT, p.BOTTOM)
            // textSize(textSize/2)
            if (this.recording) {
                p.text("Recording: " + this.fullFilename, p.width, p.height)
            } else {
                p.text("Not recording (" + this.fullFilename + ")", p.width, p.height)
            }
        }

        /**
         * Callback for when video has finished processing, downloads result
         */
        saveVideo() {
            //  Download the recording
            recorder.videoRecorder.save(recorder.fullFilename);
        }
    }
}