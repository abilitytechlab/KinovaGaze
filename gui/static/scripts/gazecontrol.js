GazeControl = {};
{
    /** Cursor for GazeControl, activates hovered buttons */
    GazeControl.Cursor = class {
        enabled = true
        buttons = [1, 2, 3]

        /**
         * @constructor
         * @param {object} p - p5.js instance
         * @param {object} buttons - Variable containing all buttons to keep track of
         * @param {object} canvas - Canvas object to use
         */
        constructor(p, buttons, canvas) {
            this.p = p
            this.buttons = buttons
            this.p.noCursor() // Hide normal cursor so it can be optionally replaced with a custom cursor
            this.x = -100 // Start cursor away from the visible canvas and buttons
            this.y = -100

            canvas.mouseOut(this.disable.bind(this)) // Register callbacks for mouse cursor entering or leaving the canvas
            canvas.mouseOver(this.enable.bind(this))
        }

        /**
         * Displays cursor location. Call every frame in draw loop to enable.
         */
        display() {
            if(this.enabled) {
                this.p.fill(255)
                this.p.circle(this.x, this.y, 20)
                this.p.fill(0)
                this.p.circle(this.x, this.y, 2)
            }
            
        }

        /**
         * Callback for when cursor moves. Call from p5.js cursorMoved event.
         * 
         * @param {number} x - x position of cursor relative to canvas
         * @param {number} y - y position of cursor relative to canvas
         */
        cursorMoved(x, y) {
            this.x = x
            this.y = y

            this.update()

            if(typeof recorder !== 'undefined') {
                recorder.write(["Cursor Moved", this.x, this.y])
            }
        }

        /**
         * Update buttons. Called on every cursor state change.
         */
        update() {
            if(this.enabled) {
                for (const button of this.buttons) {
                    if (this.x > button.x
                        && this.x < button.x + button.w
                        && this.y > button.y
                        && this.y < button.y + button.h) {
                        if (button.dwelled == false) {
                            button.hover()
                        }
                    } else if (button.dwelled == true) {
                        button.unHover()
                    }
                }
            } else {
                for (const button of this.buttons) {
                    button.unHover()
                }
            }
        }

        /**
         * Disable mouse cursor. Called per default when cursor exits canvas.
         */
        disable() {
            this.enabled = false

            this.update()

            if(typeof recorder !== 'undefined') {
                recorder.write(["Cursor Disabled"])
            }
        }

        /**
         * Enable mouse cursor. Called per default when cursor enters canvas.
         */
        enable() {
            this.enabled = true

            this.update()

            if(typeof recorder !== 'undefined') {
                recorder.write(["Cursor Enabled"])
            }
        }
    }

    /** Abstract button which can be activated by hovering the cursor over it. Intended to be extended with custom behaviour.
     * A button has 4 state booleans:
     * - Visible: true if button should be drawn and active, false if button should be invisible and inactive
     * - Enabled: true if button should be accepting inputs, false if button should not accept intputs and be semi-translucent.
     * - Dwelled: true if button is currently hovered by cursor.
     * - Active: true if button is currently active.
     * A final state variable is dwellProgress, which keeps track of the current dwell time in ms. Can only be >0 if button is visible and enabled.
     */
    GazeControl.Button = class {
        dwellProgress = 0.0
        visible = true
        enabled = true
        dwelled = false
        active = false

        /**
         * @constructor
         * @param {object} p - p5.js instance
         * @param {number} x - x  position of top left corder
         * @param {number} y - y position of top left corner
         * @param {number} w - Width in pixels
         * @param {number} h - Height in pixels
         * @param {string} name - Name of button, should be unique
         * @param {*} label - (optional) Label to display on button, either a string or p5.js image object. Defaults to button name
         * @param {number} dwellDelay (optional) Delay in ms after which the button will activate when hovered. Defaults to 1000
         * @param {object} defaultColor (optional) p5.js color of button when not interacted with. Defaults to a red color.
         * @param {object} hoverColor (optional) p5.js color to fade to when button is being hovered. Defaults to a yellow color.
         * @param {object} activatedColor (optional) p5.js color of button when activated. Defaults to a green color.
         */
        constructor(p, x, y, w, h, name, label = undefined, dwellDelay = 1000, defaultColor = p.color(200, 50, 0), hoverColor = p.color(200, 200, 50), activatedColor = p.color(0, 200, 50)) {
            this.p = p
            this.x = x
            this.y = y
            this.w = w
            this.h = h
            this.name = name
            if(label == undefined) {
                label = name
            }
            this.label = label
            this.dwellDelay = dwellDelay
            this.currentColor = defaultColor
            this.defaultColor = defaultColor
            this.hoverColor = hoverColor
            this.activatedColor = activatedColor
        }

        /**
         * Update button state. Call every frame.
         * 
         * @param {number} dt - Time in ms since last update
         */
        update(dt) {
            if (this.dwelled == true && this.active == false && this.enabled == true && this.visible == true) {
                if (this.dwellProgress >= this.dwellDelay) {
                    this.activate()
                } else {
                    this.dwellProgress += dt
                    // print(dt + " " + this.dwellProgress + " " + this.dwellDelay)
                    this.currentColor = this.p.lerpColor(this.defaultColor, this.hoverColor, this.dwellProgress / this.dwellDelay)
                }
            } else if(this.active == true) {
                this.currentColor = this.p.lerpColor(this.defaultColor, this.activatedColor, this.dwellProgress / this.dwellDelay)
            }
            this.onUpdate()
        }

        /**
         * Display button. Call every frame to enable.
         */
        display() {
            if(this.visible) {
                let fillColor = this.currentColor
                if(!this.enabled) {
                    fillColor = this.p.lerpColor(fillColor, this.p.color(64, 64, 64, 64), 0.5)
                }
                this.p.fill(fillColor)
                this.p.rectMode(this.p.CORNER)
                this.p.rect(this.x, this.y, this.w, this.h)
                this.p.fill(0)

                if(typeof this.label == "string") {
                    this.p.textAlign(this.p.CENTER, this.p.CENTER)
                    this.p.text(this.label, this.x, this.y, this.w, this.h)
                } 
                else if(this.label instanceof p5.Image) {
                    this.p.imageMode(this.p.CENTER);
                    let scale
                    if(this.label.width/this.label.height > this.w/this.h) {
                        scale = this.w/this.label.width
                    } else {
                        scale = this.h/this.label.height
                    }
                    let labelWidth = this.label.width*scale
                    let labelHeight = this.label.height*scale
                    this.p.image(this.label, this.x+this.w/2, this.y+this.h/2, labelWidth/2, labelHeight/2)
                }
            }
        }

        /**
         * Called every update. Overwrite with custom behaviour.
         */
        onUpdate() {

        }

        /**
         * Tells button it is being hovered. Usully called by cursor.
         */
        hover() {
            if(this.enabled == true && this.visible == true) {
                this.dwelled = true
                this.onHover()
                if(typeof recorder !== 'undefined') {
                    recorder.write(["Button Hovered", this.name])
                }
            }
        }

        /**
         * Called when button begins being hovered. Overwrite with custom behaviour.
         */
        onHover() {

        }

        /**
         * Tells button it is no longer hovered. Usully called by cursor.
         */
        unHover() {
            this.dwelled = false
            if (this.active == false) {
                this.reset()
            }
            this.onUnhover()
            if(typeof recorder !== 'undefined') {
                recorder.write(["Button Unhovered", this.name])
            }
        }

        /**
         * Called when button begins being hovered. Overwrite with custom behaviour.
         */
        onUnhover() {

        }

        /**
         * Activates button. Called when dwellProgress >= dwellDelay. Opposite of reset()
         */
        activate() {
            this.currentColor = this.activatedColor
            this.active = true
            this.onActivate()
            if(typeof recorder !== 'undefined') {
                recorder.write(["Button Activated", this.name])
            }
        }

        /**
         * Called when button is activated. Overwrite with custom behaviour.
         */
        onActivate() {

        }

        /**
         * Resets / deactivates button. Opposite of activate()
         */
        reset() {
            this.currentColor = this.defaultColor
            this.active = false
            this.dwellProgress = 0
            this.onReset()
        }

        /**
         * Called when button is reset. Overwrite with custom behaviour.
         */
        onReset() {
            
        }

        /**
         * Disables button. Sets dwell progress to 0 and prevents interactions. Button becomes semi-translucent. Opposite of enable()
         */
        disable() {
            this.enabled = false
            this.dwellProgress = 0
        }

        /**
         * Enables button. Button accepts interactions again. Button becomes fully opaque. Opposite of disable()
         */
        enable() {
            this.enabled = true
        }

        /**
         * Hides button. Sets dwell progress to 0 and prevents interactions. Button does not get rendered anymore. Opposite of unhide()
         */
        hide() {
            this.visible = false
            this.dwellProgress = 0
        }

        /**
         * Unhides button. Button accepts interactions again. Button gets rendered again. Opposite of hide()
         */
        unhide() {
            this.visible = true
        }

        /**
         * Overwrites string representation of button with its name.
         * 
         * @returns Button name
         */
        toString() {
            return this.name
        }
    }

    /** Class for quickly setting up a grid of objects.
     * Set grid location and size for object using set(), then read x, y, w, h for object x, y, width and height coordinates.
    */
    GazeControl.Grid = class {
        x = undefined
        y = undefined
        w = undefined
        h = undefined

        /**
         * Creates a grid spanning an area with a number of columns and rows
         * @constructor
         * @param {number} x - x position of upper left corner
         * @param {number} y - y position of upper left corner
         * @param {number} w - Width of grid
         * @param {number} h - Height of grid
         * @param {number} columns - Number of columns in grid
         * @param {number} rows - Number of rows in grid
         */
        constructor(x, y, w, h, columns, rows) {
            this.areaX = x,
            this.areaY = y,
            this.areaW = w,
            this.areaH = h,
            this.columns = columns,
            this.rows = rows
            this.set(0, 0)
        }

        /**
         * Set the grid location and size of new object
         * @param {number} column - Left column of object
         * @param {number} row - Upper row of object
         * @param {number} width - Object width in columns
         * @param {number} height - Object height in columns
         */
        set(column, row, width = 1, height = 1) {
            this.x = this.areaX+this.areaW/this.columns*column
            this.y = this.areaY+this.areaH/this.rows*row
            this.w = (this.areaW/this.columns)*width
            this.h = (this.areaH/this.rows)*height
        }
    }
}