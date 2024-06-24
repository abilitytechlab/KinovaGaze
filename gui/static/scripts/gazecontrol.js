GazeControl = {};
{
    GazeControl.Cursor = class {
        enabled = true
        buttons = [1, 2, 3]

        constructor(p, buttons) {
            this.p = p
            this.buttons = buttons
            this.p.noCursor()
            this.x = -100
            this.y = -100
        }

        display() {
            this.p.fill(255)
            this.p.circle(this.x, this.y, 20)
            this.p.fill(0)
            this.p.circle(this.x, this.y, 2)
        }

        cursorMoved(x, y) {
            this.x = x
            this.y = y
            this.update()
            if(typeof recorder !== 'undefined') {
                recorder.write(["Cursor Moved", this.x, this.y])
            }
        }

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
            }
        }

        disable() {
            this.enabled = false
            for (const button of this.buttons) {
                button.unHover()
            }
            if(typeof recorder !== 'undefined') {
                recorder.write(["Cursor Disabled"])
            }
        }

        enable() {
            this.enabled = true
            if(typeof recorder !== 'undefined') {
                recorder.write(["Cursor Enabled"])
            }
        }
    }

    GazeControl.Button = class {
        dwellProgress = 0.0
        visible = true
        enabled = true
        dwelled = false
        active = false
        
        constructor(p, x, y, w, h, name = "", label = undefined, dwellDelay = 1000, defaultColor = p.color(200, 50, 0), hoverColor = p.color(200, 200, 50), activatedColor = p.color(0, 200, 50)) {
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
                    this.p.rectMode(this.p.CENTER);
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

        onUpdate() {

        }

        hover() {
            if(this.enabled == true && this.visible == true) {
                this.dwelled = true
                this.onHover()
                if(typeof recorder !== 'undefined') {
                    recorder.write(["Button Hovered", this.name])
                }
            }
        }

        onHover() {

        }

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

        onUnhover() {

        }

        activate() {
            this.currentColor = this.activatedColor
            this.active = true
            this.onActivate()
            if(typeof recorder !== 'undefined') {
                recorder.write(["Button Activated", this.name])
            }
        }

        onActivate() {

        }

        reset() {
            this.currentColor = this.defaultColor
            this.active = false
            this.dwellProgress = 0
            this.onReset()
        }

        onReset() {
            
        }

        disable() {
            this.enabled = false
            this.dwellProgress = 0
        }

        enable() {
            this.enabled = true
        }

        hide() {
            this.visible = false
            this.dwellProgress = 0
        }

        unhide() {
            this.visible = true
        }

        toString() {
            return this.name
        }
    }

    GazeControl.Grid = class {
        x = undefined
        y = undefined
        w = undefined
        h = undefined

        constructor(x, y, w, h, columns, rows) {
            this.areaX = x,
            this.areaY = y,
            this.areaW = w,
            this.areaH = h,
            this.columns = columns,
            this.rows = rows
            this.set(0, 0)
        }

        set(column, row, width = 1, height = 1) {
            this.x = this.areaX+this.areaW/this.columns*column
            this.y = this.areaY+this.areaH/this.rows*row
            this.w = (this.areaW/this.columns)*width
            this.h = (this.areaH/this.rows)*height
        }

    //     x(column = undefined, row = undefined) {
    //         if(column == undefined) {
    //             if(this.column == undefined) {
    //                 throw new Error("Column undefined, pass as argument or call set(column, row) beforehand!")
    //             }
    //             column = this.column
    //         }
    //         return this.x+this.width/this.columns*column
    //     }

    //     w(column = undefined, row = undefined) {
    //         if(column == undefined) {
    //             if(this.column == undefined) {
    //                 throw new Error("Column undefined, pass as argument or call set(column, row) beforehand!")
    //             }
    //             column = this.column + 1
    //         } else {
    //             column = column + 1
    //         }
    //         return this.x+this.width/this.columns*column
    //     }

    //     y(row) {
    //         return this.y(undefined, row)
    //     }

    //     y(column = undefined, row = undefined) {
    //         if(row == undefined) {
    //             if(this.row == undefined) {
    //                 throw new Error("Row undefined, pass as argument or call set(column, row) beforehand!")
    //             }
    //             row = this.row
    //         }
    //         return this.y+this.height/this.rows*row
    //     }

    //     h(row) {
    //         return this.w(undefined, row)
    //     }

    //     h(column = undefined, row = undefined) {
    //         if(row == undefined) {
    //             if(this.row == undefined) {
    //                 throw new Error("Row undefined, pass as argument or call set(column, row) beforehand!")
    //             }
    //             row = this.row+1
    //         } else {
    //             row = row+1
    //         }
    //         return this.y+this.height/this.rows*row
    //     }
    }
}