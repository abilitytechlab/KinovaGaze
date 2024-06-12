let GazeControl = {};
{
    GazeControl.Cursor = class {
        enabled = true

        constructor() {
            noCursor()
            this.x = -100
            this.y = -100
        }

        display() {
            fill(255)
            circle(this.x, this.y, 20)
            fill(0)
            circle(this.x, this.y, 2)
        }

        cursorMoved(x, y) {
            this.x = x
            this.y = y
            this.update()
        }

        update() {
            if(this.enabled) {
                for (const button of buttons) {
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
            for (const button of buttons) {
                button.unHover()
            }
        }

        enable() {
            this.enabled = true
        }
    }

    GazeControl.Button = class {
        dwellProgress = 0.0
        visible = true
        enabled = true
        dwelled = false
        active = false
        
        

        constructor(x, y, w, h, name = "", label = undefined, dwellDelay = 1000, defaultColor = color(200, 50, 0), hoverColor = color(200, 200, 50), activatedColor = color(0, 200, 50)) {
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

        update() {
            if (this.dwelled == true && this.active == false && this.enabled == true && this.visible == true) {
                if (this.dwellProgress >= this.dwellDelay) {
                    this.activate()
                } else {
                    this.dwellProgress += dt
                    // print(dt + " " + this.dwellProgress + " " + this.dwellDelay)
                    this.currentColor = lerpColor(this.defaultColor, this.hoverColor, this.dwellProgress / this.dwellDelay)
                }
            }
        }

        display() {
            if(this.visible) {
                let fillColor = this.currentColor
                if(!this.enabled) {
                    fillColor = lerpColor(fillColor, color(64, 64, 64, 64), 0.5)
                }
                fill(fillColor)
                rect(this.x, this.y, this.w, this.h)
                textAlign(CENTER, CENTER)
                fill(0)
                text(this.label, this.x, this.y, this.w, this.h)
            }
        }

        hover() {
            this.dwelled = true
            this.onHover()
        }

        onHover() {

        }

        unHover() {
            this.dwelled = false
            if (this.active == false) {
                this.reset()
            }
            this.onUnhover()
        }

        onUnhover() {

        }

        activate() {
            this.currentColor = this.activatedColor
            this.active = true
            this.onActivate()
        }

        onActivate() {

        }

        reset() {
            this.currentColor = this.defaultColor
            this.active = false
            this.dwellProgress = 0
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