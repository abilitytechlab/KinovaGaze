const buttons = []

const baseWidth = 640
const baseHeight = 480
const aspectRatio = baseWidth / baseHeight
let scaleFactor = 1

function setup() {
  this.canvas = createCanvas(baseWidth, baseHeight)
  updateCanvasDimensions()

  this.lastTime = Date.now()

  buttons[0] = new GazeButton(width / 2, height / 2, 100, 100, label = "Hello World!")
  this.cursorObject = new GazeCursor()

  this.stream = createImg("/video_feed", "webcam feed")
  this.stream.hide()
}

function draw() {
  this.dt = Date.now() - this.lastTime
  this.lastTime = Date.now()

  background(220);

  imageMode(CENTER)
  image(this.stream, width/2, height/2, width, width*0.5625)

  for (const button of buttons) {
    button.update()
    button.display()
  }

  cursorObject.display()
}

var yellowColorPalette {

}

class GazeCursor {
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

class GazeButton {
  dwellProgress = 0.0
  dwelled = false
  active = false
  enabled = true

  constructor(x, y, w, h, label = "", dwellDelay = 1000, defaultColor = color(200, 50, 0), hoverColor = color(200, 200, 50), activatedColor = color(0, 200, 50)) {
    this.x = x
    this.y = y
    this.w = w
    this.h = h
    this.label = label
    this.dwellDelay = dwellDelay
    this.currentColor = defaultColor
    this.defaultColor = defaultColor
    this.hoverColor = hoverColor
    this.activatedColor = activatedColor
  }

  update() {
    if (this.dwelled == true && this.active == false) {
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
    fill(this.currentColor)
    rect(this.x, this.y, this.w, this.h)
    textAlign(CENTER, CENTER)
    fill(0)
    text(this.label, this.x, this.y, this.w, this.h)
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
    this.onUnHover()
  }

  onUnHover() {

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
    this.dwellProgress = 0.0
  }
}

function mouseMoved() {
  if (typeof cursorObject !== 'undefined') {
    this.cursorObject.cursorMoved(mouseX, mouseY)
  }
}

function windowResized() {
  updateCanvasDimensions()
}


function updateCanvasDimensions() {
  let canvasWidth
  let canvasHeight
  if (windowWidth / windowHeight > aspectRatio) {
    canvasWidth = windowHeight * aspectRatio
    canvasHeight = windowHeight
  } else {
    canvasWidth = windowWidth
    canvasHeight = windowWidth / aspectRatio
  }
  print(windowWidth, windowHeight, aspectRatio, canvasWidth, canvasHeight)

  resizeCanvas(canvasWidth, canvasHeight)

  scaleFactor = baseWidth / canvasWidth

  const x = (windowWidth - canvasWidth) / 2
  const y = (windowHeight - canvasHeight) / 2
  canvas.position(x, y)

  pixelDensity(window.devicePixelRatio)
  strokeWeight(2 * scaleFactor)
}