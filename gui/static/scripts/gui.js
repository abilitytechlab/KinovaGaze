const buttons = []

const baseWidth = 640
const baseHeight = 480
const aspectRatio = baseWidth / baseHeight
let scaleFactor = 1

moveAxis = [0, 0, 0, 0, 0, 0]

function setup() {
  this.canvas = createCanvas(baseWidth, baseHeight)
  updateCanvasDimensions()

  this.lastTime = Date.now()

  createAxisButtons()
  createTopButtons()
  createBottomButtons()
  this.cursorObject = new GazeControl.Cursor()
  this.canvas.mouseOut(this.cursorObject.disable)
  this.canvas.mouseOver(this.cursorObject.enable)

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

  this.cursorObject.display()
}

function createTopButtons() {
  let grid = new GazeControl.Grid(0, 0, width, height*1/7, 6, 1)
  grid.set(0, 0, grid.columns, grid.rows)
  stopButton = new OneTimeButton(grid.x, grid.y, grid.w, grid.h, stopButtonHandler, hideStopButton, "Stop", "Stop", 250, color(200, 50, 0), color(200, 200, 50), color(0, 200, 50))
  stopButton.disable()
  stopButton.hide()
  buttons.push(stopButton)


  topBarButtons = []

  lockButtons = []
  grid.set(0, 0, 1, 1)
  lockButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, lockButtons, lockButtonHandler, "Lock", "Lock", 1000, color(0, 150, 50), color(200, 50, 0), color(200, 200, 50))
  lockButton.disable()
  topBarButtons.push(lockButton)
  grid.set(grid.columns-1, 0, 1, 1)
  unlockButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, lockButtons, unLockButtonHandler, "Unlock", "Unlock", 1000, color(200, 50, 0), color(0, 150, 50), color(200, 200, 50))
  unlockButton.disable()
  unlockButton.hide()
  topBarButtons.push(unlockButton)

  resetButton = new OneTimeButton(grid.x, grid.y, grid.w, grid.h, resetButtonHandler, undefined, "Reset", "Reset", 1000, color(0, 50, 200), color(20, 150, 200), color(20, 200, 200))
  // resetButton.disable()
  topBarButtons.push(resetButton)

  tabButtons = []
  grid.set(1, 0, 2, 1)
  strafeTabButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, tabButtons, strafeTabButtonHandler, "Strafe", "Strafe", 1000, color(150, 110, 10), color(222, 252, 55), color(0, 200, 50))
  // strafeTabButton.disable()
  tabButtons.push(strafeTabButton)
  topBarButtons.push(strafeTabButton)
  grid.set(3, 0, 2, 1)
  forwardAndGrabTabButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, tabButtons, forwardGrabTabButtonHandler, "ForwardAndGrab", "Forward & Grab", 1000, color(150, 110, 10), color(222, 252, 55), color(0, 200, 50))
  // forwardAndGrabTabButton.disable()
  tabButtons.push(forwardAndGrabTabButton)
  topBarButtons.push(forwardAndGrabTabButton)

  for (const button of topBarButtons) {
    buttons.push(button)
  }
}

function createBottomButtons() {
  let grid = new GazeControl.Grid(0, height*6/7, width, height*1/7, 6, 1)
  grid.set(0, 0, grid.columns, grid.rows)
  startButton = new LatchingButton(grid.x, grid.y, grid.w, grid.h, startButtonHandler, "Start", "Start", 1000, color(0, 150, 50), color(20, 200, 200), color(20, 150, 200))
  startButton.disable()
  buttons.push(startButton)
}

function createAxisButtons() {
  this.direction = [0, 0, 0, 0, 0, 0]
  let webcamWidth = width
  let webcamHeight = height*(5/7)
  let webcamX = 0
  let webcamY = (height - webcamHeight)/2
  let grid = new GazeControl.Grid(webcamX, webcamY, webcamWidth, webcamHeight, 5, 5)
  let dwellDelay = 250

  axisButtons = []

  strafeButtons = []
  grid.set(0, 0)
  strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Up Left", "Up Left", dwellDelay))
  grid.set((grid.columns-1)/2, 0)
  strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Up", "Up", dwellDelay))
  grid.set(grid.columns-1, 0)
  strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Up Right", "Up Right", dwellDelay))
  grid.set(0, (grid.rows-1)/2)
  strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Left", "Left", dwellDelay))
  grid.set(grid.columns-1, (grid.rows-1)/2)
  strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Right", "Right", dwellDelay))
  grid.set(0, grid.rows-1)
  strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Down Left", "Down Left", dwellDelay))
  grid.set((grid.columns-1)/2, grid.rows-1)
  strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Down", "Down", dwellDelay))
  grid.set(grid.columns-1, grid.rows-1)
  strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Down Right", "Down Right", dwellDelay))
  for (const button of strafeButtons) {
    axisButtons.push(button)
  }

  forwardAndGrabButtons = []
  grid.set(0, 0)
  forwardAndGrabButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, forwardAndGrabButtons, axisButtonHandler, "Forward", "Forward", dwellDelay))
  grid.set(grid.columns-1, 0)
  forwardAndGrabButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, forwardAndGrabButtons, axisButtonHandler, "Backward", "Backward", dwellDelay))
  grid.set(0, grid.rows-1)
  forwardAndGrabButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, forwardAndGrabButtons, axisButtonHandler, "Open", "Open", dwellDelay))
  grid.set(grid.columns-1, grid.rows-1)
  forwardAndGrabButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, forwardAndGrabButtons, axisButtonHandler, "Close", "Close", dwellDelay))
  for (const button of forwardAndGrabButtons) {
    axisButtons.push(button)
  }

  for (const button of axisButtons) {
    button.hide()
    buttons.push(button)
  }
}

class LatchingSetButton extends GazeControl.Button {
  constructor(x, y, w, h, buttonSet, callback, name = undefined, label = undefined, dwellDelay = 1000, defaultColor = color(150, 110, 10), hoverColor = color(255, 230, 0), activatedColor = color(222, 252, 55)) {
    print(label)
    super(x=x, y=y, w=w, h=h, name=name, label=label, dwellDelay=dwellDelay, defaultColor=defaultColor, hoverColor=hoverColor, activatedColor=activatedColor)
    this.buttonSet = buttonSet
    this.callback = callback
  }

  onActivate() {
    for(const button of this.buttonSet) {
      if(button !== this) {
        button.reset()
      }
    }
    this.callback(this)
  }
}

class LatchingButton extends GazeControl.Button {
  constructor(x, y, w, h, callback, name = undefined, label = undefined, dwellDelay = 1000, defaultColor = color(200, 50, 0), hoverColor = color(200, 200, 50), activatedColor = color(0, 200, 50)) {
    print(label)
    super(x=x, y=y, w=w, h=h, name=name, label=label, dwellDelay=dwellDelay, defaultColor=defaultColor, hoverColor=hoverColor, activatedColor=activatedColor)
    this.callback = callback
  }

  onActivate() {
    this.callback(this)
  }
}

class OneTimeButton extends GazeControl.Button {
  constructor(x, y, w, h, activateCallback, unhoverCallback, name = undefined, label = undefined, dwellDelay = 1000, defaultColor = color(200, 50, 0), hoverColor = color(200, 200, 50), activatedColor = color(0, 200, 50)) {
    print(label)
    super(x=x, y=y, w=w, h=h, name=name, label=label, dwellDelay=dwellDelay, defaultColor=defaultColor, hoverColor=hoverColor, activatedColor=activatedColor)
    this.activateCallback = activateCallback
    this.unhoverCallback = unhoverCallback
  }

  onUnhover() {
    if(this.active) {
      this.reset()
      if(this.unhoverCallback !== undefined) {
        this.unhoverCallback(this)
      }
    }
  }

  onActivate() {
    if(this.activateCallback !== undefined) {
      this.activateCallback(this)
    }
  }
}

function axisButtonHandler(button) {
  unhideStopButton()
  stopButton.reset()
  stopButton.enable()
  
  startButton.enable()
  switch(button.name) {
    case "Up Left":
      moveAxis = [1, 0, 1, 0, 0, 0]
      break
    case "Up":
      moveAxis = [0, 0, 1, 0, 0, 0]
      break
    case "Up Right":
      moveAxis = [1, 0, 1, 0, 0, 0]
      break
    case "Left":
      moveAxis = [1, 0, 0, 0, 0, 0]
      break
    case "Right":
      moveAxis = [-1, 0, 0, 0, 0, 0]
      break
    case "Down Left":
      moveAxis = [1, 0, -1, 0, 0, 0]
      break
    case "Down":
      moveAxis = [0, 0, -1, 0, 0, 0]
      break
    case "Down Right":
      moveAxis = [-1, 0, -1, 0, 0, 0]
      break
    case "Forward":
      moveAxis = [0, -1, 0, 0, 0, 0]
      break
    case "Backward":
      moveAxis = [0, 1, 0, 0, 0, 0]
      break
    case "Open":
      moveAxis = [0, 0, 0]
      break
    case "Close":
      moveAxis = [100, 100, 100]
      break
    default:
      moveAxis = [0, 0, 0, 0, 0, 0]
  }
  print(button.name, moveAxis)
}

function stopButtonHandler(button) {
  print("Stop!")
  stopMovement()

  // button.disable()
  startButton.reset()
  startButton.disable()
  
  for(const button of axisButtons) {
    button.reset()
    button.enable()
  }
}

function startButtonHandler(button) {
  print("Start!")
  stopButton.enable()

  for(const button of axisButtons) {
    button.disable()
  }

  print(moveAxis)
  if(moveAxis.length == 3) {
    fingerPositionSet(moveAxis)
  } else {
    toolMoveContinuous(moveAxis)
  }
  
}

function lockButtonHandler(button) {

}

function unLockButtonHandler(button) {

}

function resetButtonHandler(button) {
  returnHome()
  setTimeout(positionForward, 3000)
}

function strafeTabButtonHandler(button) {
  for(const button of axisButtons) {
    button.hide()
  }
  for(const button of strafeButtons) {
    button.unhide()
  }
}

function forwardGrabTabButtonHandler(button) {
  for(const button of axisButtons) {
    button.hide()
  }
  for(const button of forwardAndGrabButtons) {
    button.unhide()
  }
}

function hideStopButton() {
  for(const button of topBarButtons) {
    if(button.name != unlockButton) {
      button.unhide()
    }
  }

  stopButton.hide()
}

function unhideStopButton() {
  stopButton.unhide()
  for(const button of topBarButtons) {
    button.hide()
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
  // print(windowWidth, windowHeight, aspectRatio, canvasWidth, canvasHeight)

  resizeCanvas(canvasWidth, canvasHeight)

  scaleFactor = baseWidth / canvasWidth

  const x = (windowWidth - canvasWidth) / 2
  const y = (windowHeight - canvasHeight) / 2
  canvas.position(x, y)

  pixelDensity(window.devicePixelRatio)
  strokeWeight(2 * scaleFactor)
  textSize(32)
}