const buttons = []
const buttonDimentions = {}

const baseWidth = 640
const baseHeight = 480
const aspectRatio = baseWidth / baseHeight
let scaleFactor = 1
defaultTextSize = 32

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

  if(typeof recorder !== 'undefined') {
    recorder.initialize(this.canvas)
  }
}

function draw() {
  textSize(defaultTextSize)
  this.dt = Date.now() - this.lastTime
  this.lastTime = Date.now()

  background(220);

  try {
    imageMode(CENTER)
    image(this.stream, width/2, height/2, width, width*0.5625)
  } catch(error) {
    textAlign(CENTER, CENTER)
    fill(200, 0, 0)
    text("Something went wrong with the camera feed\nPlease reload the page", width/2, height/2)
  }
  

  for (const button of buttons) {
    button.update()
    button.display()
  }

  this.cursorObject.display()

  if(typeof recorder !== 'undefined') {
    textSize(defaultTextSize/2)
    recorder.display()
  }
}

function createTopButtons() {
  let grid = new GazeControl.Grid(0, 0, width, height*1/7, 6, 1)


  topBarButtons = []

  lockButtons = []
  grid.set(0, 0, 1, 1)
  lockButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, lockButtons, lockButtonHandler, "Lock", "Lock", 1000, color(0, 150, 50), color(200, 50, 0), color(200, 200, 50))
  topBarButtons.push(lockButton)
  lockButtons.push(lockButton)
  unlockButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, lockButtons, unLockButtonHandler, "Unlock", "Unlock", 2000, color(200, 50, 0), color(0, 150, 50), color(200, 200, 50))
  topBarButtons.push(unlockButton)
  lockButtons.push(unlockButton)
  unlockButton.hide()
  grid.set(grid.columns-1, 0, 1, 1)
  resetButton = new OneTimeButton(grid.x, grid.y, grid.w, grid.h, resetButtonHandler, undefined, "Reset", "Reset", 1000, color(0, 50, 200), color(20, 150, 200), color(20, 200, 200))
  topBarButtons.push(resetButton)

  tabButtons = []
  grid.set(1, 0, 2, 1)
  strafeTabButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, tabButtons, strafeTabButtonHandler, "Strafe", "Strafe", 1000, color(150, 110, 10), color(222, 252, 55), color(0, 200, 50))
  tabButtons.push(strafeTabButton)
  topBarButtons.push(strafeTabButton)
  grid.set(3, 0, 2, 1)
  forwardAndGrabTabButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, tabButtons, forwardGrabTabButtonHandler, "ForwardAndGrab", "Forward & Grab", 1000, color(150, 110, 10), color(222, 252, 55), color(0, 200, 50))
  tabButtons.push(forwardAndGrabTabButton)
  topBarButtons.push(forwardAndGrabTabButton)

  for (const button of topBarButtons) {
    addButton(button)
  }
}

function createBottomButtons() {
  let grid = new GazeControl.Grid(0, height*6/7, width, height*1/7, 6, 1)
  // grid.set(0, 0, grid.columns, grid.rows)
  // startButton = new LatchingButton(grid.x, grid.y, grid.w, grid.h, startButtonHandler, "Start", "Start", 1000, color(0, 150, 50), color(20, 200, 200), color(20, 150, 200))
  // startButton.disable()
  // addButton(startButton)

  grid.set(0, 0, grid.columns, grid.rows)
  stopButton = new OneTimeButton(grid.x, grid.y, grid.w, grid.h, stopButtonHandler, undefined, "Stop", "Stop", 250, color(200, 50, 0), color(200, 200, 50), color(0, 200, 50))
  addButton(stopButton)
}

function createAxisButtons() {
  this.direction = [0, 0, 0, 0, 0, 0]
  let webcamWidth = width
  let webcamHeight = height*(5/7)
  let webcamX = 0
  let webcamY = (height - webcamHeight)/2
  let grid = new GazeControl.Grid(webcamX, webcamY, webcamWidth, webcamHeight, 5, 5)
  let dwellDelay = 500

  axisButtons = []

  strafeButtons = []
  grid.set(0, 0)
  strafeButtons.push(new OneTimeButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, "Up Left", "Up Left", dwellDelay, color(150, 110, 10), color(255, 230, 0), color(222, 252, 55)))
  grid.set((grid.columns-1)/2, 0)
  strafeButtons.push(new OneTimeButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, "Up", "Up", dwellDelay, color(150, 110, 10), color(255, 230, 0), color(222, 252, 55)))
  grid.set(grid.columns-1, 0)
  strafeButtons.push(new OneTimeButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, "Up Right", "Up Right", dwellDelay, color(150, 110, 10), color(255, 230, 0), color(222, 252, 55)))
  grid.set(0, (grid.rows-1)/2)
  strafeButtons.push(new OneTimeButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, "Left", "Left", dwellDelay, color(150, 110, 10), color(255, 230, 0), color(222, 252, 55)))
  grid.set(grid.columns-1, (grid.rows-1)/2)
  strafeButtons.push(new OneTimeButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, "Right", "Right", dwellDelay, color(150, 110, 10), color(255, 230, 0), color(222, 252, 55)))
  grid.set(0, grid.rows-1)
  strafeButtons.push(new OneTimeButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, "Down Left", "Down Left", dwellDelay, color(150, 110, 10), color(255, 230, 0), color(222, 252, 55)))
  grid.set((grid.columns-1)/2, grid.rows-1)
  strafeButtons.push(new OneTimeButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, "Down", "Down", dwellDelay, color(150, 110, 10), color(255, 230, 0), color(222, 252, 55)))
  grid.set(grid.columns-1, grid.rows-1)
  strafeButtons.push(new OneTimeButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, "Down Right", "Down Right", dwellDelay, color(150, 110, 10), color(255, 230, 0), color(222, 252, 55)))
  for (const button of strafeButtons) {
    axisButtons.push(button)
  }

  forwardAndGrabButtons = []
  grid.set(0, 0)
  forwardAndGrabButtons.push(new OneTimeButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, "Forward", "Forward", dwellDelay, color(150, 110, 10), color(255, 230, 0), color(222, 252, 55)))
  grid.set(grid.columns-1, 0)
  forwardAndGrabButtons.push(new OneTimeButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, "Backward", "Backward", dwellDelay, color(150, 110, 10), color(255, 230, 0), color(222, 252, 55)))
  grid.set(0, grid.rows-1)
  forwardAndGrabButtons.push(new OneTimeButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, "Open", "Open", dwellDelay, color(150, 110, 10), color(255, 230, 0), color(222, 252, 55)))
  grid.set(grid.columns-1, grid.rows-1)
  forwardAndGrabButtons.push(new OneTimeButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, "Close", "Close", dwellDelay, color(150, 110, 10), color(255, 230, 0), color(222, 252, 55)))
  for (const button of forwardAndGrabButtons) {
    axisButtons.push(button)
  }

  for (const button of axisButtons) {
    button.hide()
    addButton(button)
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

class RepeatButton extends GazeControl.Button {
  constructor(x, y, w, h, activateCallback, name = undefined, label = undefined, dwellDelay = 1000, defaultColor = color(200, 50, 0), hoverColor = color(200, 200, 50), activatedColor = color(0, 200, 50)) {
    print(label)
    super(x=x, y=y, w=w, h=h, name=name, label=label, dwellDelay=dwellDelay, defaultColor=defaultColor, hoverColor=hoverColor, activatedColor=activatedColor)
    this.activateCallback = activateCallback
    this.unhoverCallback = unhoverCallback
  }

  onActivate() {
    this.reset()
    if(this.activateCallback !== undefined) {
      this.activateCallback(this)
    }
  }
}

function axisButtonHandler(button) {
  // unhideStopButton()
  // stopButton.reset()
  // stopButton.enable()
  
  // startButton.enable()
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
  if(typeof recorder !== 'undefined') {
    recorder.write(["Move Axis Set and Applied", moveAxis])
  }

  if(moveAxis.length == 3) {
    fingerPositionSet(moveAxis)
  } else {
    tool_move_relative(moveAxis[0]*0.1, moveAxis[1]*0.1, moveAxis[2]*0.1, moveAxis[3]*1, moveAxis[4]*1, moveAxis[5]*1)
  }
}

function stopButtonHandler(button) {
  print("Stop!")
  stopMovement()
  
  for(const button of axisButtons) {
    button.reset()
    button.enable()
  }
}

// function startButtonHandler(button) {
//   print("Start!")
//   stopButton.enable()

//   for(const button of axisButtons) {
//     button.disable()
//   }

//   print(moveAxis)
//   if(moveAxis.length == 3) {
//     fingerPositionSet(moveAxis)
//   } else {
//     toolMoveContinuous(moveAxis)
//   }
  
// }

function lockButtonHandler(button) {
  clearSelections()
  oldButtonState = {}
  for(const button of buttons) {
    if(button !== lockButton && button !== unlockButton) {
      oldButtonState[button.name] = button.enabled
      button.disable()
    }
  }
  unlockButton.unhide()
  unlockButton.enable()
  lockButton.hide()
}

function unLockButtonHandler(button) {
  for(const button of buttons) {
    if(button !== lockButton && button !== unlockButton) {
      if(oldButtonState[button.name] == true)
      button.enable()
    }
  }
  lockButton.unhide()
  unlockButton.hide()
}

function resetButtonHandler(button) {
  clearSelections()
  returnHome()
  setTimeout(positionForward, 3000)
  if(typeof recorder !== 'undefined') {
    recorder.stop()
  }
}

function strafeTabButtonHandler(button) {
  for(const button of axisButtons) {
    button.hide()
  }
  for(const button of strafeButtons) {
    button.unhide()
  }
  if(typeof recorder !== 'undefined') {
    recorder.start()
  }
}

function forwardGrabTabButtonHandler(button) {
  for(const button of axisButtons) {
    button.hide()
  }
  for(const button of forwardAndGrabButtons) {
    button.unhide()
  }
  if(typeof recorder !== 'undefined') {
    recorder.start()
  }
}

// function hideStopButton() {
//   for(const button of topBarButtons) {
//     if(button.name != unlockButton) {
//       button.unhide()
//     }
//   }

//   stopButton.hide()
// }

// function unhideStopButton() {
//   stopButton.unhide()
//   for(const button of topBarButtons) {
//     button.hide()
//   }
// }

function clearSelections() {
  // startButton.reset()
  // startButton.disable()
  
  for(const button of axisButtons) {
    button.reset()
    button.enable()
  }
  
  for(const button of tabButtons) {
    button.reset()
  }
  for(const button of axisButtons) {
    button.reset()
    button.hide()
  }
}

function addButton(button) {
  buttons.push(button)
  buttonDimentions[button.name] = {
    x: button.x/width, 
    y: button.y/height, 
    w: button.w/width, 
    h: button.h/height
  }
}

function updateButtonDimensions() {
  for(const button of buttons) {
    dimentions = buttonDimentions[button.name]
    button.x = dimentions.x*width
    button.y = dimentions.y*height
    button.w = dimentions.w*width
    button.h = dimentions.h*height
  }
}

function mouseMoved() {
  if (typeof cursorObject !== 'undefined') {
    this.cursorObject.cursorMoved(mouseX, mouseY)
  }
}

function windowResized() {
  updateCanvasDimensions()
  updateButtonDimensions()
}


function updateCanvasDimensions() {
  let canvasWidth
  let canvasHeight

  // if (windowWidth / windowHeight > aspectRatio) {
  //   canvasWidth = windowHeight * aspectRatio
  //   canvasHeight = windowHeight
  // } else {
  //   canvasWidth = windowWidth
  //   canvasHeight = windowWidth / aspectRatio
  // }

  canvasWidth = windowWidth
  canvasHeight = windowHeight
  // print(windowWidth, windowHeight, aspectRatio, canvasWidth, canvasHeight)

  resizeCanvas(canvasWidth, canvasHeight)

  scaleFactor = baseWidth / canvasWidth

  const x = (windowWidth - canvasWidth) / 2
  const y = (windowHeight - canvasHeight) / 2
  canvas.position(x, y)

  pixelDensity(window.devicePixelRatio)
  strokeWeight(2 * scaleFactor)
  defaultTextSize = 32
}