"use strict";

const gui = ( p ) => {
  let canvas
  const baseWidth = 640
  const baseHeight = 480
  const aspectRatio = baseWidth / baseHeight
  let scaleFactor = 1
  let defaultTextSize = 32

  let rosComm
  let connectionState = "Loading"
  let moveAxis = [0, 0, 0, 0, 0, 0]

  let stream
  let streamLoaded = false

  let dt
  let lastTime

  let cursorObject
  let buttons = []
  const oldButtonState = {}
  const buttonDimentions = {}
  const topBarButtons = []
  const tabButtons = []
  const lockButtons = []
  const axisButtons = []
  const strafeButtons = []
  const forwardAndGrabButtons = []
  let stopButton
  let startButton
  let lockButton
  let unlockButton
  let resetButton
  let strafeTabButton
  let forwardAndGrabTabButton

  p.setup = () => {
    canvas = p.createCanvas(baseWidth, baseHeight)
    updateCanvasDimensions()

    rosComm = new RosComm('ws://kinovagaze.local:9090', onConnection, onError, onClose)
    window.rosComm = rosComm // Allows sending ros commands via the browser console

    lastTime = Date.now()

    createAxisButtons()
    createTopButtons()
    createBottomButtons()
    cursorObject = new GazeControl.Cursor(p, buttons)
    canvas.mouseOut(cursorObject.disable.bind(cursorObject))
    canvas.mouseOver(cursorObject.enable.bind(cursorObject))

    stream = p.createImg("/video_feed", "webcam feed", undefined, streamLoadedHandler)
    stream.hide()

    if (typeof recorder !== 'undefined') {
      recorder.initialize(canvas)
    }
  }

  p.draw = () => {
    p.textSize(defaultTextSize)
    dt = Date.now() - lastTime
    lastTime = Date.now()

    p.background(220);
    if (streamLoaded) {
      try {
        p.imageMode(p.CENTER)
        p.image(stream, p.width / 2, p.height / 2, p.width, p.width * 0.5625)
      } catch (error) {
        p.textAlign(p.CENTER, p.CENTER)
        p.fill(200, 0, 0)
        p.text("Something went wrong with the camera feed,\nplease reload the page.", p.width / 2, p.height / 2)
      }
    } else {
      p.textAlign(p.CENTER, p.CENTER)
      p.fill(0)
      p.text("Webcam stream loading, if this takes a while\nplease ensure camera is connected and refresh this page.", p.width / 2, p.height / 2)
    }


    if (connectionState == "Connected") {
      for (const button of buttons) {
        button.update(dt)
        button.display(p)
      }
    }
    else {
      let stateText = ""
      switch (connectionState) {
        case "Closed":
          stateText = "Connection to robot arm lost,\ncheck if arm is on and connected,\nthen try refreshing the page."
          break
        case "Error":
          stateText = "Error connecting to the robot arm,\ncheck if arm is on and connected,\nthen try refreshing the page."
          break
        case "Loading":
          stateText = "Connecting to the robot arm, please wait..."
          break
        default:
          stateText = "Something went wrong,\nplease reload the page."
      }
      p.textAlign(p.CENTER, p.TOP)
      p.text(stateText, p.width/2, 100)
      p.fill(128, 0, 0, 192)
      p.rectMode(CORNER)
      p.rect(0, 0, p.width, p.height)
    }

    cursorObject.display(p)

    if (typeof recorder !== 'undefined') {
      p.textSize(defaultTextSize / 2)
      recorder.display(p)
    }
  }

  function onConnection() {
    connectionState = "Connected"
  }

  function onError() {
    connectionState = "Error"
  }

  function onClose() {
    connectionState = "Closed"
  }

  function createTopButtons() {
    let grid = new GazeControl.Grid(0, 0, p.width, p.height * 1 / 7, 6, 1)

    grid.set(0, 0, 1, 1)
    lockButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, lockButtons, lockButtonHandler, "Lock", "Lock", 1000, p.color(0, 150, 50), p.color(200, 50, 0), p.color(200, 200, 50))
    topBarButtons.push(lockButton)
    lockButtons.push(lockButton)
    unlockButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, lockButtons, unLockButtonHandler, "Unlock", "Unlock", 2000, p.color(200, 50, 0), p.color(0, 150, 50), p.color(200, 200, 50))
    topBarButtons.push(unlockButton)
    lockButtons.push(unlockButton)
    unlockButton.hide()
    grid.set(grid.columns - 1, 0, 1, 1)
    resetButton = new OneTimeButton(grid.x, grid.y, grid.w, grid.h, resetButtonHandler, undefined, "Reset", "Reset", 1000, p.color(0, 50, 200), p.color(20, 150, 200), p.color(20, 200, 200))
    topBarButtons.push(resetButton)

    grid.set(1, 0, 2, 1)
    strafeTabButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, tabButtons, strafeTabButtonHandler, "Strafe", "Strafe", 1000, p.color(150, 110, 10), p.color(222, 252, 55), p.color(0, 200, 50))
    tabButtons.push(strafeTabButton)
    topBarButtons.push(strafeTabButton)
    grid.set(3, 0, 2, 1)
    forwardAndGrabTabButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, tabButtons, forwardGrabTabButtonHandler, "ForwardAndGrab", "Forward & Grab", 1000, p.color(150, 110, 10), p.color(222, 252, 55), p.color(0, 200, 50))
    tabButtons.push(forwardAndGrabTabButton)
    topBarButtons.push(forwardAndGrabTabButton)

    for (const button of topBarButtons) {
      addButton(button)
    }
  }

  function createBottomButtons() {
    let grid = new GazeControl.Grid(0, p.height * 6 / 7, p.width, p.height * 1 / 7, 6, 1)
    grid.set(0, 0, grid.columns, grid.rows)
    stopButton = new OneTimeButton(grid.x, grid.y, grid.w, grid.h, stopButtonHandler, undefined, "Stop", "Stop", 250, p.color(200, 50, 0), p.color(200, 200, 50), p.color(0, 200, 50))
    addButton(stopButton)
  }

  function createAxisButtons() {
    let webcamWidth = p.width
    let webcamHeight = p.height * (5 / 7)
    let webcamX = 0
    let webcamY = (p.height - webcamHeight) / 2
    let grid = new GazeControl.Grid(webcamX, webcamY, webcamWidth, webcamHeight, 5, 5)
    let dwellDelay = 500

    grid.set(0, 0)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Up Left", "Up Left", dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set((grid.columns - 1) / 2, 0)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Up", "Up", dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(grid.columns - 1, 0)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Up Right", "Up Right", dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(0, (grid.rows - 1) / 2)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Left", "Left", dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(grid.columns - 1, (grid.rows - 1) / 2)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Right", "Right", dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(0, grid.rows - 1)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Down Left", "Down Left", dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set((grid.columns - 1) / 2, grid.rows - 1)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Down", "Down", dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(grid.columns - 1, grid.rows - 1)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Down Right", "Down Right", dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    for (const button of strafeButtons) {
      axisButtons.push(button)
    }

    grid.set(0, 0)
    forwardAndGrabButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Forward", "Forward", dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(grid.columns - 1, 0)
    forwardAndGrabButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Backward", "Backward", dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(0, grid.rows - 1)
    forwardAndGrabButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Open", "Open", dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(grid.columns - 1, grid.rows - 1)
    forwardAndGrabButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Close", "Close", dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    for (const button of forwardAndGrabButtons) {
      axisButtons.push(button)
    }

    for (const button of axisButtons) {
      button.hide()
      addButton(button)
    }
  }

  class LatchingSetButton extends GazeControl.Button {
    constructor(x, y, w, h, buttonSet, callback, name = undefined, label = undefined, dwellDelay = 1000, defaultColor = p.color(150, 110, 10), hoverColor = p.color(255, 230, 0), activatedColor = p.color(222, 252, 55)) {
      console.log(label)
      super(p, x, y, w, h, name, label, dwellDelay, defaultColor, hoverColor, activatedColor)
      this.buttonSet = buttonSet
      this.callback = callback
    }

    onActivate() {
      for (const button of this.buttonSet) {
        if (button !== this) {
          button.reset()
        }
      }
      if(typeof(this.callback !== 'undefined')) {
        this.callback(this)
      }
    }
  }

  class LatchingButton extends GazeControl.Button {
    constructor(x, y, w, h, callback, name = undefined, label = undefined, dwellDelay = 1000, defaultColor = p.color(200, 50, 0), hoverColor = p.color(200, 200, 50), activatedColor = p.color(0, 200, 50)) {
      console.log(label)
      super(p, x, y, w, h, name, label, dwellDelay, defaultColor, hoverColor, activatedColor)
      this.callback = callback
    }
    
    onActivate() {
      if(typeof(this.callback !== 'undefined')) {
        this.callback(this)
      }
    }
  }

  class OneTimeButton extends GazeControl.Button {
    constructor(x, y, w, h, activateCallback, unhoverCallback, name = undefined, label = undefined, dwellDelay = 1000, defaultColor = p.color(200, 50, 0), hoverColor = p.color(200, 200, 50), activatedColor = p.color(0, 200, 50)) {
      console.log(label)
      super(p, x, y, w, h, name, label, dwellDelay, defaultColor, hoverColor, activatedColor)
      this.activateCallback = activateCallback
      this.unhoverCallback = unhoverCallback
    }

    onUnhover() {
      if (this.active) {
        this.reset()
        if (typeof this.unhoverCallback !== 'undefined') {
          this.unhoverCallback(this)
        }
      }
    }

    onActivate() {
      if (typeof this.activateCallback !== 'undefined') {
        this.activateCallback(this)
      }
    }
  }

  class HoldButton extends GazeControl.Button {
    constructor(x, y, w, h, activateCallback, unhoverCallback, updateCallback, timeout, name = undefined, label = undefined, dwellDelay = 1000, defaultColor = p.color(200, 50, 0), hoverColor = p.color(200, 200, 50), activatedColor = p.color(0, 200, 50)) {
      console.log(label)
      super(p, x, y, w, h, name, label, dwellDelay, defaultColor, hoverColor, activatedColor)
      this.activateCallback = activateCallback
      this.unhoverCallback = unhoverCallback
      this.updateCallback = updateCallback
      this.timeout = timeout
    }

    onUpdate() {
      if (this.active) {
        if(this.dwelled == true) {
          this.dwellProgress = this.dwellDelay
        } else {
          this.dwellProgress -= (this.dwellDelay/this.timeout)*dt
          if(this.dwellProgress <= 0) {
            this.dwellProgress = 0
            this.reset()
            return
          }
        }
        if(typeof this.updateCallback !== 'undefined') {
          this.updateCallback(this)
        }
      }
    }
    

    onUnhover() {
      if (this.active && typeof this.unhoverCallback !== 'undefined') {
        this.unhoverCallback(this)
      }
    }

    onActivate() {
      if (typeof this.activateCallback !== 'undefined') {
        this.activateCallback(this)
      }
    }
  }

  function axisButtonHandler(button) {
    stopButton.enable()

    switch (button.name) {
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
    console.log(button.name, moveAxis)
    if (typeof recorder !== 'undefined') {
      recorder.write(["Move Axis Set and Applied", moveAxis])
    }

    rosComm.setTimeout(1000)
    if (moveAxis.length == 3) {
      rosComm.fingerPositionSet(moveAxis[0], moveAxis[1], moveAxis[2])
    } else {
      rosComm.toolMoveRelative(moveAxis[0], moveAxis[1], moveAxis[2], moveAxis[3], moveAxis[4], moveAxis[5])
    }
  }

  function axisButtonHeldUpdate(button) {
    console.log(button.dwelled)
    if(button.dwelled) {
      rosComm.setTimeout(1000)
    }
  }

  function stopButtonHandler(button) {
    console.log("Stop!")
    rosComm.stopMovement()

    for (const button of axisButtons) {
      button.reset()
      button.enable()
    }
  }

  function lockButtonHandler(button) {
    clearSelections()
    for (const button of buttons) {
      if (button !== lockButton && button !== unlockButton) {
        oldButtonState[button.name] = button.enabled
        button.disable()
      }
    }
    unlockButton.unhide()
    unlockButton.enable()
    lockButton.hide()
  }

  function unLockButtonHandler(button) {
    for (const button of buttons) {
      if (button !== lockButton && button !== unlockButton) {
        if (oldButtonState[button.name] == true)
          button.enable()
      }
    }
    lockButton.unhide()
    unlockButton.hide()
  }

  function resetButtonHandler(button) {
    clearSelections()
    rosComm.returnHome()
    setTimeout(setForwardPosition, 3000)
    if (typeof recorder !== 'undefined') {
      recorder.stop()
    }
  }

  function setForwardPosition() {
    rosComm.positionForward()
    rosComm.setTimeout(15000)
  }

  function strafeTabButtonHandler(button) {
    for (const button of axisButtons) {
      button.hide()
    }
    for (const button of strafeButtons) {
      button.unhide()
    }
    if (typeof recorder !== 'undefined') {
      recorder.start(p)
    }
  }

  function forwardGrabTabButtonHandler(button) {
    for (const button of axisButtons) {
      button.hide()
    }
    for (const button of forwardAndGrabButtons) {
      button.unhide()
    }
    if (typeof recorder !== 'undefined') {
      recorder.start(p)
    }
  }

  function clearSelections() {
    for (const button of tabButtons) {
      button.reset()
    }
    for (const button of axisButtons) {
      button.reset()
      button.enable()
      button.hide()
    }
  }

  function addButton(button) {
    buttons.push(button)
    buttonDimentions[button.name] = {
      x: button.x / p.width,
      y: button.y / p.height,
      w: button.w / p.width,
      h: button.h / p.height
    }
  }

  function streamLoadedHandler(image) {
    streamLoaded = true
  }

  function updateButtonDimensions() {
    for (const button of buttons) {
      let dimentions = buttonDimentions[button.name]
      button.x = dimentions.x * p.width
      button.y = dimentions.y * p.height
      button.w = dimentions.w * p.width
      button.h = dimentions.h * p.height
    }
  }

  p.mouseMoved = () => {
    if (typeof cursorObject !== 'undefined') {
      cursorObject.cursorMoved(p.mouseX, p.mouseY)
    }
  }

  p.windowResized = () => {
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

    canvasWidth = p.windowWidth
    canvasHeight = p.windowHeight
    // console.log(windowWidth, windowHeight, aspectRatio, canvasWidth, canvasHeight)

    p.resizeCanvas(canvasWidth, canvasHeight)

    scaleFactor = baseWidth / canvasWidth

    const x = (p.windowWidth - canvasWidth) / 2
    const y = (p.windowHeight - canvasHeight) / 2
    canvas.position(x, y)

    p.pixelDensity(window.devicePixelRatio)
    p.strokeWeight(2 * scaleFactor)
    defaultTextSize = 32
  }
};

let myp5 = new p5(gui)