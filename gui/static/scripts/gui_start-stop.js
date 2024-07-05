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
        p.fill(192)
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
      p.fill(128, 128, 128, 192)
      p.rectMode(p.CORNER)
      p.rect(0, 0, p.width, p.height)
      p.fill(0)
      p.textAlign(p.CENTER, p.TOP)
      p.text(stateText, p.width/2, 100)
    }

    cursorObject.display(p)

    if (typeof recorder !== 'undefined') {
      p.textSize(defaultTextSize / 2)
      recorder.display(p)
    }

    this.rosComm.setTimeout(1000)
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
    grid.set(0, 0, grid.columns, grid.rows)
    stopButton = new OneTimeButton(grid.x, grid.y, grid.w, grid.h, stopButtonHandler, hideStopButton, "Stop", p.loadImage("static/icons/icons_stop.svg"), 250, p.color(200, 50, 0), p.color(200, 200, 50), p.color(0, 200, 50))
    stopButton.disable()
    stopButton.hide()
    addButton(stopButton)

    grid.set(0, 0, 1, 1)
    lockButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, lockButtons, lockButtonHandler, "Lock", p.loadImage("static/icons/icons_lock.svg"), 1000, p.color(0, 150, 50), p.color(200, 50, 0), p.color(200, 200, 50))
    topBarButtons.push(lockButton)
    lockButtons.push(lockButton)
    unlockButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, lockButtons, unLockButtonHandler, "Unlock", p.loadImage("static/icons/icons_unlock.svg"), 2000, p.color(200, 50, 0), p.color(0, 150, 50), p.color(200, 200, 50))
    topBarButtons.push(unlockButton)
    lockButtons.push(unlockButton)
    unlockButton.hide()
    grid.set(grid.columns - 1, 0, 1, 1)
    resetButton = new OneTimeButton(grid.x, grid.y, grid.w, grid.h, resetButtonHandler, undefined, "Reset", p.loadImage("static/icons/icons_reset.svg"), 1000, p.color(0, 50, 200), p.color(20, 150, 200), p.color(20, 200, 200))
    topBarButtons.push(resetButton)

    grid.set(1, 0, 2, 1)
    strafeTabButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, tabButtons, strafeTabButtonHandler, "Strafe", p.loadImage("static/icons/icons_strafe.svg"), 1000, p.color(150, 110, 10), p.color(222, 252, 55), p.color(0, 200, 50))
    tabButtons.push(strafeTabButton)
    topBarButtons.push(strafeTabButton)
    grid.set(3, 0, 2, 1)
    forwardAndGrabTabButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, tabButtons, forwardGrabTabButtonHandler, "ForwardAndGrab", p.loadImage("static/icons/icons_forward-backward.svg"), 1000, p.color(150, 110, 10), p.color(222, 252, 55), p.color(0, 200, 50))
    tabButtons.push(forwardAndGrabTabButton)
    topBarButtons.push(forwardAndGrabTabButton)

    for (const button of topBarButtons) {
      addButton(button)
    }
  }

  function createBottomButtons() {
    let grid = new GazeControl.Grid(0, p.height * 6 / 7, p.width, p.height * 1 / 7, 6, 1)
    grid.set(0, 0, grid.columns, grid.rows)
    startButton = new LatchingButton(grid.x, grid.y, grid.w, grid.h, startButtonHandler, "Start", p.loadImage("static/icons/icons_start.svg"), 1000, p.color(0, 150, 50), p.color(20, 200, 200), p.color(20, 150, 200))
    startButton.disable()
    addButton(startButton)
  }

  function createAxisButtons() {
    let webcamWidth = p.width
    let webcamHeight = p.height * (5 / 7)
    let webcamX = 0
    let webcamY = (p.height - webcamHeight) / 2
    let grid = new GazeControl.Grid(webcamX, webcamY, webcamWidth, webcamHeight, 5, 5)
    let dwellDelay = 500

    grid.set(0, 0)
    strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Up Left", p.loadImage("static/icons/icons_up-left.svg"), dwellDelay))
    grid.set((grid.columns - 1) / 2, 0)
    strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Up", p.loadImage("static/icons/icons_up.svg"), dwellDelay))
    grid.set(grid.columns - 1, 0)
    strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Up Right", p.loadImage("static/icons/icons_up-right.svg"), dwellDelay))
    grid.set(0, (grid.rows - 1) / 2)
    strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Left", p.loadImage("static/icons/icons_left.svg"), dwellDelay))
    grid.set(grid.columns - 1, (grid.rows - 1) / 2)
    strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Right", p.loadImage("static/icons/icons_right.svg"), dwellDelay))
    grid.set(0, grid.rows - 1)
    strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Down Left", p.loadImage("static/icons/icons_down-left.svg"), dwellDelay))
    grid.set((grid.columns - 1) / 2, grid.rows - 1)
    strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Down", p.loadImage("static/icons/icons_down.svg"), dwellDelay))
    grid.set(grid.columns - 1, grid.rows - 1)
    strafeButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, strafeButtons, axisButtonHandler, "Down Right", p.loadImage("static/icons/icons_down-right.svg"), dwellDelay))
    for (const button of strafeButtons) {
      axisButtons.push(button)
    }

    grid.set(0, 0)
    forwardAndGrabButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, forwardAndGrabButtons, axisButtonHandler, "Forward", p.loadImage("static/icons/icons_forward.svg"), dwellDelay))
    grid.set(grid.columns - 1, 0)
    forwardAndGrabButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, forwardAndGrabButtons, axisButtonHandler, "Backward", p.loadImage("static/icons/icons_backward.svg"), dwellDelay))
    grid.set(0, grid.rows - 1)
    forwardAndGrabButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, forwardAndGrabButtons, axisButtonHandler, "Open", p.loadImage("static/icons/icons_open.svg"), dwellDelay))
    grid.set(grid.columns - 1, grid.rows - 1)
    forwardAndGrabButtons.push(new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, forwardAndGrabButtons, axisButtonHandler, "Close", p.loadImage("static/icons/icons_close.svg"), dwellDelay))
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

  function axisButtonHandler(button) {
    unhideStopButton()
    stopButton.reset()
    stopButton.enable()

    startButton.enable()
    switch (button.name) {
      case "Up Left":
        moveAxis = [1, 0, 1, 0, 0, 0]
        break
      case "Up":
        moveAxis = [0, 0, 1, 0, 0, 0]
        break
      case "Up Right":
        moveAxis = [-1, 0, 1, 0, 0, 0]
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
      recorder.write(["Move Axis Set", moveAxis])
    }
  }

  function stopButtonHandler(button) {
    console.log("Stop!")
    rosComm.stopMovement()

    // button.disable()
    startButton.reset()
    startButton.disable()

    for (const button of axisButtons) {
      button.reset()
      button.enable()
    }
  }

  function startButtonHandler(button) {
    console.log("Start!")
    stopButton.enable()

    for (const button of axisButtons) {
      button.disable()
    }

    console.log(moveAxis)
    if (moveAxis.length == 3) {
      rosComm.fingerPositionSet(moveAxis[0], moveAxis[1], moveAxis[2])
    } else {
      rosComm.toolMoveContinuous(moveAxis[0], moveAxis[1], moveAxis[2], moveAxis[3], moveAxis[4], moveAxis[5])
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
    // rosComm.returnHome()
    setForwardPosition()
    // setTimeout(setForwardPosition, 3000)
    if (typeof recorder !== 'undefined') {
      recorder.stop()
    }
  }

  function setForwardPosition() {
    // rosComm.fingerPositionSet(0, 0, 0)
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

  function hideStopButton() {
    for (const button of topBarButtons) {
      if (button.name != unlockButton) {
        button.unhide()
      }
    }

    stopButton.hide()
  }

  function unhideStopButton() {
    stopButton.unhide()
    for (const button of topBarButtons) {
      button.hide()
    }
  }

  function clearSelections() {
    startButton.reset()
    startButton.disable()

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