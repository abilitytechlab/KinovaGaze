"use strict";

/**
 * Hold interface. Select an axis group, then select and hold an axis to move the arm. 
 * Look away for movement to stop after 1 second, or look at the stop button to stop the arm more quickly.
 * Lock button locks the interface, reset button returns the arm to a predifined position.
 * rosComm is exposed to the console to allow manual commands.
 * Before the arm can move, use console to call rosComm.returnHome()
 * 
 * This file is sparcely documented. Relevant parts may be better documented in whack-a-button.js, roscomm.js and gazecontrol.js
 * 
 * @param {object} p - p5.js instance
 */
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

  /**
   * Sets up the canvas, rosComm, buttons, stream and recorder
   */
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

  /**
   * Update and draw the UI
   */
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

    // This UI does not constantly update the timeout variable, only setting it when a movement is happening.
  }

  /** 
   * Callback function from rosComm for when the connection is established
   */ 
  function onConnection() {
    connectionState = "Connected"
  }

  /** 
   * Callback function from rosComm for when the connection errors
   */
  function onError() {
    connectionState = "Error"
  }

  /**
   * Callback function from rosComm for when the connection closes
   */
  function onClose() {
    connectionState = "Closed"
  }

  /**
   * Creates all the buttons for the top bar and sets their initial state
   */
  function createTopButtons() {
    let grid = new GazeControl.Grid(0, 0, p.width, p.height * 1 / 7, 6, 1)

    grid.set(0, 0, 1, 1)
    lockButton = new LatchingSetButton(grid.x, grid.y, grid.w, grid.h, lockButtons, lockButtonHandler, "Lock", p.loadImage("static/icons/icons_lock.svg"), 1000, p.color(0, 150, 50), p.color(200, 50, 0), p.color(200, 200, 50))
    topBarButtons.push(lockButton)
    lockButtons.push(lockButton)
    grid.set(0, 3, 1, 1)
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

  /**
   * Creates all the buttons for the bottom bar (aka the stop button) and sets their initial state
   */
  function createBottomButtons() {
    let grid = new GazeControl.Grid(0, p.height * 6 / 7, p.width, p.height * 1 / 7, 6, 1)
    grid.set(0, 0, grid.columns, grid.rows)
    stopButton = new OneTimeButton(grid.x, grid.y, grid.w, grid.h, stopButtonHandler, undefined, "Stop", p.loadImage("static/icons/icons_stop.svg"), 250, p.color(200, 50, 0), p.color(200, 200, 50), p.color(0, 200, 50))
    addButton(stopButton)
  }

  /**
   * Creates all the axis buttons
   */
  function createAxisButtons() {
    let webcamWidth = p.width
    let webcamHeight = p.height * (5 / 7)
    let webcamX = 0
    let webcamY = (p.height - webcamHeight) / 2
    let grid = new GazeControl.Grid(webcamX, webcamY, webcamWidth, webcamHeight, 5, 5)
    let dwellDelay = 500

    grid.set(0, 0)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Up Left", p.loadImage("static/icons/icons_up-left.svg"), dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set((grid.columns - 1) / 2, 0)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Up", p.loadImage("static/icons/icons_up.svg"), dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(grid.columns - 1, 0)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Up Right", p.loadImage("static/icons/icons_up-right.svg"), dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(0, (grid.rows - 1) / 2)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Left", p.loadImage("static/icons/icons_left.svg"), dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(grid.columns - 1, (grid.rows - 1) / 2)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Right", p.loadImage("static/icons/icons_right.svg"), dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(0, grid.rows - 1)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Down Left", p.loadImage("static/icons/icons_down-left.svg"), dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set((grid.columns - 1) / 2, grid.rows - 1)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Down", p.loadImage("static/icons/icons_down.svg"), dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(grid.columns - 1, grid.rows - 1)
    strafeButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Down Right", p.loadImage("static/icons/icons_down-right.svg"), dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    for (const button of strafeButtons) {
      axisButtons.push(button)
    }

    grid.set(0, 0)
    forwardAndGrabButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Forward", p.loadImage("static/icons/icons_forward.svg"), dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(grid.columns - 1, 0)
    forwardAndGrabButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Backward", p.loadImage("static/icons/icons_backward.svg"), dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(0, grid.rows - 1)
    forwardAndGrabButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Open", p.loadImage("static/icons/icons_open.svg"), dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    grid.set(grid.columns - 1, grid.rows - 1)
    forwardAndGrabButtons.push(new HoldButton(grid.x, grid.y, grid.w, grid.h, axisButtonHandler, undefined, axisButtonHeldUpdate, 1000, "Close", p.loadImage("static/icons/icons_close.svg"), dwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
    for (const button of forwardAndGrabButtons) {
      axisButtons.push(button)
    }

    for (const button of axisButtons) {
      button.hide()
      addButton(button)
    }
  }

  /**
   * A button which is part of a set where only one button can be active at a time.
   * When activated, disables all other buttons in its set. Includes a callback for activation.
   */
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

  /**
   * A button which deactivates when unhovered.
   * Includes a callback for activating and for unhovering.
   */
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

  /**
   * A button which stays active for a certain amount of time after being unhovered.
   * Includes a callback for activating, unhovering and updating.
   */
  class HoldButton extends GazeControl.Button {
    constructor(x, y, w, h, activateCallback, unhoverCallback, updateCallback, timeout, name = undefined, label = undefined, dwellDelay = 1000, defaultColor = p.color(200, 50, 0), hoverColor = p.color(200, 200, 50), activatedColor = p.color(0, 200, 50)) {
      console.log(label)
      super(p, x, y, w, h, name, label, dwellDelay, defaultColor, hoverColor, activatedColor)
      this.activateCallback = activateCallback
      this.unhoverCallback = unhoverCallback
      this.updateCallback = updateCallback
      this.timeout = timeout
    }

    /**
     * If the button is active but not hovered, slowly revert it to being inactive.
     * @returns 
     */
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
  /**
   * Callback method for when an axis button is activated. Activates the stop and start buttons and makes the arm move in the appropriate axis.
   * @param {object} button - button which was activated
   */
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
      recorder.write(["Move Axis Set and Applied", moveAxis])
    }

    rosComm.setTimeout(1000)
    if (moveAxis.length == 3) {
      rosComm.fingerPositionSet(moveAxis[0], moveAxis[1], moveAxis[2])
    } else {
      rosComm.toolMoveRelative(moveAxis[0], moveAxis[1], moveAxis[2], moveAxis[3], moveAxis[4], moveAxis[5])
    }
  }

  /**
   * Callback for when an axis button is active and updated, sets the timeout variable if the button is being held.
   * @param {object} button - button which was updated
   */
  function axisButtonHeldUpdate(button) {
    // console.log(button.dwelled)
    if(button.dwelled) {
      rosComm.setTimeout(1000)
    }
  }

  /**
   * Callback method for when the stop button is activated. Stops movement and resets the axis buttons.
   * @param {object} button - button which was activated
   */
  function stopButtonHandler(button) {
    console.log("Stop!")
    rosComm.stopMovement()

    for (const button of axisButtons) {
      button.reset()
      button.enable()
    }
  }
  
  /**
   * Callback method for when the lock button is activated. Stops movement, disables all other buttons and reveals the unlock button.
   * @param {object} button - button which was activated
   */
  function lockButtonHandler(button) {
    rosComm.stopMovement()
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

  /**
   * Callback method for when the unlock button is activated. Enables all other buttons and reveals the lock button.
   * @param {object} button - button which was activated
   */
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

  /**
   * Callback method for when the reset button is activated. Clears axis group selection, returns the arm to the starting position and stops the recording.
   * @param {object} button - button which was activated
   */
  function resetButtonHandler(button) {
    clearSelections()

    // rosComm.returnHome()
    // rosComm.fingerPositionSet(0, 0, 0)
    rosComm.positionForward()
    rosComm.setTimeout(15000)

    if (typeof recorder !== 'undefined') {
      recorder.stop()
    }
  }

  function setForwardPosition() {
    
  }

  /**
   * Callback method for when the strafe tab button is activated. Hides all axis buttons, then unhides the strafe axis buttons. Also start the recording.
   * @param {object} button - button which was activated
   */
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

  /**
   * Callback method for when the forward/backward/grab tab button is activated. Hides all axis buttons, then unhides the forward/backward/grab axis buttons. Also start the recording.
   * @param {object} button - button which was activated
   */
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

  /**
   * Clears all axis and axis group selections.
   */
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

  /**
   * Adds a button to the button array and stores its dimensions relative to the canvas
   * @param {object} button - Button to add
   */
  function addButton(button) {
    buttons.push(button)
    buttonDimentions[button.name] = {
      x: button.x / p.width,
      y: button.y / p.height,
      w: button.w / p.width,
      h: button.h / p.height
    }
  }

  /**
   * Callback for when the stream finishes loading.
   * @param {object} image
   */
  function streamLoadedHandler(image) {
    streamLoaded = true
  }

  /**
   * Update all button dimensions relative to the canvas size.
   */
  function updateButtonDimensions() {
    for (const button of buttons) {
      let dimentions = buttonDimentions[button.name]
      button.x = dimentions.x * p.width
      button.y = dimentions.y * p.height
      button.w = dimentions.w * p.width
      button.h = dimentions.h * p.height
    }
  }

  /**
   * Called by p5.js when the mouse moves, updates the cursor.
   */
  p.mouseMoved = () => {
    if (typeof cursorObject !== 'undefined') {
      cursorObject.cursorMoved(p.mouseX, p.mouseY)
    }
  }

  /**
   * Called by p5.js when the window resizes, updates canvas and button dimensions
   */
  p.windowResized = () => {
    updateCanvasDimensions()
    updateButtonDimensions()
  }

  /**
   * Updates the canvas dimensions to fill the entire window
   */
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