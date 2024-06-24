"use strict";

const gui = ( p ) => {
  let canvas
  const baseWidth = 640
  const baseHeight = 480
  const aspectRatio = baseWidth / baseHeight
  let scaleFactor = 1
  let defaultTextSize = 32

  let dt
  let lastTime

  let cursorObject
  const buttons = []
  const buttonDimentions = {}
  let buttonDict = {}
  const targets = []
  const badTargets = []
  const goodTargets = []
  const activeButtonsTime = {}
  let startButton

  const targetDwellDelay = 500
  const columns = 5
  const rows = 5
  const startTime = 30
  const startSpeed = 0.5
  const speedIncrement = 0.05
  const badSpawnChanceLimit = 0.3
  const badSpawnChanceIncrement = 0.5
  const buttonActiveTimeInitial = 5
  const buttonActiveTimeFinal = 2

  let started = false
  let time = 0
  let score = 0
  let spawnSpeed = 0.5
  let spawnTimer = 1
  let spawnOffsetCorrection = 0
  let buttonsHit = 0
  let averageHitSpeed = -1 

  let topText = "Press start to play!"

  p.setup = () => {
    canvas = p.createCanvas(baseWidth, baseHeight)
    updateCanvasDimensions()
    lastTime = Date.now()

    createButtons()
    cursorObject = new GazeControl.Cursor(p, buttons)
    canvas.mouseOut(cursorObject.disable.bind(cursorObject))
    canvas.mouseOver(cursorObject.enable.bind(cursorObject))

    if (typeof recorder !== 'undefined') {
      recorder.initialize(canvas)
    }
  }

  p.draw = () => {
    p.textSize(defaultTextSize)
    dt = Date.now() - lastTime
    lastTime = Date.now()
    
    if(started) {
      mainLoop()
    }

    p.background(220);

    for (const button of buttons) {
      button.update(dt)
      button.display(p)
    }

    cursorObject.display(p)
    
    p.textAlign(p.CENTER, p.CENTER)
    p.fill(0)
    let grid = new GazeControl.Grid(0, 0, p.width, p.height*1/6, 1, 2)
    grid.set(grid.columns/2, 0.5)
    p.text("Whack-A-Button: Whack the yellow buttons, not the red ones!", grid.x, grid.y)
    grid.set(grid.columns/2, 1.5)
    p.text(topText, grid.x, grid.y)

    if (typeof recorder !== 'undefined') {
      p.textSize(defaultTextSize / 2)
      recorder.display(p)
    }
  }

  function createButtons() {
    let grid = new GazeControl.Grid(0, p.height * 1/6, p.width, p.height * 5 / 6, columns, rows)
    grid.set(grid.columns/2-1.5, grid.rows/2-1, 3, 1)
    startButton = new OneTimeButton(grid.x, grid.y, grid.w, grid.h, startGame, undefined, "start", "start", 1000, p.color(0, 150, 50), p.color(20, 200, 200), p.color(20, 150, 200))
    addButton(startButton)

    for(let row = 0; row < grid.rows; row++) {
      for(let column = 0; column < grid.columns; column++) {
        grid.set(column, row, 1, 1);
        goodTargets.push(new OneTimeButton(grid.x, grid.y, grid.w, grid.h, goodTargetHandler, hideButton, `goodButton_${column}-${row}`, "O", targetDwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
        badTargets.push(new OneTimeButton(grid.x, grid.y, grid.w, grid.h, badTargetHandler, hideButton, `badButton_${column}-${row}`, "X", targetDwellDelay, p.color(200, 50, 0), p.color(200, 200, 50), p.color(0, 200, 50)))
      }
    }

    for (const button of goodTargets) {
      addButton(button)
      button.hide()
    }
    for (const button of badTargets) {
      addButton(button)
      button.hide()
    }
  }

  function startGame(button) {
    console.log("Start!")
    started = true
    time = startTime
    score = 0
    spawnSpeed = startSpeed
    spawnTimer = 0
    spawnOffsetCorrection = 0
    buttonsHit = 0
    averageHitSpeed = -1 
    startButton.hide()

    if (typeof recorder !== 'undefined') {
      recorder.start(p)
      recorder.write(["Dwelldelay", targetDwellDelay])
    }
  }

  function mainLoop() {
    time -= dt/1000
    if(time <= 0) {
      time = 0
      endGame()
      return
    }
    topText = `Time: ${Math.ceil(time)}, Score: ${score}`

    let buttonsToHide = []
    for(const buttonName in activeButtonsTime) {
      activeButtonsTime[buttonName] += dt/1000
      if(activeButtonsTime[buttonName] >= buttonActiveTimeInitial*time/startTime + buttonActiveTimeFinal*(1-time/startTime)) {
        if(!getButton(buttonName).dwelled) {
          buttonsToHide.push(buttonName)
        }
      }
    }
    for(const buttonName of buttonsToHide) {
      getButton(buttonName).hide()
      delete activeButtonsTime[buttonName]
    }

    spawnSpeed += speedIncrement*(dt/1000)
    spawnTimer -= dt/1000
    if (spawnTimer <= 0) {
      spawnButton()
      let spawnOffset = Math.random() + 0.5 + spawnOffsetCorrection/2
      spawnOffsetCorrection += 1-spawnOffset
      spawnTimer = spawnOffset*(1/spawnSpeed)
      console.log(spawnSpeed, spawnOffset, spawnOffsetCorrection, spawnTimer)
    }
  }

  function spawnButton() {
    console.log(buttonActiveTimeInitial*time/startTime + buttonActiveTimeFinal*(1-time/startTime))
    let i = 0
    let column
    let row
    while(i == 0 || getButton(`goodButton_${column}-${row}`).visible == true || getButton(`badButton_${column}-${row}`).visible == true
    ) {
      if (i >= rows*columns) {
        return
      }
      i++
      column = Math.floor(Math.random()*columns)
      row = Math.floor(Math.random()*rows)
    }

    let badSpawnChance = Math.min((spawnSpeed-startSpeed)*badSpawnChanceIncrement, badSpawnChanceLimit)
    console.log(badSpawnChance)
    let buttonToUnhide
    if(Math.random() <= badSpawnChance) {
      buttonToUnhide = getButton(`badButton_${column}-${row}`)
    } else {
      buttonToUnhide = getButton(`goodButton_${column}-${row}`)
    }
    buttonToUnhide.unhide()
    activeButtonsTime[buttonToUnhide] = 0

    if (typeof recorder !== 'undefined') {
      recorder.write(["Button spawned", buttonToUnhide])
    }
  }


  function endGame() {
    console.log("Game Over!")
    started = false
    topText = `Game Over! Score: ${score}, average hit speed: ${roundTo(averageHitSpeed, 2)}`
    for (const button of goodTargets) {
      button.hide()
    }
    for (const button of badTargets) {
      button.hide()
    }
    // startButton.unhide()

    if (typeof recorder !== 'undefined') {
      recorder.write(["Game Over", buttonsHit, averageHitSpeed, score])
      recorder.stop()
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

  function goodTargetHandler(button) {
    score += 1
    buttonsHit += 1
    let hitSpeed = activeButtonsTime[button.name]
    averageHitSpeed = (averageHitSpeed*(buttonsHit-1)+hitSpeed)/buttonsHit

    button.reset()
    button.hide()
    delete activeButtonsTime[button.name]

    if (typeof recorder !== 'undefined') {
      recorder.write(["Good target hit", button, hitSpeed, score])
    }
  }

  function badTargetHandler(button) {
    score -= 3
    button.reset()
    button.hide()
    hitSpeed = activeButtonsTime[button.name]
    buttonsHit += 1

    if (typeof recorder !== 'undefined') {
      recorder.write(["Bad target hit!", button, score])
    }
  }

  function hideButton(button) {
    button.hide()
  }

  function addButton(button) {
    buttons.push(button)
    buttonDict[button.name] = button
    buttonDimentions[button.name] = {
      x: button.x / p.width,
      y: button.y / p.height,
      w: button.w / p.width,
      h: button.h / p.height
    }
  }

  function getButton(name) {
    return buttonDict[name]
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

  function roundTo(n, digits) {
    var negative = false;
    if (digits === undefined) {
        digits = 0;
    }
    if (n < 0) {
        negative = true;
        n = n * -1;
    }
    var multiplicator = Math.pow(10, digits);
    n = parseFloat((n * multiplicator).toFixed(11));
    n = (Math.round(n) / multiplicator).toFixed(digits);
    if (negative) {
        n = (n * -1).toFixed(digits);
    }
    return n;
  }
};

let myp5 = new p5(gui)