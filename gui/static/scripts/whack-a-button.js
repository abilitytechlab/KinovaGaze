"use strict";

/**
 * Whack-A-Button game. A simple game showcasing the gazecontrol system.
 * 
 * @param {object} p - p5.js instance
 */
const gui = ( p ) => {
  // Game settings
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

  // GazeControl variables (TODO: Move this functionality into a single GazeControl object)
  let cursorObject
  const buttons = [] // Array of buttons to iterate over
  const buttonDict = {} // Dictionary of buttons to retreive a button by name
  
  // Variables to access buttons directly by type
  const badTargets = []
  const goodTargets = []
  let startButton

  // Game state
  let started = false
  let time = 0
  let score = 0
  let spawnSpeed = 0.5
  let spawnTimer = 1
  let spawnOffsetCorrection = 0
  let buttonsHit = 0
  let averageHitSpeed = -1
  let topText = "Press start to play!"
  let activeButtons = [] // Array storing which buttons are currently spawned

  // Helper variables
  let canvas
  let scaleFactor = 1
  let defaultTextSize = 32
  let dt
  let lastTime

  /**
   * Initial method called by p5.js. Sets up game including canvas, buttons, cursor and recorder.
   */
  p.setup = () => {
    canvas = p.createCanvas(640, 480) // Any reasonable size works
    updateCanvasDimensions()

    lastTime = Date.now()

    createButtons()
    cursorObject = new GazeControl.Cursor(p, buttons, canvas)

    // Setup recorder if included in the HTML file
    if (typeof recorder !== 'undefined') {
      recorder.initialize(canvas)
    }
  }

  /**
   * Method calls by p5.js every frame after setup. Runs the main game loop.
   */
  p.draw = () => {
    p.textSize(defaultTextSize)

    dt = Date.now() - lastTime
    lastTime = Date.now()
    
    if(started) {
      mainLoop()
    }

    p.background(220);

    for (const button of buttons) {
      button.update(dt) // Update the state of the buttons
      button.display(p) // Display the buttons. Can be called later in the code to change their display priority.
    }

    cursorObject.display(p)
    
    // Draw top bar
    p.textAlign(p.CENTER, p.CENTER) // Alligns all text to the center
    p.fill(0) // Sets fill color to black
    let grid = new GazeControl.Grid(0, 0, p.width, p.height*1/6, 2, 2) // Create a grid covering the full width of the top 1/6th of the screen with 2 columns and 2 rows.
    grid.set(grid.columns/2, 0.5) // Sets the current grid position to the middle column seperator and the center of the first row
    p.text("Whack-A-Button: Whack the yellow buttons, not the red ones!", grid.x, grid.y) // Draw text centered on the current grid position
    grid.set(grid.columns/2, 1.5) // Sets the current grid position to the middle column seperator and the center of the second row
    p.text(topText, grid.x, grid.y) // Draw text centered on the current grid position

    // Display recorder status if included in the HTML file
    if (typeof recorder !== 'undefined') {
      p.textSize(defaultTextSize / 2)
      recorder.display(p)
    }
  }

  /**
   * Create the buttons used in the game. Creates the start button as well as a grid with both a good and a bad button on each position.
   */
  function createButtons() {
    let grid = new GazeControl.Grid(0, p.height * 1/6, p.width, p.height * 5 / 6, columns, rows) // Create a grid covering the full width of the button 5/6th of the screen with a number of rows and columns as defined in the game settings
    grid.set(grid.columns/2-1.5, grid.rows/2-1, 3, 1) // Set the grid to an object 3 columns wide and 1 row high, horizontally centered and vertically centered but moved up by 1 row
    startButton = new OneTimeButton(grid.x, grid.y, grid.w, grid.h, startGame, "start", "start", 1000, p.color(0, 150, 50), p.color(20, 200, 200), p.color(20, 150, 200)) //Create the start button at the current grid settings with a green color scheme which calls the startGame method when activated
    addButton(startButton) // Add the start button to the list of buttons needed to be updated

    // Iterate through every row and column, creating a good and a bad button on each cell. The button names include the column and row to keep them unique and so this information can later be extracted.
    for(let row = 0; row < grid.rows; row++) {
      for(let column = 0; column < grid.columns; column++) {
        grid.set(column, row, 1, 1);
        goodTargets.push(new TargetButton(grid.x, grid.y, grid.w, grid.h, goodTargetHandler, `goodButton_${column}-${row}`, "O", targetDwellDelay, p.color(150, 110, 10), p.color(255, 230, 0), p.color(222, 252, 55)))
        badTargets.push(new TargetButton(grid.x, grid.y, grid.w, grid.h, badTargetHandler, `badButton_${column}-${row}`, "X", targetDwellDelay, p.color(200, 50, 0), p.color(200, 200, 50), p.color(0, 200, 50)))
      }
    }

    // Add the good and bad buttons to the list of buttons needed to be updated
    for (const button of goodTargets) {
      addButton(button)
      button.hide()
    }
    for (const button of badTargets) {
      addButton(button)
      button.hide()
    }
  }
  
  /**
   * Start the game. Initialises the game state and hides the start button.
   */
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

    // Start the recorder if included in the HTML file
    if (typeof recorder !== 'undefined') {
      recorder.start(p)
      recorder.write(["Dwelldelay", targetDwellDelay])
    }
  }

  /**
   * Main game loop. Keeps track of the game timer, spawns buttons and increments game difficulty.
   * @returns 
   */
  function mainLoop() {
    // Update timer, end game if timer reaches 0
    time -= dt/1000
    if(time <= 0) {
      time = 0
      endGame()
      return
    }
    topText = `Time: ${Math.ceil(time)}, Score: ${score}`

    // Despawn buttons which have not been whacked in time
    let buttonsToHide = []
    for(const button of activeButtons) {
      button.spawnedTime += dt/1000
      if(button.spawnedTime >= buttonActiveTimeInitial*time/startTime + buttonActiveTimeFinal*(1-time/startTime)) {
        if(!button.dwelled) { // Ensure the button doesn't despawn if it is currently hovered
          buttonsToHide.push(button) // Don't remove the button from the list we're currently iterating over, remove it afterwards
        }
      }
    }
    for(const buttonToHide of buttonsToHide) {
      buttonToHide.hide()
      activeButtons = activeButtons.filter(button => button !== buttonToHide)
    }

    // Incremeant spawn speed and decrement spawn timer, spawn button if spawn timer reacher 0.
    spawnSpeed += speedIncrement*(dt/1000)
    spawnTimer -= dt/1000
    if (spawnTimer <= 0) {
      spawnButton()
      let spawnOffset = Math.random() + 0.5 + spawnOffsetCorrection/2
      spawnOffsetCorrection += 1-spawnOffset // Mechanism to decrease the effect of luck on the total number of buttons spawned.
      spawnTimer = spawnOffset*(1/spawnSpeed)
      console.log(spawnSpeed, spawnOffset, spawnOffsetCorrection, spawnTimer)
    }
  }

  /**
   * Spawn a button in a location where there is not yet a button spawned, governed by a calculation deciding the chance of whether the button should be good or bad.
   */
  function spawnButton() {
    console.log(buttonActiveTimeInitial*time/startTime + buttonActiveTimeFinal*(1-time/startTime))

    // Find a grid location where no button is currently spawned
    let i = 0
    let column
    let row
    while(i == 0 || getButton(`goodButton_${column}-${row}`).visible == true || getButton(`badButton_${column}-${row}`).visible == true) {
      if (i >= rows*columns) { // Exit if no space is available
        return
      }
      i++
      column = Math.floor(Math.random()*columns)
      row = Math.floor(Math.random()*rows)
    }

    let badSpawnChance = Math.min((spawnSpeed-startSpeed)*badSpawnChanceIncrement, badSpawnChanceLimit) // Determine the current chance of a bad button spawning
    let buttonToUnhide
    if(Math.random() <= badSpawnChance) {
      buttonToUnhide = getButton(`badButton_${column}-${row}`)
    } else {
      buttonToUnhide = getButton(`goodButton_${column}-${row}`)
    }
    buttonToUnhide.unhide()
    activeButtons.push(buttonToUnhide)
    buttonToUnhide.spawnedTime = 0

    if (typeof recorder !== 'undefined') {
      recorder.write(["Button spawned", buttonToUnhide])
    }
  }

  /**
   * End the game, hiding all target buttons and displaying the results.
   */
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
    startButton.unhide()

    if (typeof recorder !== 'undefined') {
      recorder.write(["Game Over", buttonsHit, averageHitSpeed, score])
      recorder.stop()
    }
  }

  /**
   * A simple button with a callback on activation and automatic resize functionality.
   */
  class OneTimeButton extends GazeControl.Button {
    constructor(x, y, w, h, activateCallback, name = undefined, label = undefined, dwellDelay = 1000, defaultColor = p.color(200, 50, 0), hoverColor = p.color(200, 200, 50), activatedColor = p.color(0, 200, 50)) {
      super(p, x, y, w, h, name, label, dwellDelay, defaultColor, hoverColor, activatedColor)
      this.activateCallback = activateCallback
      
      this.dimentions = { // Store the current location and size relative to the canvas height
        x: x / p.width,
        y: y / p.height,
        w: w / p.width,
        h: h / p.height
      }
    }

    onActivate() {
      if (typeof this.activateCallback !== 'undefined') {
        this.activateCallback(this)
      }
    }

    updateDimensions() {
      this.x = this.dimentions.x * p.width
      this.y = this.dimentions.y * p.height
      this.w = this.dimentions.w * p.width
      this.h = this.dimentions.h * p.height
    }
  }
  
  /**
   * A OneTimeButton which also stores a spawned time variable.
   */
  class TargetButton extends OneTimeButton {
    spawnedTime = 0

    constructor(x, y, w, h, activateCallback, name = undefined, label = undefined, dwellDelay = 1000, defaultColor = p.color(200, 50, 0), hoverColor = p.color(200, 200, 50), activatedColor = p.color(0, 200, 50)) {
      super(x, y, w, h, activateCallback, name, label, dwellDelay, defaultColor, hoverColor, activatedColor)
    }
  }

  /**
   * Callback for activation of a good button. Increases the score, stores statistics and hides the button.
   * @param {object} button - The button which was hit
   */
  function goodTargetHandler(button) {
    score += 1
    buttonsHit += 1
    let hitSpeed = button.spawnedTime
    averageHitSpeed = (averageHitSpeed*(buttonsHit-1)+hitSpeed)/buttonsHit

    button.reset()
    button.hide()
    activeButtons = activeButtons.filter(activeButton => activeButton !== button)

    if (typeof recorder !== 'undefined') {
      recorder.write(["Good target hit", button, hitSpeed, score])
    }
  }

  /**
   * Callback for activation of a bad button. Decreases the score, stores statistics and hides the button.
   * @param {object} button - The button which was hit
   */
  function badTargetHandler(button) {
    score -= 3
    buttonsHit += 1
    let hitSpeed = button.spawnedTime
    averageHitSpeed = (averageHitSpeed*(buttonsHit-1)+hitSpeed)/buttonsHit

    button.reset()
    button.hide()
    activeButtons = activeButtons.filter(activeButton => activeButton !== button)

    if (typeof recorder !== 'undefined') {
      recorder.write(["Bad target hit", button, hitSpeed, score])
    }
  }

  /**
   * Adds a button to the buttons array and dictionary
   * @param {object} button - button to add
   */
  function addButton(button) {
    buttons.push(button)
    buttonDict[button.name] = button
  }

  /**
   * Gets a button by its name
   * @param {string} name - name of button to get
   * @returns - button with name equal to input
   */
  function getButton(name) {
    return buttonDict[name]
  }

  /**
   * Updates the location size of all buttons
   */
  function updateButtonDimensions() {
    for (const button of buttons) {
      button.updateDimensions()
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

    canvasWidth = p.windowWidth
    canvasHeight = p.windowHeight
    p.resizeCanvas(canvasWidth, canvasHeight)

    scaleFactor = 640 / canvasWidth

    const x = (p.windowWidth - canvasWidth) / 2
    const y = (p.windowHeight - canvasHeight) / 2
    canvas.position(x, y)

    p.pixelDensity(window.devicePixelRatio)
    p.strokeWeight(2 * scaleFactor)
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