// Path Generation Configuration
const PATH_DEGREE = 6;  // Minimum of 4, recommended 6-8 for balance of accuracy/performance
const INTEGRATION_INTERVALS = 50;  // Number of intervals for path integral approximation

// Path Finding Parameters
const DISTANCE_WEIGHT = 50;  // Increased to prioritize shorter paths
const GRADIENT_STEP = 0.02;  // Reduced for more stable path optimization
const MOMENTUM = 0.3;        // Reduced to prevent overshooting
const GRADIENT_DELTA = 0.01;  // Step size for gradient approximation

// Path Finding State
let change = [];  // Vector for gradient approximation
let guess = [];   // Current path guess vector
let step = [];    // Change between consecutive guesses

// Obstacle Configuration
const OBSTACLE_HEIGHT = 5000;  // Maximum height of obstacles
const OBSTACLE_DECAY = 0.0025; // Spatial decay rate of obstacles

// Obstacle State
let obsX = [];  // Obstacle X coordinates
let obsY = [];  // Obstacle Y coordinates
let obsVX = []; // Obstacle X velocities
let obsVY = []; // Obstacle Y velocities

// Robot Parameters
const PATH_CHECKS = 100;  // Keep this value
const MAX_VELOCITY = 2.5;      // Increased from 2 for better speed
const MAX_ACCELERATION = 0.8;  // Increased from 0.5 for faster response
const MAX_ANGULAR_VEL = 0.15;   // Increased for faster rotation
const PATH_FOLLOW_CONST = 0.25; // Slightly increased for better path following
const LOOK_AHEAD = 3;        // Reduced from 4 to be less conservative

// Robot State
let botInfo = [200, 200, 0, 0, 0, 0];  // [x, y, vx, vy, angle, angular_vel]
let state = 0;   // Path endpoint selection state
let tempX = [];  // Temporary X coordinates for path planning
let tempY = [];  // Temporary Y coordinates for path planning

// Visualization Settings
const PATH_RESOLUTION = 25;  // Path drawing segments
const OBS_RESOLUTION = 5;    // Obstacle visualization resolution
let displayObs = 0;          // Toggle obstacle visualization

// Path State
let pathX = [200, 215, 200, 215, 0, 0];  // X path constraints
let pathY = [200, 200, 200, 200, 0, 0];  // Y path constraints
let followingPath = false;
const PATH_TOLERANCE = 8;    // Increased to prevent oscillation near target

// UI Constants
const PANEL_WIDTH = 200;
const PANEL_HEIGHT = 150;
const PANEL_MARGIN = 10;
const TEXT_SIZE = 14;

// Add new constants for kinematics
const MIN_VELOCITY = 0.05;   // Keep this value
const APPROACH_SPEED = 2;    // Increased from 1.5 for faster approach
const SLOW_DOWN_DIST = 150;  // Reduced from 200 for later deceleration
const TURN_RADIUS = 80;      // Reduced from 100 for tighter turns

// Add new UI constants
const COLORS = {
    BACKGROUND: '#ffffff',
    PANEL: '#ffffff',
    PANEL_BORDER: '#dddddd',
    TEXT_PRIMARY: '#333333',
    TEXT_SECONDARY: '#666666',
    PATH: '#ff4444',
    PATH_PREVIEW: '#ffaaaa',
    ROBOT: '#2196F3',
    ROBOT_DIRECTION: '#ff4444',
    VELOCITY_VECTOR: '#4CAF50',
    TARGET: '#4CAF50',
    TARGET_AREA: 'rgba(76, 175, 80, 0.1)',
    OBSTACLE: '#424242',
    GRID: '#e0e0e0'
};

const UI = {
    GRID_SIZE: 50,
    ROBOT_SIZE: 30,
    DIRECTION_LENGTH: 15,
    PANEL_PADDING: 15,
    PANEL_RADIUS: 8
};

/**
 * Calculates the obstacle field value at a given point
 */
function obsValue(x, y) {
    let output = 0;
    for (let i = 0; i < obsX.length; i++) {
        output += OBSTACLE_HEIGHT * exp(-OBSTACLE_DECAY * (sq(x - obsX[i]) + sq(y - obsY[i])));
    }
    return output;
}

/**
 * Adjusts the length of rotation direction indicators
 */
function correctLength(x1, y1, x2, y2) {
    const angle = atan2(y2 - y1, x2 - x1);
    return [
        x1 + cos(angle) * 15,
        y1 + sin(angle) * 15
    ];
}

// Convert V0 to V1
function v1(a) {
    let v = [];
    // Add constraint vector
    v[1] = pathX[0];
    v[2] = pathX[4];
    v[3] = -3 * pathX[0] + 3 * pathX[2] - 2 * pathX[4] - pathX[5];
    v[4] = 2 * pathX[0] - 2 * pathX[2] + pathX[4] + pathX[5];
    // Multiply by transformation matrix from R^2n-8 to R^2n
    for (let i = 1; i <= PATH_DEGREE - 4; i++) {
        v[3] += i * a[i];
    }
    for (let j = 1; j <= PATH_DEGREE - 4; j++) {
        v[4] += (-j - 1) * a[j];
    }
    for (let k = 5; k <= PATH_DEGREE; k++) {
        v[k] = a[k - 4];
    }
    return v;
}

// Convert V0 to V2
function v2(a) {
    let v = [];
    // Add constraint vector
    v[1] = pathY[0];
    v[2] = pathY[4];
    v[3] = -3 * pathY[0] + 3 * pathY[2] - 2 * pathY[4] - pathY[5];
    v[4] = 2 * pathY[0] - 2 * pathY[2] + pathY[4] + pathY[5];
    // Multiply by transformation matrix from R^2n-8 to R^2n
    for (let i = 1; i <= PATH_DEGREE - 4; i++) {
        v[3] += i * a[i + PATH_DEGREE - 4];
    }
    for (let j = 1; j <= PATH_DEGREE - 4; j++) {
        v[4] += (-j - 1) * a[j + PATH_DEGREE - 4];
    }
    for (let k = 5; k <= PATH_DEGREE; k++) {
        v[k] = a[k - 4 + PATH_DEGREE - 4];
    }
    return v;
}

// Outputs value of Pn at t
var PnTransform = function (v, t) {
  var output = 0;
  for (var i = 1; i <= PATH_DEGREE; i++) {
    output += v[i] * pow(t, i - 1);
  }
  return output;
};

// Outputs derivative of Pn at t
var PnDeriv = function (v, t) {
  var output = 0;
  for (var i = 1; i <= PATH_DEGREE - 1; i++) {
    output += i * v[i + 1] * pow(t, i - 1);
  }
  return output;
};

// Calculate quality of given path using trapezoidal approximation
function qual(v1, v2) {
    let output = 0;
    for (let i = 0; i < INTEGRATION_INTERVALS; i++) {
        output +=
            (obsValue(PnTransform(v1, i / INTEGRATION_INTERVALS), PnTransform(v2, i / INTEGRATION_INTERVALS)) + DISTANCE_WEIGHT) *
            mag(PnDeriv(v1, i / INTEGRATION_INTERVALS), PnDeriv(v2, i / INTEGRATION_INTERVALS)) +
            (obsValue(PnTransform(v1, (i + 1) / INTEGRATION_INTERVALS), PnTransform(v2, (i + 1) / INTEGRATION_INTERVALS)) + DISTANCE_WEIGHT) *
            mag(PnDeriv(v1, (i + 1) / INTEGRATION_INTERVALS), PnDeriv(v2, (i + 1) / INTEGRATION_INTERVALS));
    }
    output = output / (2 * INTEGRATION_INTERVALS);
    return output;
}

// Approximate gradient of quality function at V0
function grad(a) {
    let g = [];
    for (let i = 1; i <= 2 * PATH_DEGREE - 8; i++) {
        change[i] = a[i] + GRADIENT_DELTA;
        g[i] = (qual(v1(change), v2(change)) - qual(v1(a), v2(a))) / GRADIENT_DELTA;
        change[i] = a[i];
    }
    return g;
}

var anglePath = function (t) {
  var angle1 = atan2(pathY[1] - pathY[0], pathX[1] - pathX[0]);
  var angle2 = atan2(pathY[3] - pathY[2], pathX[3] - pathX[2]);
  return -2 * pow(t, 3) * (angle2 - angle1) + 3 * pow(t, 2) * (angle2 - angle1) + angle1;
}; //Calculate desired heading for all points on path

//Robot Simulation Functions

/**
 * Calculates the desired velocity at a point along the path
 */
function calculateDesiredVelocity(distToTarget) {
    // More sophisticated velocity profile
    const minSpeed = 0.5;  // Minimum speed when close to target
    const speedFactor = Math.min(1, Math.pow(distToTarget / SLOW_DOWN_DIST, 0.7));
    return Math.max(minSpeed, APPROACH_SPEED * speedFactor);
}

/**
 * Generates a kinematically feasible path to the target
 */
function generatePath(targetX, targetY, aimX, aimY) {
    const [x, y, vx, vy, angle, va] = botInfo;
    const currentSpeed = mag(vx, vy);
    
    // Calculate approach vector and distance
    const approachAngle = atan2(aimY - targetY, aimX - targetX);
    const distToTarget = dist(x, y, targetX, targetY);
    
    // Calculate desired approach speed based on distance
    const desiredSpeed = calculateDesiredVelocity(distToTarget);
    
    // Start position (with lookahead based on current velocity)
    tempX[0] = x + LOOK_AHEAD * vx;
    tempY[0] = y + LOOK_AHEAD * vy;
    
    // Initial heading based on current motion
    if (currentSpeed > MIN_VELOCITY) {
        // Use current velocity direction
        const velAngle = atan2(vy, vx);
        tempX[1] = tempX[0] + 15 * cos(velAngle);
        tempY[1] = tempY[0] + 15 * sin(velAngle);
    } else {
        // Use current robot orientation
        tempX[1] = tempX[0] + 15 * cos(angle);
        tempY[1] = tempY[0] + 15 * sin(angle);
    }
    
    // Target position
    tempX[2] = targetX;
    tempY[2] = targetY;
    
    // Final heading considering approach angle and turning radius
    const finalApproachDist = Math.max(20, desiredSpeed * 2);
    tempX[3] = targetX + cos(approachAngle) * finalApproachDist;
    tempY[3] = targetY + sin(approachAngle) * finalApproachDist;
    
    // Set initial and final velocities
    if (currentSpeed > MIN_VELOCITY) {
        // Maintain current velocity direction but scale magnitude
        const speedScale = desiredSpeed / currentSpeed;
        tempX[4] = vx * speedScale;
        tempY[4] = vy * speedScale;
    } else {
        // Set velocity based on path direction
        tempX[4] = desiredSpeed * cos(angle);
        tempY[4] = desiredSpeed * sin(angle);
    }
    
    // Target velocity (aligned with approach angle)
    const finalSpeed = Math.min(desiredSpeed, APPROACH_SPEED * 0.5);
    tempX[5] = finalSpeed * cos(approachAngle);
    tempY[5] = finalSpeed * sin(approachAngle);
    
    // Update path
    for (let i = 0; i < 6; i++) {
        pathX[i] = tempX[i];
        pathY[i] = tempY[i];
    }
    
    resetPath();
    followingPath = true;
}

/**
 * Updates robot motion considering kinematic constraints
 */
function updateBot(v1, v2, b) {
    let newInfo = [...b];

    if (!followingPath) {
        return newInfo;
    }

    // Update positions by velocities
    newInfo[0] += b[2];
    newInfo[1] += b[3];
    newInfo[4] += b[5];

    // Find point on path closest to robot
    let t = 0;
    let minDist = Infinity;
    for (let i = 1 / PATH_CHECKS; i <= 1; i += 1 / PATH_CHECKS) {
        const d = dist(b[0], b[1], PnTransform(v1, i), PnTransform(v2, i));
        if (d < minDist) {
            minDist = d;
            t = i;
        }
    }
    
    const d = minDist;
    
    // Look ahead on path based on current speed
    const lookAheadT = Math.min(1, t + (mag(b[2], b[3]) / MAX_VELOCITY) * 0.2); // Increased from 0.1
    
    // Calculate path tangent for desired heading
    const pathTangentX = PnDeriv(v1, lookAheadT);
    const pathTangentY = PnDeriv(v2, lookAheadT);
    const pathSpeed = mag(pathTangentX, pathTangentY);
    
    // Normalize and scale desired velocity
    const desiredSpeed = calculateDesiredVelocity(dist(b[0], b[1], pathX[2], pathY[2]));
    let vxIdeal = 0, vyIdeal = 0;
    
    if (pathSpeed > MIN_VELOCITY) {
        vxIdeal = (pathTangentX / pathSpeed) * desiredSpeed;
        vyIdeal = (pathTangentY / pathSpeed) * desiredSpeed;
    }
    
    // Add correction towards path with distance-based scaling
    const pathDist = dist(b[0], b[1], PnTransform(v1, lookAheadT), PnTransform(v2, lookAheadT));
    const correctionScale = Math.min(1, pathDist / 50); // Scale correction based on distance from path
    vxIdeal += PATH_FOLLOW_CONST * correctionScale * (PnTransform(v1, lookAheadT) - b[0]);
    vyIdeal += PATH_FOLLOW_CONST * correctionScale * (PnTransform(v2, lookAheadT) - b[1]);
    
    // Scale down velocity if it exceeds maximum
    const currentSpeed = mag(vxIdeal, vyIdeal);
    if (currentSpeed > MAX_VELOCITY) {
        vxIdeal = (vxIdeal / currentSpeed) * MAX_VELOCITY;
        vyIdeal = (vyIdeal / currentSpeed) * MAX_VELOCITY;
    }

    // Calculate ideal accelerations with smoother transitions
    let axIdeal = (vxIdeal - b[2]) * 0.8; // Added damping factor
    let ayIdeal = (vyIdeal - b[3]) * 0.8;

    // Update velocities
    newInfo[2] += axIdeal;
    newInfo[3] += ayIdeal;

    // Update and limit angular velocity with improved PID
    const targetAngle = anglePath(t + 1 / PATH_CHECKS);
    const angleError = targetAngle - b[4];
    // Normalize angle error to [-PI, PI]
    const normalizedError = atan2(sin(angleError), cos(angleError));
    
    // PID-like control for angular velocity
    const Kp = 0.3;  // Proportional gain
    const Kd = 0.1;  // Derivative gain for damping
    newInfo[5] = Kp * normalizedError - Kd * b[5];  // Include current angular velocity for damping
    
    // Limit angular velocity
    newInfo[5] = constrain(newInfo[5], -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

    // Check if we've reached the target with adjusted tolerances
    if (
        abs(newInfo[0] - pathX[2]) < PATH_TOLERANCE &&
        abs(newInfo[1] - pathY[2]) < PATH_TOLERANCE &&
        abs(normalizedError) < PATH_TOLERANCE / 20  // Tighter angle tolerance
    ) {
        followingPath = false;
    }
    
    return newInfo;
}

function updatePath() {
    for (let i = 1; i <= 2 * PATH_DEGREE - 8; i++) {
        step[i] = MOMENTUM * step[i] + grad(guess)[i];
        guess[i] -= GRADIENT_STEP * step[i];
    }
}

var updateObs = function () {
  // console.log(obsX);

  for (var i = 0; i < obsX.length; i++) {
    obsX[i] += obsVX[i];
    obsY[i] += obsVY[i];
  }
};

//Visualization Functions
var drawObs = function () {
  if (displayObs === 1) {
    // Display obstacles as decaying circles
    strokeWeight(OBS_RESOLUTION);
    // noprotect
    for (let i = 0; i < width / OBS_RESOLUTION; i++) {
      for (let j = 0; j < height / OBS_RESOLUTION; j++) {
        stroke(255 * (1 - obsValue(i * OBS_RESOLUTION, j * OBS_RESOLUTION) / OBSTACLE_HEIGHT), 255, 255);
        point(i * OBS_RESOLUTION, j * OBS_RESOLUTION);
      }
    }
  } else {
    // Display obstacles as points
    strokeWeight(5);
    stroke(0);
    for (let k = 0; k < obsX.length; k++) {
      point(obsX[k], obsY[k]);
    }
  }
}; //Draw obstacles

var drawPath = function (v1, v2) {
  if (!followingPath) {
    return;
  }
  // Draw path
  stroke(255, 0, 0);
  strokeWeight(2);
  for (let i = 0; i < PATH_RESOLUTION; i++) {
    line(
      PnTransform(v1, i / PATH_RESOLUTION),
      PnTransform(v2, i / PATH_RESOLUTION),
      PnTransform(v1, (i + 1) / PATH_RESOLUTION),
      PnTransform(v2, (i + 1) / PATH_RESOLUTION)
    );
  }
  //Draw endpoints and shorten direction indicators if ther are too long
  stroke(0, 255, 0);
  strokeWeight(2);
  if (dist(pathX[0], pathY[0], pathX[1], pathY[1]) !== 15) {
    pathX[1] = correctLength(pathX[0], pathY[0], pathX[1], pathY[1])[0];
    pathY[1] = correctLength(pathX[0], pathY[0], pathX[1], pathY[1])[1];
  }
  if (dist(pathX[2], pathY[2], pathX[3], pathY[3]) !== 15) {
    pathX[3] = correctLength(pathX[2], pathY[2], pathX[3], pathY[3])[0];
    pathY[3] = correctLength(pathX[2], pathY[2], pathX[3], pathY[3])[1];
  }
  line(pathX[0], pathY[0], pathX[1], pathY[1]);
  line(pathX[2], pathY[2], pathX[3], pathY[3]);
  stroke(0, 0, 255);
  strokeWeight(5);
  point(pathX[0], pathY[0]);
  point(pathX[2], pathY[2]);
  grad(guess);
}; //Draw current path

/**
 * Draws the robot at its current position
 */
function drawBot(botInfo) {
    const [x, y, vx, vy, angle, va] = botInfo;
    const robotSize = 30;
    
    // Velocity vector
    stroke(0, 255, 0);
    strokeWeight(2);
    line(x, y, x + vx * 10, y + vy * 10);
    
    // Robot body
    stroke(0);
    strokeWeight(2);
    noFill();
    
    push();
    translate(x, y);
    rotate(angle);
    
    rect(-robotSize/2, -robotSize/2, robotSize, robotSize);
    
    // Direction indicator
    stroke(255, 0, 0);
    line(0, 0, robotSize, 0);
    
    pop();
}

// Visualization Control Functions

// Create new paths and obstacles
function mousePressed() {
  if (mouseX < 0 || mouseY < 0 || mouseX > 600 || mouseY > 600) {
    return;
  }
  if (keyCode === SHIFT && keyIsPressed) {
    // Add an obstacle at mouse position
    obsVX.push(0);
    obsVY.push(0);
    obsX.push(mouseX);
    obsY.push(mouseY);
  } else if (state === 0) {
    // Establish new end point position
    tempX[2] = mouseX;
    tempY[2] = mouseY;
    state = 1;
  } else if (state === 1) {
    // Establish new end point direction
    tempX[3] = mouseX;
    tempY[3] = mouseY;
    // Establish remaining initial path variables using current state of the robot
    tempX[0] = botInfo[0] + LOOK_AHEAD * botInfo[2];
    tempY[0] = botInfo[1] + LOOK_AHEAD * botInfo[3];
    tempX[1] = botInfo[0] + LOOK_AHEAD * botInfo[2] + cos(botInfo[4] + LOOK_AHEAD * botInfo[5]);
    tempY[1] = botInfo[1] + LOOK_AHEAD * botInfo[3] + sin(botInfo[4] + LOOK_AHEAD * botInfo[5]);
    tempX[4] = botInfo[2];
    tempY[4] = botInfo[3];
    tempX[5] = 0;
    tempY[5] = 0;
    // Create new path
    for (let i = 0; i < 6; i++) {
      pathX[i] = tempX[i];
      pathY[i] = tempY[i];
    }
    resetPath();
    followingPath = true;
    state = 0;
  }
}

function keyTyped() {
  if (key === "o") {
    if (displayObs === 0) {
      displayObs = 1;
    } else {
      displayObs = 0;
    }
  }
}

function goToTarget() {
  generatePath(300, 575, 300, 1000);
}

function moveBot() {
  var vel = 3;
  if (keyIsDown(16)) {
    // Shift
    vel = 7;
  }
  if (keyIsDown(LEFT_ARROW) === true) {
    botInfo[0] -= vel;
    botInfo[2] = -vel;
  }
  if (keyIsDown(RIGHT_ARROW) === true) {
    botInfo[0] += vel;
    botInfo[2] = vel;
  }
  if (keyIsDown(UP_ARROW) === true) {
    botInfo[1] -= vel;
    botInfo[3] = -vel;
  }
  if (keyIsDown(DOWN_ARROW) === true) {
    botInfo[1] += vel;
    botInfo[3] = vel;
  }
  if (keyIsDown(65) === true) {
    botInfo[4] -= 0.1;
    botInfo[5] = -0.1;
  }
  if (keyIsDown(68) === true) {
    botInfo[4] += 0.1;
    botInfo[5] = 0.1;
  }
  if (keyIsDown(32)) {
    goToTarget();
  }
}

function drawTarget() {
  stroke(200, 200, 200);
  strokeWeight(1);
  // rectangle centered at 300, 575
  line(275, 550, 325, 550);
  line(275, 550, 275, 600);
  line(325, 550, 325, 600);
  line(275, 600, 325, 600);
}

var resetPath = function () {
  for (var i = 1; i <= 2 * PATH_DEGREE - 8; i++) {
    guess[i] = 0;
    step[i] = 0;
  }
}; //Resets guess to initial state

//Run Program
function setup() {
  createCanvas(600, 600);
  resetPath();
}

function draw() {
  background(COLORS.BACKGROUND);

  drawTarget();

  moveBot();

  //Draw Obstacles
  drawObs();
  //Draw and Update Path
  if (followingPath) {
    drawPath(v1(guess), v2(guess));
    updatePath();
  }
  //Draw and Update Robot
  drawBot(botInfo);
  botInfo = updateBot(v1(guess), v2(guess), botInfo);
  updateObs();

  // Draw status panel on top
  drawStatusPanel();

  noStroke();
  fill(0);
}

/**
 * Draws the status panel with robot information
 */
function drawStatusPanel() {
    // Panel background with shadow effect
    fill(COLORS.PANEL);
    stroke(COLORS.PANEL_BORDER);
    strokeWeight(1);
    rect(
        width - PANEL_WIDTH - PANEL_MARGIN,
        PANEL_MARGIN,
        PANEL_WIDTH,
        PANEL_HEIGHT,
        UI.PANEL_RADIUS
    );
    
    // Content
    const x = width - PANEL_WIDTH + UI.PANEL_PADDING;
    const y = PANEL_MARGIN + UI.PANEL_PADDING;
    const lineHeight = TEXT_SIZE * 1.5;
    
    // Status section
    noStroke();
    fill(COLORS.TEXT_PRIMARY);
    textSize(TEXT_SIZE);
    textAlign(LEFT);
    textStyle(BOLD);
    text('Status', x, y + lineHeight * 1);
    
    textStyle(NORMAL);
    fill(COLORS.TEXT_SECONDARY);
    text(`Position: (${round(botInfo[0])}, ${round(botInfo[1])})`, x, y + lineHeight * 2);
    text(`Speed: ${round(mag(botInfo[2], botInfo[3]), 1)}`, x, y + lineHeight * 3);
    text(`Heading: ${round(degrees(botInfo[4]))}°`, x, y + lineHeight * 4);
    
    // Path status with color indicator
    const pathStatus = followingPath ? '● Following' : '○ Idle';
    fill(followingPath ? COLORS.PATH : COLORS.TEXT_SECONDARY);
    text(pathStatus, x, y + lineHeight * 5);
}

/**
 * Draws a grid background
 */
function drawGrid() {
    stroke(COLORS.GRID);
    strokeWeight(1);
    
    // Draw vertical lines
    for (let x = 0; x < width; x += UI.GRID_SIZE) {
        line(x, 0, x, height);
    }
    
    // Draw horizontal lines
    for (let y = 0; y < height; y += UI.GRID_SIZE) {
        line(0, y, width, y);
    }
}

/**
 * Draws the target area with improved visuals
 */
function drawTarget() {
    // Draw target area with fill
    noStroke();
    fill(COLORS.TARGET_AREA);
    rect(275, 550, 50, 50);
    
    // Draw target border
    stroke(COLORS.TARGET);
    strokeWeight(2);
    rect(275, 550, 50, 50);
    
    // Draw target cross
    const center = { x: 300, y: 575 };
    const size = 10;
    line(center.x - size, center.y, center.x + size, center.y);
    line(center.x, center.y - size, center.x, center.y + size);
}

/**
 * Draws the robot with improved visuals
 */
function drawBot(botInfo) {
    const [x, y, vx, vy, angle, va] = botInfo;
    
    // Velocity vector
    if (mag(vx, vy) > 0.1) {
        stroke(COLORS.VELOCITY_VECTOR);
        strokeWeight(2);
        drawArrow(x, y, x + vx * 10, y + vy * 10);
    }
    
    push();
    translate(x, y);
    rotate(angle);
    
    // Robot body
    stroke(COLORS.ROBOT);
    strokeWeight(2);
    fill(COLORS.PANEL);
    rect(-UI.ROBOT_SIZE/2, -UI.ROBOT_SIZE/2, UI.ROBOT_SIZE, UI.ROBOT_SIZE, 5);
    
    // Direction indicator
    stroke(COLORS.ROBOT_DIRECTION);
    strokeWeight(2);
    drawArrow(0, 0, UI.ROBOT_SIZE, 0);
    
    pop();
}

/**
 * Draws an arrow from (x1,y1) to (x2,y2)
 */
function drawArrow(x1, y1, x2, y2) {
    const angle = atan2(y2 - y1, x2 - x1);
    const length = dist(x1, y1, x2, y2);
    
    line(x1, y1, x2, y2);
    
    push();
    translate(x2, y2);
    rotate(angle);
    const arrowSize = 6;
    line(0, 0, -arrowSize, -arrowSize);
    line(0, 0, -arrowSize, arrowSize);
    pop();
}

/**
 * Draws the path with improved visuals
 */
function drawPath(v1, v2) {
    if (!followingPath) return;
    
    // Draw path
    stroke(COLORS.PATH);
    strokeWeight(2);
    noFill();
    
    beginShape();
    for (let i = 0; i <= PATH_RESOLUTION; i++) {
        const t = i / PATH_RESOLUTION;
        vertex(PnTransform(v1, t), PnTransform(v2, t));
    }
    endShape();
    
    // Draw endpoints
    stroke(COLORS.PATH);
    strokeWeight(2);
    
    // Draw direction indicators
    drawArrow(pathX[0], pathY[0], pathX[1], pathY[1]);
    drawArrow(pathX[2], pathY[2], pathX[3], pathY[3]);
    
    // Draw endpoint markers
    fill(COLORS.PATH);
    noStroke();
    circle(pathX[0], pathY[0], 8);
    circle(pathX[2], pathY[2], 8);
}
