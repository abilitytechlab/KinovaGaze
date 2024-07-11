/** Object to communicate with the locally running ROS through rosbridge*/
RosComm = class {
    /**
     * @constructor
     * @param {string} url - url of rosbridge
     * @param {() => void} [onConnectionCallback=this.onConnection] - (optoinal) Callback method when connection is established
     * @param {() => void} [onErrorCallback=this.onError] - (optoinal) Callback method on connection error
     * @param {() => void} [onCloseCallback=this.onClose] - (optoinal) Callback method on connection closed
     */
    constructor(url, onConnectionCallback = this.onConnection, onErrorCallback = this.onError, onCloseCallback = this.onClose) {
        this.ros = new ROSLIB.Ros({
            url: url 
        });

         // Make and call a function to update the ROS status depending on the last update
        this.ros.on('connection', onConnectionCallback);

        this.ros.on('error', onErrorCallback);

        this.ros.on('close', onCloseCallback);

        this.finger_position_set_publisher = new ROSLIB.Topic({
            ros: this.ros,
            name: "/kinovagaze/finger_position_set",
            messageType: 'kinova_msgs/FingerPosition'
        });

        this.return_home_publisher = new ROSLIB.Topic({
            ros: this.ros,
            name: "/kinovagaze/return_home",
            messageType: 'std_msgs/Empty'
        });

        this.get_tool_pose_publisher = new ROSLIB.Topic({
            ros: this.ros,
            name: "/kinovagaze/get_tool_pose",
            messageType: 'std_msgs/Empty'
        });

        this.stop_movement_publisher = new ROSLIB.Topic({
            ros: this.ros,
            name: "/kinovagaze/stop_movement",
            messageType: 'std_msgs/Empty'
        });

        this.tool_move_absolute_publisher = new ROSLIB.Topic({
            ros: this.ros,
            name: "/kinovagaze/tool_move_absolute",
            messageType: "kinova_msgs/KinovaPose"
        });

        this.tool_move_relative_publisher = new ROSLIB.Topic({
            ros: this.ros,
            name: "/kinovagaze/tool_move_relative",
            messageType: "kinova_msgs/KinovaPose"
        });

        this.tool_move_continuous_publisher = new ROSLIB.Topic({
            ros: this.ros,
            name: "/kinovagaze/tool_move_continuous",
            messageType: 'kinova_msgs/PoseVelocity'
        });

        this.timeout_publisher = new ROSLIB.Topic({
            ros: this.ros,
            name: "/kinovagaze/timeout",
            messageType: 'std_msgs/Int32'
        })

        this.msg_send_publisher = new ROSLIB.Topic({
            ros: ros,
            name: "kinovagaze/msg_send",
            messageType: 'std_msgs/String'
        });
    }

    /**
     * Default callback for connection established. Can be overwritten
     */
    onConnection() {

    }

    /**
     * Default callback for connection error. Can be overwritten
     */
    onError() {

    }
    /**
     * Default callback for connection closed. Can be overwritten
     */
    onClose() {

    }

    /**
     * Send a message to the node
     * @param {string} text - message to send
     */
    msg_send = function (text) {
        let string = new ROSLIB.Message({
            data: text
        });
        msg_send_publisher.publish(string);
    }

    /**
     * Set the timeout variable in the ROS node. 
     * Callback variable always counts down and stops movement if it reaches 0. Should either be constantly set or set for every movement. 
     * Movement commands increase it to at least 1000 ms per default.
     * @param {number} timeout - Timeout in ms
     */
    setTimeout = function (timeout) {
        let timeoutMsg = new ROSLIB.Message({
            data: timeout
        })
        this.timeout_publisher.publish(timeoutMsg)
    }
    /**
     * Sets the state of each finger, with 0 being fully open and 100 being fully closed.
     * @param {number} finger1 - State of finger 1. Between 0 and 100.
     * @param {number} finger2 - State of finger 2. Between 0 and 100.
     * @param {number} finger3 - State of finger 3. Between 0 and 100.
     */
    fingerPositionSet = function (finger1, finger2, finger3) {
        let finger_position = new ROSLIB.Message({
            finger1: finger1,
            finger2: finger2,
            finger3: finger3
        });
        this.finger_position_set_publisher.publish(finger_position);
    }

    /**
     * Fully close all fingers.
     */
    closeFingers() {
        this.fingerPositionSet(100, 100, 100);
    }
    
    /**
     * Fully open all fingers.
     */
    openFingers() {
        this.fingerPositionSet(0, 0, 0);
    }
    
    /**
     * Home the arm. Cannot be manually stopped.
     */
    returnHome = function () {
        this.return_home_publisher.publish(new ROSLIB.Message());
    }

    /**
     * Causes the node to print the current tool pose to console.
     */
    getToolPose = function () {
        this.get_tool_pose_publisher.publish(new ROSLIB.Message());
    }

    /**
     * Stops arm movement except for homing.
     */
    stopMovement = function () {
        this.stop_movement_publisher.publish(new ROSLIB.Message());
    }

    /**
     * Move the tool to an absolute position.
     * @param {*} x - x target in meters
     * @param {*} y - y target in meters
     * @param {*} z - z target in meters
     * @param {*} yaw - yaw target in radians
     * @param {*} pitch - pitch target in radians
     * @param {*} roll - roll target in radians
     */
    toolMoveAbsolute = function (x = 0, y = 0, z = 0, yaw = 0, pitch = 0, roll = 0) {
        console.log(x, y, z, yaw, pitch, roll);
        let position_axis = new ROSLIB.Message({
            X: x,
            Y: y,
            Z: z,
            ThetaX: yaw,
            ThetaY: pitch,
            ThetaZ: roll
        });
        this.tool_move_absolute_publisher.publish(position_axis);
    }

    /**
     * Move to default forward position.
     */
    positionForward = function () {
        this.toolMoveAbsolute(0.3, -0.3, 0.2, 90, 0, 0);
    }

    /**
     * Move the tool to a relative position. 
     * Relative to the tool position and orientation. Orientation is not affected by tool roll state.
     * @param {*} x - relative x target in meters
     * @param {*} y - relative y target in meters
     * @param {*} z - relative z target in meters
     * @param {*} yaw - relative yaw target in radians
     * @param {*} pitch - relative pitch target in radians
     * @param {*} roll - relative roll target in radians
     */
    toolMoveRelative = function (x = 0, y = 0, z = 0, yaw = 0, pitch = 0, roll = 0) {
        console.log(x, y, z, yaw, pitch, roll);
        let position_axis = new ROSLIB.Message({
            X: x,
            Y: y,
            Z: z,
            ThetaX: yaw,
            ThetaY: pitch,
            ThetaZ: roll
        });
        this.tool_move_relative_publisher.publish(position_axis);
    }

    /**
     * Move the tool continuously.
     * Relative to the tool position and orientation. Orientation is not affected by tool roll state.
     * @param {*} x - x axis movement between -1 and 1
     * @param {*} y - y axis movement between -1 and 1
     * @param {*} z - z axis movement between -1 and 1
     * @param {*} yaw - yaw axis movement between -1 and 1
     * @param {*} pitch - pitch axis movement between -1 and 1
     * @param {*} roll - roll axis movement between -1 and 1
     */
    toolMoveContinuous = function (x = 0, y = 0, z = 0, yaw = 0, pitch = 0, roll = 0) {
        console.log(x, y, z, yaw, pitch, roll)
        let move_axis = new ROSLIB.Message({
            twist_linear_x: x,
            twist_linear_y: y,
            twist_linear_z: z,
            twist_angular_x: yaw,
            twist_angular_y: pitch,
            twist_angular_z: roll
        });
        console.log(move_axis);
        this.tool_move_continuous_publisher.publish(move_axis);
    }
    
    left = function () {
        this.toolMoveContinuous(1, 0, 0, 0, 0, 0);
    }

    right = function () {
        this.toolMoveContinuous(-1, 0, 0, 0, 0, 0);
    }

    forward = function () {
        this.toolMoveContinuous(0, -1, 0, 0, 0, 0);
    }

    back = function () {
        this.toolMoveContinuous(0, 1, 0, 0, 0, 0);
    }

    up = function () {
        this.toolMoveContinuous(0, 0, 1, 0, 0, 0);
    }

    down = function () {
        this.toolMoveContinuous(0, 0, -1, 0, 0, 0);
    }

    pitchUp = function () {
        this.toolMoveContinuous(0, 0, 0, -1, 0, 0);
    }

    pitchDown = function () {
        this.toolMoveContinuous(0, 0, 0, 1, 0, 0);
    }

    rollClockwise = function () {
        this.toolMoveContinuous(0, 0, 0, 0, 0, 1);
    }

    rollCounterClockwise = function () {
        this.toolMoveContinuous(0, 0, 0, 0, 0, -1);
    }

    yawLeft = function () {
        this.toolMoveContinuous(0, 0, 0, 0, 1, 0);
    }

    yawRight = function () {
        this.toolMoveContinuous(0, 0, 0, 0, -1, 0);
    }
}