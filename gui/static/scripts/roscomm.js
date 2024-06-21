RosComm = class {
    // Create ROS object which connects to the locally running ROS
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
    }
    
    onConnection() {

    }

    onError() {

    }

    onClose() {

    }
   

    // // Create a listener for updates to the global /txt_msg parameter
    // let txt_listener = new ROSLIB.Topic({
    //     ros: ros,
    //     name: '/txt_msg',
    //     messageType: 'std_msgs/String'
    // });

    // // Subscribe to the txt_listener, update the msg field for every change
    // txt_listener.subscribe(function (m) {
    //     document.getElementById("msg").innerHTML = m.data;
    //     msg_send("You sent " + m.data);
    // });
    // txt_listener.subscribe(function (m) {
    //     document.getElementById("msg").innerHTML = m.data;
    //     msg_send("You sent " + m.data);
    // });

    // msg_send_publisher = new ROSLIB.Topic({
    //     ros: ros,
    //     name: "kinovagaze/msg_send",
    //     messageType: 'std_msgs/String'
    // });

    // msg_send = function (text) {
    //     let string = new ROSLIB.Message({
    //         data: text
    //     });
    //     msg_send_publisher.publish(string);
    // }

    setTimeout = function (timeout) {
        let timeoutMsg = new ROSLIB.Message({
            data: timeout
        })
        this.timeout_publisher.publish(timeoutMsg)
    }

    fingerPositionSet = function (finger1, finger2, finger3) {
        let finger_position = new ROSLIB.Message({
            finger1: finger1,
            finger2: finger2,
            finger3: finger3
        });
        this.finger_position_set_publisher.publish(finger_position);
    }

    closeFingers() {
        this.fingerPositionSet(100, 100, 100);
    }

    openFingers() {
        this.fingerPositionSet(0, 0, 0);
    }

    returnHome = function () {
        this.return_home_publisher.publish(new ROSLIB.Message());
    }

    getToolPose = function () {
        this.get_tool_pose_publisher.publish(new ROSLIB.Message());
    }

    stopMovement = function () {
        this.stop_movement_publisher.publish(new ROSLIB.Message());
    }

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

    positionForward = function () {
        this.toolMoveAbsolute(0.2, -0.6, 0.3, 90, 0, 0);
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