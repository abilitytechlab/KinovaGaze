// Create ROS object which connects to the locally running ROS
var ros = new ROSLIB.Ros({
    url: 'ws://kinovagaze.local:9090'
});

// Make and call a function to update the ROS status depending on the last update
ros.on('connection', function () {
    document.getElementById("status").innerHTML = "Connected";
});

ros.on('error', function (error) {
    document.getElementById("status").innerHTML = "Error";
});

ros.on('close', function () {
    document.getElementById("status").innerHTML = "Closed";
});

// Create a listener for updates to the global /txt_msg parameter
var txt_listener = new ROSLIB.Topic({
    ros: ros,
    name: '/txt_msg',
    messageType: 'std_msgs/String'
});

// Subscribe to the txt_listener, update the msg field for every change
txt_listener.subscribe(function (m) {
    document.getElementById("msg").innerHTML = m.data;
    msg_send("You sent " + m.data);
});
txt_listener.subscribe(function (m) {
    document.getElementById("msg").innerHTML = m.data;
    msg_send("You sent " + m.data);
});

msg_send_listener = new ROSLIB.Topic({
    ros: ros,
    name: "kinovagaze/msg_send",
    messageType: 'std_msgs/String'
});

msg_send = function (text) {
    var string = new ROSLIB.Message({
        data: text
    });
    msg_send_listener.publish(string);
}

finger_position_set_listener = new ROSLIB.Topic({
    ros: ros,
    name: "/kinovagaze/finger_position_set",
    messageType: 'kinova_msgs/FingerPosition'
});

finger_position_set = function (finger1, finger2, finger3) {
    var finger_position = new ROSLIB.Message({
        finger1: finger1,
        finger2: finger2,
        finger3: finger3
    });
    finger_position_set_listener.publish(finger_position);
}

fingerPositionSet = function(fingers) {
    finger_position_set(fingers[0], fingers[1], fingers[2])
}

async function closeFingers() {
    finger_position_set(100, 100, 100);
}

async function openFingers() {
    finger_position_set(0, 0, 0);
}

return_home_listener = new ROSLIB.Topic({
    ros: ros,
    name: "/kinovagaze/return_home",
    messageType: 'std_msgs/Empty'
});

return_home = function () {
    return_home_listener.publish(new ROSLIB.Message());
}

async function returnHome() {
    return_home();
}

get_tool_pose_listener = new ROSLIB.Topic({
    ros: ros,
    name: "/kinovagaze/get_tool_pose",
    messageType: 'std_msgs/Empty'
});

get_tool_pose = function () {
    get_tool_pose_listener.publish(new ROSLIB.Message());
}

async function getToolPose() {
    get_tool_pose();
}

stop_movement_listener = new ROSLIB.Topic({
    ros: ros,
    name: "/kinovagaze/stop_movement",
    messageType: 'std_msgs/Empty'
});

stop_movement = function () {
    stop_movement_listener.publish(new ROSLIB.Message());
}

async function stopMovement() {
    stop_movement();
}

tool_move_position_listener = new ROSLIB.Topic({
    ros: ros,
    name: "/kinovagaze/tool_move_position",
    messageType: "kinova_msgs/KinovaPose"
});

tool_move_position = function (x = 0, y = 0, z = 0, yaw = 0, pitch = 0, roll = 0) {
    console.log(x, y, z, yaw, pitch, roll);
    var position_axis = new ROSLIB.Message({
        X: x,
        Y: y,
        Z: z,
        ThetaX: yaw,
        ThetaY: pitch,
        ThetaZ: roll
    });
    tool_move_position_listener.publish(position_axis);
}

positionForward = function () {
    tool_move_position(0, -0.4, 0.5, 90, 0, 0);
}

tool_move_continuous_listener = new ROSLIB.Topic({
    ros: ros,
    name: "/kinovagaze/tool_move_continuous",
    messageType: 'kinova_msgs/PoseVelocity'
});

tool_move_continuous = function (x = 0, y = 0, z = 0, yaw = 0, pitch = 0, roll = 0) {
    console.log(x, y, z, yaw, pitch, roll)
    var move_axis = new ROSLIB.Message({
        twist_linear_x: x,
        twist_linear_y: y,
        twist_linear_z: z,
        twist_angular_x: yaw,
        twist_angular_y: pitch,
        twist_angular_z: roll
    });
    console.log(move_axis);
    tool_move_continuous_listener.publish(move_axis);
}

toolMoveContinuous = function(axis) {
    tool_move_continuous(axis[0], axis[1], axis[2], axis[3], axis[4], axis[5])
}

left = function () {
    tool_move_continuous(1, 0, 0, 0, 0, 0);
}

right = function () {
    tool_move_continuous(-1, 0, 0, 0, 0, 0);
}

forward = function () {
    tool_move_continuous(0, -1, 0, 0, 0, 0);
}

back = function () {
    tool_move_continuous(0, 1, 0, 0, 0, 0);
}

up = function () {
    tool_move_continuous(0, 0, 1, 0, 0, 0);
}

down = function () {
    tool_move_continuous(0, 0, -1, 0, 0, 0);
}

pitchUp = function () {
    tool_move_continuous(0, 0, 0, -1, 0, 0);
}

pitchDown = function () {
    tool_move_continuous(0, 0, 0, 1, 0, 0);
}

rollClockwise = function () {
    tool_move_continuous(0, 0, 0, 0, 0, 1);
}

rollCounterClockwise = function () {
    tool_move_continuous(0, 0, 0, 0, 0, -1);
}

yawLeft = function () {
    tool_move_continuous(0, 0, 0, 0, 1, 0);
}

yawRight = function () {
    tool_move_continuous(0, 0, 0, 0, -1, 0);
}