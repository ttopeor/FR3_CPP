

function robot_control(callback) {
    while(True)
    {

        callback(robot_state, period);
    }
}

let outter_value = 1;


robot_control((robot_state, period) => {
    outter_value = 2;
})