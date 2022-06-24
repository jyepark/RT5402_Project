const ros = new ROSLIB.Ros();
ros.connect('ws://localhost:9090');

ros.on('error', (error) => {
    console.log(error); 
});

ros.on('connection', (error) => {
    console.log('Connection: ok!');
});

ros.on('close', (error) => {
    console.log('Connection closed.');
});



const topic1 = new ROSLIB.Topic({
    ros: ros,
    name: '/gui_table',
    messageType: 'std_msgs/String'
});

const topic2 = new ROSLIB.Topic({
    ros: ros,
    name: '/menu',
    messageType: 'std_msgs/String'
});


let msg1 = new ROSLIB.Message({
    data : ''
});

let msg2 = new ROSLIB.Message({
    data : ''
});

function publish() {
    msg1.data = document.getElementById('table').value
    msg2.data = document.getElementById('menu').value
    topic1.publish(msg1)
    topic2.publish(msg2)
}

function return_dish() {
    msg1.data = document.getElementById('table').value
    msg2.data = 'return'
    topic1.publish(msg1)
    topic2.publish(msg2)
}

window.onload = () => {
    topic1.subscribe((res) => {
        document.getElementById('subscribe').innerHTML = res.data
    })
    topic2.subscribe((res) => {
        document.getElementById('subscribe2').innerHTML = res.data
    })
}

