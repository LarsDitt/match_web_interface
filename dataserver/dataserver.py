# !! PYTHON NEEDS NETWORK CAPABILITY TO USE THE PING LIBARY !!
# sudo setcap cap_net_raw+ep /usr/bin/python3.8

from flask import Flask, jsonify
from pythonping import ping
from std_msgs.msg import Float32
import os
import rospy
import threading
import signal
import sys

os.environ['ROS_MASTER_URI'] = 'http://10.145.8.93:11311/'
app = Flask(__name__)
battery = 0
battery_lock = threading.Lock()
shutdown_event = threading.Event()

def callback(data):
    global battery
    with battery_lock:
        battery = data.data
            
def listener():
    print("Started ROSPY")
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/bms_status/SOC", Float32, callback)
    rospy.spin()

data = {
    'battery': battery,
    'ping': 0
}

@app.route('/metrics', methods=['GET'])
def get_metrics():
    global battery
    with battery_lock:
        battery_value = battery
    result = ping('roscore', count=1)
    data['ping'] = result.rtt_avg_ms
    data['battery'] = battery_value
    return jsonify(data)

def start_flask_server():
    app.run(host='10.145.8.85', port=5678)
    
def signal_handler(sig, frame):
    print('Received Ctrl+C, shutting down...')
    shutdown_event.set()
    sys.exit(0)
    rospy.signal_shutdown('Shutting down')

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    flask_thread = threading.Thread(target=start_flask_server)
    flask_thread.start()
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    shutdown_event.wait()
    flask_thread.join()
