from flask import Flask, jsonify
from pythonping import ping
from std_msgs.msg import Float32
import os
import rospy
import threading
import signal
import sys
import asyncio
from urllib.parse import urlparse

RASPI_ROS_MASTER_IP = 'http://10.145.8.91:11311/'
LOCAL_IP = '10.145.8.54'

os.environ['ROS_MASTER_URI'] = RASPI_ROS_MASTER_IP
app = Flask(__name__)
battery = 0
battery_lock = threading.Lock()
shutdown_event = threading.Event()
master_reachable = False

def callback(data):
    global battery
    with battery_lock:
        battery = data.data
            
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/bms_status/SOC", Float32, callback)
    rospy.spin()

async def wait_for_ros_master():
    await wait_for_network_connection()
    print("Network connection established. Proceeding with dataserver startup.")
    master_reachable = True
    
async def wait_for_network_connection():
    loop = asyncio.get_event_loop()
    while True:
        try:
            await loop.run_in_executor(None, ping, urlparse(RASPI_ROS_MASTER_IP).hostname, 1)
            return
        except Exception as e:
            print(f"Error occurred while pinging ROS Master: {e}")
            await asyncio.sleep(1)

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
    app.run(host=LOCAL_IP, port=5678)
    
def signal_handler(sig, frame):
    print('Received Ctrl+C, shutting down...')
    shutdown_event.set()
    rospy.signal_shutdown('Shutting down')
    sys.exit(0)
    
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    asyncio.run(wait_for_ros_master())
    flask_thread = threading.Thread(target=start_flask_server)
    flask_thread.start()
    while True:
        if master_reachable:
            listener()
            break
    shutdown_event.wait()
    flask_thread.join()
