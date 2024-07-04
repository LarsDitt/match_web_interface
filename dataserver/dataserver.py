from flask import Flask, jsonify
from std_msgs.msg import Float32
from urllib.parse import urlparse
import os
import rospy
import threading
import signal
import sys
import asyncio
import subprocess
import re

######################################## CONFIGURATION ########################################
# Set ROS_MASTER_URI to the IP of the Raspberry Pi
# Set LOCAL_IP to the IP of the machine running the dataserver
# Do not use localhost!

RASPI_ROS_MASTER_IP = 'http://10.145.8.93:11311/'
LOCAL_IP = '10.145.8.85'

###############################################################################################

os.environ['ROS_MASTER_URI'] = RASPI_ROS_MASTER_IP
app = Flask(__name__)
battery = 0
battery_lock = threading.Lock()
shutdown_event = threading.Event()
master_reachable = False

# Function uses ping command to check if a host is reachable and measures the round trip time
# Libary pythonping returned wrong results, so we use subprocess to call the ping command
def ping_func(host):
    try:
        result = subprocess.run(['ping', '-c', '1', host], capture_output=True, text=True)
        if result.returncode == 0:
            match = re.search(r'(?:time|Zeit)=([\d\.]+)\s*ms', result.stdout)
            print(match)
            if match:
                return float(match.group(1))
            else:
                return None
        else:
            return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

# ROS callback to update the battery status
def callback(data):
    global battery
    print(battery)
    with battery_lock:
        battery = data.data
            
# ROS listener for the battery status
def listener():
    rospy.init_node('dataserver_listener', anonymous=True)
    rospy.Subscriber("/bms_status/SOC", Float32, callback)
    rospy.loginfo("Started listener!")
    rospy.spin()
       
# Functions to check if the Raspberry Pi is reachable
async def wait_for_ros_master():
    global master_reachable
    await wait_for_network_connection()
    if master_reachable:
        print("Network connection established. Proceeding with dataserver startup.")
    else:
        print("Failed to establish network connection.")

async def wait_for_network_connection():
    global master_reachable
    loop = asyncio.get_event_loop()
    while True:
        try:
            ping_time = await loop.run_in_executor(None, ping_func, urlparse(RASPI_ROS_MASTER_IP).hostname)
            if ping_time is not None:
                master_reachable = True
                return
        except Exception as e:
            print(f"Error occurred while pinging ROS Master: {e}")
        await asyncio.sleep(1)

data = {
    'battery': battery,
    'ping': 0
}

# Flask route to send the metrics to the frontend
@app.route('/metrics', methods=['GET'])
def get_metrics():
    global battery
    with battery_lock:
        battery_value = battery
    data['battery'] = battery_value
    result = ping_func('roscore')
    if result is not None:
        data['ping'] = result
    else:
        data['ping'] = 999999
    return jsonify(data)

def start_flask_server():
    app.run(host=LOCAL_IP, port=5678)
    
# Signal handler to shut down the dataserver
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
        if master_reachable == True:
            listener()
            break
    shutdown_event.wait()
    flask_thread.join()
