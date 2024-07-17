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
import socket
import time
import struct

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
URL_data = "BEG"
URR_data = "BEG"

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
    rospy.init_node('listener', anonymous=True)
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
    'ping': 0,
    'URL' : URL_data,
    'URR' : URR_data
}

# Flask route to send the metrics to the frontend
@app.route('/metrics', methods=['GET'])
def get_metrics():
    global battery
    global URL_data
    global URR_data
    with battery_lock:
        battery_value = battery
    data['battery'] = battery_value
    data['URL'] = URL_data
    data['URR'] = URR_data
    print(str(data))
    result = ping_func('roscore')
    if result is not None:
        data['ping'] = result
    else:
        data['ping'] = 999999
    return jsonify(data)

def start_flask_server():
    app.run(host=LOCAL_IP, port=5678)
    
def signal_handler(sig, frame):
    print('Received Ctrl+C, shutting down...')
    shutdown_event.set()
    rospy.signal_shutdown('Shutting down')
    sys.exit(0)

class URServer:
    def __init__(self, host, port, left):
        self.host = host
        self.port = port
        self.url_data = 'OFFLINE'
        self.running = True
        self.left = left

    def start_server(self):
        if self.left == False:
            global URR_data
        else:
            global URL_data
        print("Started UR Server!")
        
        while self.running:
            time.sleep(3)
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.connect((self.host, self.port))
                print("Connected to server")
                
                while self.running:
                    try:
                        data = s.recv(4096)
                        if not data:
                            raise ConnectionError("No data received, connection might be lost.")
                        i = 0
                        package_length = (struct.unpack('!i', data[0:4]))[0]
                        package_type = (struct.unpack('!B', data[4:5]))[0]
                        if package_type == 16:
                            while i + 5 < package_length:
                                message_length = (struct.unpack('!i', data[5 + i:9 + i]))[0]
                                message_type = (struct.unpack('!B', data[9:10]))[0]
                                if message_type == 0:
                                    sub_package = data[5:i + message_length]
                                    is_program_running = struct.unpack('!?', sub_package[18:19])[0]
                                    is_protective_stop = struct.unpack('!?', sub_package[17:18])[0]
                                    is_robot_on = struct.unpack('!?', sub_package[15:16])[0]
                                    
                                    if self.left == False:
                                        URR_data = str(is_robot_on) + '/' + str(is_protective_stop) + '/' + str(is_program_running)
                                        print('UR_R PWR ON/STOP/PROG RUN : ' + URR_data)
                                        time.sleep(3)
                                    else:
                                        URL_data = str(is_robot_on) + '/' + str(is_protective_stop) + '/' + str(is_program_running)
                                        print('UR_L PWR ON/STOP/PROG RUN : ' + URL_data)
                                        time.sleep(3)
                                    
                                i += message_length + 1
                    except (ConnectionError, struct.error, socket.error) as e:
                        print("Connection lost or data error:", e)
                        break  # Exit inner loop to attempt reconnect

            except (socket.error, ConnectionError) as e:
                print("Failed to connect or connection lost:", e)

            if self.left == False:
                URR_data = 'OFFLINE'
                print("URR OFFLINE, attempting to reconnect in 5 seconds...")
            else:
                URL_data = 'OFFLINE'
                print("URL OFFLINE, attempting to reconnect in 5 seconds...")
            
            time.sleep(5)  # Wait before reconnecting

    def get_data(self):
        return self.url_data

    def stop(self):
        self.running = False

    
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    asyncio.run(wait_for_ros_master())
    flask_thread = threading.Thread(target=start_flask_server)
    flask_thread.start()
    URL_server = URServer('192.168.12.89', 30002, left=True)
    URL_server_thread = threading.Thread(target=URL_server.start_server)
    URL_server_thread.start()
    URR_server = URServer('192.168.12.90', 30002, left=False)
    URR_server_thread = threading.Thread(target=URR_server.start_server)
    URR_server_thread.start()
    while True:
        if master_reachable == True:
            listener()
            break
    shutdown_event.wait()
    flask_thread.join()