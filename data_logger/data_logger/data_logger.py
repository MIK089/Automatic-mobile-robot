from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data
from dotenv import load_dotenv
import rclpy
import os
import numpy as np
import message_filters
import math
import threading
import requests
import io
import json
import time, hmac, hashlib

load_dotenv()  # Load environment variables from .env file
HMAC_KEY = os.getenv("hmac_key")
API_KEY = os.getenv("api_key")
URL = "https://ingestion.edgeimpulse.com/api/training/data"


class DataLogger(Node):

    def __init__(self):
        super().__init__('data_logger')

        # Initialize target position as None
        self.lock = threading.Lock()
        self.target_position = None
        self.rows = []
        self.column_labels = None
        self.pos_threshold = 0.25
        self.yaw_threshold = 0.10 
        self.done = False  

        # Start a separate thread to handle live terminal input
        self.input_thread = threading.Thread(target=self.read_keyboard_input, daemon=True)
        self.input_thread.start()
        
        # qos =   # best effort; compatible with reliable publishers too
        self.scan_subscriber  = message_filters.Subscriber(self, LaserScan, '/scan',    qos_profile=qos_profile_sensor_data)
        self.odom_subscriber  = message_filters.Subscriber(self, Odometry,  '/odom_rf2o',    qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.scan_subscriber, self.odom_subscriber],
            queue_size=20,
            slop=0.1,                # max time difference (s) between the three messages
            allow_headerless=True,   
        )
        self.sync.registerCallback(self.universal_callback)
        self.i = 0  # Initialize a counter for the number of messages processed
        



    def read_keyboard_input(self):
        while rclpy.ok():
            try:
                text = input("Enter target x,y,theta (m,m,deg): ").strip()
                x, y, theta = map(float, text.split(','))
                print(f'{text}')
            except ValueError:
                print("Expected: x,y,theta")
                continue
            except EOFError:
                self.get_logger().error("stdin closed, keyboard input unavailable (are you using ros2 launch?)")
                break
            with self.lock:
                self.target_position = [x, y, np.radians(theta)]
                print(f"{self.target_position}")
            break  # Exit the loop after setting the target position



    def process_laserscan(self, scan_msg):
        # Process the LaserScan message
        laser_scan = [round(r, 2) if np.isfinite(r) and (r >= scan_msg.range_min and r <= scan_msg.range_max) else 0.0 for r in scan_msg.ranges]  # Replace NaN and Inf with 0.0
        # laser_scan = [round(r, 2) for r in scan_msg.ranges if not (r < scan_msg.range_min or r > scan_msg.range_max)]
        if len(laser_scan) < 400:
            laser_scan.extend([0.0] * (400 - len(laser_scan)))  # Pad with zeros if less than 400 valid ranges
        return laser_scan[:400]  # Return only the first 400 valid ranges



    def absolute_to_relative(self, absolute_position):
        # Convert absolute coordinates to relative coordinates
        absolute_pos = np.array([absolute_position[0], absolute_position[1], absolute_position[2]])
        target_position = self.target_position
        target_pos = np.array([target_position[0], target_position[1], target_position[2]])

        relative_pos = target_pos - absolute_pos

        orientation_matrix = np.array([[np.cos(absolute_position[2]), np.sin(absolute_position[2]), 0],
                                    [-np.sin(absolute_position[2]), np.cos(absolute_position[2]), 0],
                                    [0, 0, 1]])
        
        relative_coordinates = np.matmul(orientation_matrix, relative_pos.reshape(3, 1))
        relative_coordinates = relative_coordinates.flatten().tolist()

        relative_xcoordinate = round(relative_coordinates[0], 2)
        relative_ycoordinate = round(relative_coordinates[1], 2)
        relative_orientation = round(relative_coordinates[2], 2)

        return relative_xcoordinate, relative_ycoordinate, relative_orientation 



    def process_odometry(self, odom_msg, target_position):
        # Process the Odometry message
        orientation = odom_msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        position_x = odom_msg.pose.pose.position.x 
        position_y = odom_msg.pose.pose.position.y
        planar_orientation = yaw
        
        velocity_x = round(odom_msg.twist.twist.linear.x, 2) # round velocity to 2dp
        angular_velocity_z = round(odom_msg.twist.twist.angular.z, 2) # round angular velocity to 2dp

        absolute_position = [position_x, position_y, planar_orientation]
        relative_x, relative_y, relative_orientation = self.absolute_to_relative(absolute_position)  # Assuming target position is (0, 0, 0)

        return relative_x, relative_y, relative_orientation, velocity_x, angular_velocity_z



    def universal_callback(self, laser_msg, odom_msg):
        if self.done:
            return

        if self.target_position is None:
            self.get_logger().warn("No target set yet", throttle_duration_sec=5.0)
            return

        processed_laserscan = self.process_laserscan(laser_msg)
        relative_x, relative_y, relative_orientation, velocity_x, angular_velocity_z = self.process_odometry(odom_msg, self.target_position)
        
        if self.column_labels is None:
            self.column_labels = [{"name": f"LaserScan{i}", "units": "m"} for i in range(len(processed_laserscan))] + [
                            { "name": "relXcoord", "units": "m" },
                            { "name": "relYcoord", "units": "m" },
                            { "name": "relOrientation", "units": "radians" },
                            { "name": "Velx", "units": "m/s" },
                            { "name": "AngVelZ", "units": "rad/s" }]

        row = processed_laserscan + [relative_x, relative_y, relative_orientation, velocity_x, angular_velocity_z]
        self.rows.append(row)

        distance = math.hypot(relative_x, relative_y)
        reached = distance < self.pos_threshold and (
            self.yaw_threshold is None or abs(relative_orientation) < self.yaw_threshold)

        self.get_logger().info(f"rows={len(self.rows)} distance={distance:.2f} m",
                           throttle_duration_sec=2.0)

        
        if reached:
            self.done = True
            self.get_logger().info(f"Target reached, uploading {len(self.rows)} rows")
            self.upload_rows()
            self.shutdown_node()



    def upload_rows(self):
        # empty signature (all zeros). HS256 gives 32 byte signature, and we encode in hex, so we need 64 characters here
        # emptySignature = ''.join(['0'] * 64)

        data = {
            "protected": {
                "ver": "v1",
                "alg": "HS256",
                "iat": time.time() # epoch time, seconds since 1970
            },
            "signature": "0" * 64,
            "payload": {
                "device_name": "AutomaticMobileRobot01",
                "device_type": "AMR",
                "interval_ms": 100,
                "sensors": self.column_labels,
                "values": self.rows
            }
        }

        # encode in JSON
        encoded = json.dumps(data)

        # sign message
        signature = hmac.new(bytes(HMAC_KEY, 'utf-8'), msg = encoded.encode('utf-8'), digestmod = hashlib.sha256).hexdigest()

        # set the signature again in the message, and encode again
        data['signature'] = signature
        encoded = json.dumps(data)

        try:
            # and upload the file
            res = requests.post(url=URL,
                                data=encoded,
                                headers={
                                    'Content-Type': 'application/json',
                                    'x-file-name': 'AMR01',
                                    'x-api-key': API_KEY
                                })
        except request.RequestException as e:
            self.get_logger().error(f"Upload failed: {e}")
            return False
            
        if (res.status_code == 200):
            # assert len(sensors) == len(row), f"{len(sensors)} sensors vs {len(row)} values"
            self.get_logger().info(f"Uploaded {len(self.rows)} rows")
            return True
        self.get_logger().error(f"Upload failed: {res.status_code} {res.content}")
        return False    

    def shutdown_node(self):
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()