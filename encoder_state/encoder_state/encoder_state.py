import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String



class EncoderState(Node):

    def __init__(self):
        super().__init__('encoder_state')
        self.declare_parameter('port', '/dev/serial/by-path/pci-0000:03:00.0-usb-0:3.1:1.0')# Adjust the port and baudrate as needed
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        timer_period = 0.01  # seconds

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.write_serial,
            10)
        
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.node_callback)
        self.i = 0

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.serial_port = serial.Serial(port, baud, timeout=0.1)
        self.get_logger().info(f'Opened {port} @ {baud}')

        # Poll the port at 100 Hz without blocking the executor
        self.timer = self.create_timer(timer_period, self.read_serial)

    def read_serial(self):
        try:
            while self.serial_port.in_waiting:
                self.line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                self.data = self.line.split(',')

                if self.data[0] == 'ENCA' and len(self.data) == 10:
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = ['left_wheel_joint', 'right_wheel_joint']
                    msg.position = [float(self.data[1]), float(self.data[6])]
                    msg.velocity = [float(self.data[3]), float(self.data[8])]
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Publishing----> {msg.name[0]} position:{msg.position[0]} velocity:{msg.velocity[0]} | {msg.name[1]} Position:{msg.position[1]} Velocity:{msg.velocity[1]}")
                    self.i += 1

        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
        

    
    def destroy_node(self): 
        if self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()



    def write_serial(self, msg):
        DISTANCE_BETWEEN_WHEELS = 0.344  # Adjust this value based on your robot's specifications

        if msg.linear.x > 0:
            rightWheelVelocity = msg.linear.x
            leftWheelVelocity = msg.linear.x
        elif msg.angular.z > 0:
            rightWheelVelocity = msg.angular.z * DISTANCE_BETWEEN_WHEELS / 2
            leftWheelVelocity = -(msg.angular.z) * DISTANCE_BETWEEN_WHEELS / 2
        elif msg.angular.z < 0:
            rightWheelVelocity = -(msg.angular.z) * DISTANCE_BETWEEN_WHEELS / 2
            leftWheelVelocity = msg.angular.z * DISTANCE_BETWEEN_WHEELS / 2
        else:
            rightWheelVelocity = 0
            leftWheelVelocity = 0

        line = f'{rightWheelVelocity:.1f}, {leftWheelVelocity:.1f}\n'
        try:
            self.serial_port.write(line.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
        finally:
            self.get_logger().info('sending: left="%f", right="%f" to serial port' % (rightWheelVelocity, leftWheelVelocity))




def main(args=None):
    rclpy.init(args=args)
    node = EncoderState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()

