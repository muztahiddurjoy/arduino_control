import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelToPWMNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_pwm_node')
        
        # Initialize serial communication with Arduino
        self.serial_port = '/dev/tty40' 
        self.baud_rate = 9600
        self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        
        
        self.wheel_separation = 0.3  
        self.wheel_radius = 0.05    
        
        self.max_linear_vel = 1.0
        self.max_angular_vel = 2.0
        
        
        self.max_pwm = 255
        self.min_pwm = -255
        
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("CmdVelToPWMNode initialized and subscribed to /cmd_vel")

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        v_left = linear_vel - (angular_vel * self.wheel_separation / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_separation / 2.0)


        left_pwm = self.velocity_to_pwm(v_left)
        right_pwm = self.velocity_to_pwm(v_right)


        self.send_pwm_to_arduino(left_pwm, right_pwm)

    def velocity_to_pwm(self, velocity):

        velocity = max(-self.max_linear_vel, min(self.max_linear_vel, velocity))


        pwm = int((velocity / self.max_linear_vel) * self.max_pwm)
        pwm = max(self.min_pwm, min(self.max_pwm, pwm))  # Clamp to PWM range

        return pwm

    def send_pwm_to_arduino(self, left_pwm, right_pwm):

        command = f"L{left_pwm} R{right_pwm}\n"
        try:
            self.arduino.write(command.encode())
            self.get_logger().info(f"Sent to Arduino: {command.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send data to Arduino: {e}")

    def destroy_node(self):

        if self.arduino.is_open:
            self.arduino.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPWMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
