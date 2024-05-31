#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pysabertooth import Sabertooth

class SabertoothDriverNode(Node):
    def __init__(self):
        super().__init__('sabertooth_driver_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Inicializa el controlador Sabertooth
        self.saber = Sabertooth('/dev/ttyACM0', baudrate=115200, address=128, timeout=0.1)

    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x  # Asume que este es el componente de velocidad que quieres usar
        angular_speed = msg.angular.z  # Asume que este es el componente de velocidad que quieres usar

        # Convierte las velocidades a valores que acepta el Sabertooth
    
        left_speed = self.convert_to_sabertooth_speed(linear_speed - angular_speed)
        right_speed = self.convert_to_sabertooth_speed(linear_speed + angular_speed) # Ruedas sin Encoder

        self.saber.drive(1, left_speed)
        self.saber.drive(2, right_speed)
    def convert_to_sabertooth_speed(self, speed):

        return max(min(int(speed * 50), 50), -50)

def main():
    rclpy.init()
    sabertooth_driver_node = SabertoothDriverNode()
    rclpy.spin(sabertooth_driver_node)
    sabertooth_driver_node.destroy_node()
    rclpy.shutdown()

if  __name__ == '__main__':
    main()

