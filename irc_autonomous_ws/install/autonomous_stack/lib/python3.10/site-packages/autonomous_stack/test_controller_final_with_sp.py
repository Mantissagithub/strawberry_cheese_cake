#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
from std_msgs.msg import Int32
import sys

class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.linear_velocity = 1.0  # default linear velocity
        self.angular_velocity = 0.0
        self.distance_received = False
        self.direction_received = False
        self.turn_complete_received = False
        self.stop_received = False
        self.aligned = False
        self.distance_time = 0.0
        self.turn_complete_time = 0.0
        self.rover_aligned = False
        self.straight_path_no = False
        self.direction_subscriber = self.create_subscription(
            String, "direction", self.direction_callback, 10
        )
        self.distance_subscriber = self.create_subscription(
            Int32, "distance", self.distance_callback, 10
        )
        self.turn_complete_subscriber = self.create_subscription(
            Int32, "turn", self.turn_complete_callback, 10
        )
        self.stop_subscriber = self.create_subscription(
            String, "stop_command", self.stop_callback, 10
        )
        self.align_subscriber = self.create_subscription(
            Twist, "/align_publisher", self.align_callback, 10
        )
        self.straight_path_subscriber = self.create_subscription(
            Int32, "straight_path", self.straight_path_callback, 10
        )

        self.direction = None
        self.distance = 0
        self.deviation = 0.0
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cmd_vel = Twist()

    def align_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.aligned = True
        self.get_logger().info("Alignment topic received")

    def direction_callback(self, msg):
        self.direction_received = True
        self.direction = msg.data
        self.get_logger().info("Direction topic received")
        self.direction_time = self.get_clock().now().nanoseconds / 1e9

    def distance_callback(self, msg):
        self.distance_received = True
        self.distance_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info("Distance topic received")

    def turn_complete_callback(self, msg):
        self.turn_complete_received = True
        self.turn_complete_time = self.get_clock().now().nanoseconds / 1e9
        self.direction_received = False
        self.get_logger().info("Turn complete topic received")

    def stop_callback(self, msg):
        self.stop_received = True
        self.get_logger().info("Stop topic received")

    def straight_path_callback(self, msg):
        self.deviation = msg.data
        self.straight_path_no = True
        self.get_logger().info("Straight path topic received")

    def timer_callback(self):
        if self.stop_received:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.get_logger().info("STOPPED")

        elif self.aligned:
            self.linear_velocity = self.linear_velocity
            self.angular_velocity = self.angular_velocity
            self.aligned = False
            self.get_logger().info("Alignment is happening")

        elif self.turn_complete_received:
            self.linear_velocity = 1.0
            self.angular_velocity = 0.0
            self.turn_complete_received = False
            self.get_logger().info("Turn complete, now moving forward")

        elif self.direction_received:
            if self.get_clock().now().nanoseconds / 1e9 - self.direction_time < 3.0:
                self.linear_velocity = 0.0
                self.angular_velocity = 0.0
                self.get_logger().info("Waiting for direction stability")
            else:
                if self.direction == "Right":
                    self.linear_velocity = 0.0
                    self.angular_velocity = 1.0
                    self.get_logger().info("Turning Right")
                elif self.direction == "Left":
                    self.linear_velocity = 0.0
                    self.angular_velocity = -1.0
                    self.get_logger().info("Turning Left")

        elif self.distance_received:
            self.linear_velocity = 1.0
            self.angular_velocity = 0.0
            self.distance_received = False
            self.get_logger().info("Distance received")

        elif not (self.direction_received or self.aligned or self.distance_received or self.turn_complete_received) and self.straight_path_no:
            self.linear_velocity = 0.0
            #self.angular_velocity = float(self.deviation) 
            self.angular_velocity = -0.5 if self.deviation < 0 else 0.5
            self.straight_path_no = False
            self.get_logger().info(f"Following straight path with deviation: {self.deviation}")

        elif not self.straight_path_no: 
            self.linear_velocity = 1.0
            self.angular_velocity = 0.0


        self.cmd_vel.linear.x = self.linear_velocity
        self.cmd_vel.angular.z = self.angular_velocity
        self.cmd_vel_publisher.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
