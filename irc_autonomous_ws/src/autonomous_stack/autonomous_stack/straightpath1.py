import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist
import pyrealsense2 as rs
import numpy as np
import pygame


global alpha
alpha = 0.01
#class KalmanFilter:
#    def __init__(self, process_variance, measurement_variance):
#     self.process_variance = process_variance
#        self.measurement_variance = measurement_variance
#        self.estimated_value = 0.0
#        self.error_covariance = 1.0

#    def update(self, measurement):
#        kalman_gain = self.error_covariance / (self.error_covariance + self.measurement_variance)
#        self.estimated_value += kalman_gain * (measurement - self.estimated_value)
#        self.error_covariance = (1 - kalman_gain) * self.error_covariance + self.process_variance
#        return self.estimated_value

class IMUVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')

        # Publishers
        self.turn_publisher = self.create_publisher(Int32, 'turn', 10)
        self.straight_path_publisher = self.create_publisher(Int32, 'straight_path', 10)
        self.distance = self.create_subscription(Int32, 'distance', self.distance_callback, 10)

       
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)
        self.pipeline.start(config)

        self.pitch = 0.0
        self.dt = 0.1
        self.gyro_drift_threshold = 0.01  

        self.deviation_threshold = 10.0
        self.turn_threshold = 90.0
        self.straight_path_deviation = 0.0

        self.distance = 0.0
        self.prev_pitch = 0

        # Kalman filter for pitch
        #self.pitch_kalman_filter = KalmanFilter(0.1, 0.1)

        # Pygame GUI
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("IMU Data Visualization")
        self.font = pygame.font.Font(None, 36)
        self.clock = pygame.time.Clock()
    

    def get_motion_data(self):
        """Get gyro and accelerometer data."""
        try:
            frames = self.pipeline.wait_for_frames()
            gyro_frame = frames.first_or_default(rs.stream.gyro)
            accel_frame = frames.first_or_default(rs.stream.accel)

            if gyro_frame and accel_frame:
                gyro_data = gyro_frame.as_motion_frame().get_motion_data()
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                return (gyro_data.x, gyro_data.y, gyro_data.z), (accel_data.x, accel_data.y, accel_data.z)
            return None, None
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            return None, None

    def publish_turn_data(self):
        msg = Int32()
        msg.data = 1
        self.turn_publisher.publish(msg)
        self.get_logger().info("Turn Detected!")

    def distance_callback(self, msg):
        self.distance = msg.data
        self.get_logger().info(f"Distance: {self.distance}")

    def check_straight_path(self, pitch_deg):
        """Check and reset deviation, and publish deviation when it occurs."""
        if abs(pitch_deg) < self.deviation_threshold:
            self.straight_path_deviation = 0.0
        else:
            self.straight_path_deviation = pitch_deg  


            msg = Int32()
            msg.data = int(self.straight_path_deviation)
            if self.distance > 170:
                self.straight_path_publisher.publish(msg)
                self.get_logger().info(f"Straight Path Deviation Published: {msg.data}")

    def run(self):
        running = True
        try:
            while rclpy.ok():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        #raise KeyboardInterrupt
                        running = False

                gyro_data, accel_data = self.get_motion_data()
                if gyro_data and accel_data:
                    # Extract gyro data for pitch (y-axis)
                    gyro_pitch_rate = gyro_data[1]  


                    if abs(gyro_pitch_rate) > self.gyro_drift_threshold:
                        self.pitch += gyro_pitch_rate * self.dt
                    # Convert to degrees
                    pitch_deg = np.degrees(self.pitch)
                    pitch_value = (alpha*self.prev_pitch) +((1-alpha)*pitch_deg)

                    # Check for straight path deviation
                    self.check_straight_path(pitch_value)

                    # Detect turn based on threshold
                    if abs(pitch_value) >= self.turn_threshold:
                        self.publish_turn_data()
                        self.pitch = 0.0  
                        self.prev_pitch = 0.0
                    else:
                        self.prev_pitch = self.pitch

                    # Pygame display
                    self.screen.fill((0, 0, 0))
                    pitch_text = self.font.render(f"Pitch: {pitch_value:.2f}°", True, (255, 255, 255))
                    deviation_text = self.font.render(f"Deviation: {self.straight_path_deviation:.2f}", True, (255, 255, 255))

                    self.screen.blit(pitch_text, (20, 50))
                    self.screen.blit(deviation_text, (20, 100))
                    # self.prev_pitch = pitch_value
                    pygame.display.flip()

                self.clock.tick(10)

        except KeyboardInterrupt:
            self.get_logger().info("Program interrupted.")
        finally:
            self.pipeline.stop()
            pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    node = IMUVisualizer()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
