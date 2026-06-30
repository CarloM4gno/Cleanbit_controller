#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import transforms3d as t3d
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState, Imu

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # Serial connection to Arduino
        self.ser = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_55930343536351510132-if00', 115200, timeout=0.1)
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)

        # Tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber to /cmd_vel
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Timer for serial read
        # NOTA: 20Hz invece di 50Hz — il Mega manda al massimo a 20Hz (IMU),
        # leggere troppo spesso non serve, leggiamo tutte le righe in coda
        # ad ogni chiamata cosi' non si accumula ritardo nel buffer seriale.
        self.timer = self.create_timer(0.05, self.read_serial)  # 20 Hz

        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def cmd_vel_callback(self, msg: Twist):
        """
        Sends vel commands to Arduino.
        Protocol: "V {v_lin} {v_ang}\n"
        """
        command = f"V {msg.linear.x:.3f} {msg.angular.z:.3f}\n"
        self.ser.write(command.encode())
        self.get_logger().debug(f"Comando inviato: {command.strip()}")

    def read_serial(self):
        """
        Svuota tutto il buffer seriale disponibile ad ogni chiamata,
        cosi' non accumuliamo ritardo se il Mega manda dati piu'
        velocemente di quanto il timer ROS interroghi la seriale.

        Righe gestite:
        - "O x y th"               -> odometria
        - "J pos_left pos_right"   -> joint states (radianti)
        - "I ax ay az gx gy gz"    -> dati IMU grezzi
        """
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                continue
            self.process_line(line)

    def process_line(self, line):
        try:
            if line.startswith("O"):
                _, x, y, th = line.split()
                self.x = float(x)
                self.y = float(y)
                self.th = float(th)
                self.publish_odom()

            elif line.startswith("J"):
                _, pos_left, pos_right = line.split()
                self.publish_joint_states(float(pos_left), float(pos_right))

            elif line.startswith("I"):
                _, ax, ay, az, gx, gy, gz = line.split()
                self.publish_imu(
                    float(ax), float(ay), float(az),
                    float(gx), float(gy), float(gz)
                )

        except Exception as e:
            self.get_logger().warn(f"Serial parsing error: {e}, line: {line}")

    def publish_odom(self):
        # Odometry msg
        odom = Odometry()
        now = self.get_clock().now().to_msg()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "body_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = t3d.euler.euler2quat(0, 0, self.th)
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.pose.pose.orientation.w = q[0]

        self.odom_pub.publish(odom)

        # TF odom -> body_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "body_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def publish_joint_states(self, pos_left, pos_right):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["left_wheel_joint", "right_wheel_joint"]
        js.position = [pos_left, pos_right]
        self.joint_pub.publish(js)

    def publish_imu(self, ax, ay, az, gx, gy, gz):
        """
        Pubblica i dati grezzi IMU.
        - Accelerazione: convertita da [g] a [m/s^2] (Arduino manda in g)
        - Velocita' angolare: convertita da [deg/s] a [rad/s]
        ROS si aspetta SI units: m/s^2 e rad/s.
        """
        G_TO_MS2 = 9.80665
        DEG_TO_RAD = 0.017453292519943295

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        msg.linear_acceleration.x = ax * G_TO_MS2
        msg.linear_acceleration.y = ay * G_TO_MS2
        msg.linear_acceleration.z = az * G_TO_MS2

        msg.angular_velocity.x = gx * DEG_TO_RAD
        msg.angular_velocity.y = gy * DEG_TO_RAD
        msg.angular_velocity.z = gz * DEG_TO_RAD

        # Orientamento non disponibile da MPU6050 grezzo (serve fusion)
        msg.orientation_covariance[0] = -1.0

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()