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

        # =========================================================
        # Frame ROS
        # Catena corretta:
        #
        # odom -> base_link -> body_link -> laser_frame
        #
        # Questo bridge deve pubblicare SOLO:
        # odom -> base_link
        #
        # robot_state_publisher pubblica invece:
        # base_link -> body_link
        # body_link -> laser_frame
        # body_link -> ruote
        # =========================================================

        self.odom_frame = "odom"
        self.base_frame = "base_link"
        self.imu_frame = "imu_link"

        # =========================================================
        # Serial connection to Arduino Mega
        # =========================================================

        self.ser = serial.Serial(
            '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_55930343536351510132-if00',
            115200,
            timeout=0.1
        )

        # =========================================================
        # Publishers
        # =========================================================

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)

        # =========================================================
        # TF broadcaster
        # =========================================================

        self.tf_broadcaster = TransformBroadcaster(self)

        # =========================================================
        # Subscriber a /cmd_vel
        # =========================================================

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # =========================================================
        # Timer lettura seriale
        # 0.05 s = 20 Hz
        # =========================================================

        self.timer = self.create_timer(0.05, self.read_serial)

        # =========================================================
        # Variabili odometria
        # =========================================================

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Contatore righe seriali ricevute
        self.lines_received = 0

        self.get_logger().info("Arduino bridge avviato.")
        self.get_logger().info("Pubblico /odom con frame odom -> base_link.")
        self.get_logger().info("NON pubblico odom -> body_link.")

    def cmd_vel_callback(self, msg: Twist):
        # Non inverto qui angular.z.
        # Se J/L risultano invertiti dopo aver sistemato le TF,
        # allora si valuta se invertire il segno qui o nel firmware.
        command = f"V {msg.linear.x:.3f} {msg.angular.z:.3f}\n"

        try:
            self.ser.write(command.encode())
            self.get_logger().debug(f"Comando inviato: {command.strip()}")
        except Exception as e:
            self.get_logger().warn(f"Errore invio comando seriale: {e}")

    def read_serial(self):
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode(errors='ignore').strip()

                if not line:
                    continue

                self.lines_received += 1
                self.process_line(line)

        except Exception as e:
            self.get_logger().warn(f"Errore lettura seriale: {e}")

    def process_line(self, line):
        try:
            # =====================================================
            # O x y theta
            # Esempio:
            # O 0.120 0.030 1.570
            # =====================================================
            if line.startswith("O"):
                parts = line.split()

                if len(parts) != 4:
                    self.get_logger().warn(f"Riga odometria non valida: {line}")
                    return

                _, x, y, th = parts

                self.x = float(x)
                self.y = float(y)
                self.th = float(th)

                if self.x == 0.0 and self.y == 0.0 and self.th == 0.0:
                    self.get_logger().warn(
                        "Odometria ricevuta dal Mega ma x=y=th=0.0 — "
                        "seriale comunica ma gli encoder non stanno incrementando "
                        "oppure il robot e' davvero fermo.",
                        throttle_duration_sec=5.0
                    )

                self.publish_odom()

            # =====================================================
            # J pos_left pos_right
            # Esempio:
            # J 1.230 -1.240
            # =====================================================
            elif line.startswith("J"):
                parts = line.split()

                if len(parts) != 3:
                    self.get_logger().warn(f"Riga joint non valida: {line}")
                    return

                _, pos_left, pos_right = parts

                pos_left = float(pos_left)
                pos_right = float(pos_right)

                if pos_left == 0.0 and pos_right == 0.0:
                    self.get_logger().warn(
                        "Joint states a zero — controlla che gli encoder stiano contando.",
                        throttle_duration_sec=5.0
                    )

                self.publish_joint_states(pos_left, pos_right)

            # =====================================================
            # I ax ay az gx gy gz
            # Esempio:
            # I 0.01 0.02 1.00 0.0 0.0 0.0
            # =====================================================
            elif line.startswith("I"):
                parts = line.split()

                if len(parts) != 7:
                    self.get_logger().warn(f"Riga IMU non valida: {line}")
                    return

                _, ax, ay, az, gx, gy, gz = parts

                ax = float(ax)
                ay = float(ay)
                az = float(az)
                gx = float(gx)
                gy = float(gy)
                gz = float(gz)

                if ax == 0.0 and ay == 0.0 and az == 0.0:
                    self.get_logger().warn(
                        "IMU a zero su tutti e 3 gli assi accelerometro — "
                        "sospetto: MPU6050 non risponde su I2C.",
                        throttle_duration_sec=5.0
                    )

                self.publish_imu(ax, ay, az, gx, gy, gz)

            else:
                self.get_logger().debug(f"Riga seriale ignorata: {line}")

        except Exception as e:
            self.get_logger().warn(f"Serial parsing error: {e}, line: {line}")

    def publish_odom(self):
        now = self.get_clock().now().to_msg()

        # =========================================================
        # Quaternione da roll=0, pitch=0, yaw=self.th
        # transforms3d restituisce:
        # q = [w, x, y, z]
        # ROS vuole:
        # x, y, z, w
        # =========================================================

        q = t3d.euler.euler2quat(0.0, 0.0, self.th)

        # =========================================================
        # Messaggio Odometry
        # frame_id = odom
        # child_frame_id = base_link
        #
        # QUESTO È IL FIX PRINCIPALE.
        # Prima era body_link.
        # =========================================================

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.pose.pose.orientation.w = q[0]

        # Covarianze semplici ma non nulle.
        # Puoi regolarle meglio dopo.
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05

        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        # =========================================================
        # TF odom -> base_link
        #
        # QUESTO È IL FIX PRINCIPALE.
        # Prima pubblicavi:
        # odom -> body_link
        #
        # Ora pubblichi:
        # odom -> base_link
        #
        # Poi robot_state_publisher pubblica:
        # base_link -> body_link
        # body_link -> laser_frame
        # =========================================================

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def publish_joint_states(self, pos_left, pos_right):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        js.name = [
            "left_wheel_joint",
            "right_wheel_joint"
        ]

        js.position = [
            pos_left,
            pos_right
        ]

        js.velocity = []
        js.effort = []

        self.joint_pub.publish(js)

    def publish_imu(self, ax, ay, az, gx, gy, gz):
        G_TO_MS2 = 9.80665
        DEG_TO_RAD = 0.017453292519943295

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Attenzione:
        # nello Xacro attuale non abbiamo ancora creato imu_link.
        # Se vuoi evitare warning TF, puoi mettere base_link.
        # Per ora lo lascio imu_link perché era così nel tuo codice.
        msg.header.frame_id = self.imu_frame

        msg.linear_acceleration.x = ax * G_TO_MS2
        msg.linear_acceleration.y = ay * G_TO_MS2
        msg.linear_acceleration.z = az * G_TO_MS2

        msg.angular_velocity.x = gx * DEG_TO_RAD
        msg.angular_velocity.y = gy * DEG_TO_RAD
        msg.angular_velocity.z = gz * DEG_TO_RAD

        # Orientamento non fornito dall'IMU raw
        msg.orientation_covariance[0] = -1.0

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ArduinoBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
